#pragma once

#include <iostream>
#include <optional>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

#include "../model/model.hpp"
#include "trajectory_attrj.hpp"
#include "trajectory_attrl.hpp"
#include "trajectory_playj.hpp"
#include "trajectory_stop.hpp"
#include "trajectory_trapj.hpp"

namespace rt_control {
namespace trajectory {

enum class TrajState {
    STOP,
    STOPPING,
    TRAPJ,
    ATTRJ,
    ATTRL,
    PLAYJ,
};

class TrajGenerator {
  public:
    using traj_state_t = TrajState;
    using value_t = rt_control::value_t;
    using angles_t = rt_control::angles_t;
    using tmat_t = Eigen::Isometry3d;
    using a_t = Eigen::Matrix<value_t, 6, 1>;

  public:
    TrajGenerator() = default;

    void initialize(const rt_control::model::RobotModel& robot_model,
                    const angles_t &q, const angles_t &dq, const angles_t &ddq) noexcept {
        m_model = robot_model;
        m_angles = q; m_angvels = dq; m_angaccs = ddq;
        m_traj_state = traj_state_t::STOP; 
        update_subordinates();
    }

    void update(const value_t &dt) noexcept {
        switch (m_traj_state) {
        case traj_state_t::STOP:     m_angvels.setZero(); m_angaccs.setZero(); break;
        case traj_state_t::STOPPING: update_stopping(dt); break;
        case traj_state_t::TRAPJ:    update_trapj(dt);    break;
        case traj_state_t::ATTRJ:    update_attrj_step(dt);    break;
        case traj_state_t::ATTRL:    update_attrl_step(dt);    break;
        case traj_state_t::PLAYJ:    update_playj(dt);    break;
        }
        update_clip();
        update_subordinates();
    }

    // --- Trajectory Commands ---
    [[nodiscard]] bool trapj(const angles_t &goal_angles, 
                             std::optional<value_t> duration = std::nullopt) noexcept {
        m_gen_trapj = TrajTrapJ(&m_model, m_angles, m_angvels, goal_angles, 
                                angles_t::Zero(), m_model.get_max_angvels(), 
                                m_model.get_max_angaccs(), duration);
        m_traj_state = traj_state_t::TRAPJ;
        return true;
    }

    [[nodiscard]] bool attrj(const angles_t &goal_angles, value_t kp = 10.0) noexcept {
        m_gen_attrj = TrajAttrJ(&m_model, m_angles, m_angvels, m_angaccs, 
                                m_model.get_max_angvels(), m_model.get_max_angaccs());
        m_gen_attrj.set_goal_angles(goal_angles);
        m_gen_attrj.set_pd_gains(kp);
        m_traj_state = traj_state_t::ATTRJ;
        return true;
    }

    [[nodiscard]] bool attrl(const tmat_t &goal_tmat, value_t kp = 50.0) noexcept {
        m_gen_attrl = TrajAttrL(&m_model, m_angles, m_angvels, m_angaccs);
        m_gen_attrl.set_goal_pose(goal_tmat);
        m_gen_attrl.set_kp_cartesian(kp);
        m_traj_state = traj_state_t::ATTRL;
        return true;
    }

    // --- Error Norm Helpers ---
    [[nodiscard]] std::optional<value_t> angles_enorm() const noexcept {
        if (m_traj_state == traj_state_t::TRAPJ) return (m_gen_trapj.goal_angles() - m_angles).norm();
        if (m_traj_state == traj_state_t::ATTRJ) return (m_gen_attrj.goal_angles() - m_angles).norm();
        return std::nullopt;
    }

    [[nodiscard]] std::optional<value_t> pos_enorm() const noexcept {
        if (m_traj_state == traj_state_t::ATTRL) return (m_gen_attrl.goal_pose().translation() - m_tmat.translation()).norm();
        return std::nullopt;
    }

    [[nodiscard]] std::optional<value_t> rot_enorm() const noexcept {
        if (m_traj_state == traj_state_t::ATTRL) {
            Eigen::AngleAxisd err_aa(m_gen_attrl.goal_pose().linear() * m_tmat.linear().transpose());
            return std::abs(err_aa.angle()) * (180.0 / M_PI);
        }
        return std::nullopt;
    }

    [[nodiscard]] std::optional<value_t> angvels_enorm() const noexcept {
        return m_angvels.norm();
    }

    [[nodiscard]] std::optional<value_t> vel_enorm() const noexcept {
        return m_a.head<3>().norm(); // Cartesian 선속도
    }

    [[nodiscard]] std::optional<value_t> w_enorm() const noexcept {
        return m_a.tail<3>().norm(); // Cartesian 각속도
    }

    // --- Improved goal_reached ---
    [[nodiscard]] bool goal_reached(
        const std::optional<value_t> &angles_enorm_thold = 0.1,   // deg
        const std::optional<value_t> &pos_enorm_thold = 0.002,   // m (2mm)
        const std::optional<value_t> &rot_enorm_thold = 1.0,     // deg
        const std::optional<value_t> &angvels_enorm_thold = 0.5, // deg/s
        const std::optional<value_t> &vel_enorm_thold = 0.01,    // m/s
        const std::optional<value_t> &w_enorm_thold = 0.02       // rad/s
    ) const noexcept {
        
        if (m_traj_state == traj_state_t::STOP) return true;

        // TRAPJ의 경우 시간 기반 도달 판정을 기본으로 사용 가능
        if (m_traj_state == traj_state_t::TRAPJ && m_gen_trapj.goal_reached()) return true;

        // 1. 관절 각도 체크
        auto q_err = angles_enorm();
        if (angles_enorm_thold && q_err && *q_err > *angles_enorm_thold) return false;

        // 2. Cartesian 위치 체크
        auto p_err = pos_enorm();
        if (pos_enorm_thold && p_err && *p_err > *pos_enorm_thold) return false;

        // 3. Cartesian 회전 체크
        auto r_err = rot_enorm();
        if (rot_enorm_thold && r_err && *r_err > *rot_enorm_thold) return false;

        // 4. 관절 속도 체크
        auto dq_err = angvels_enorm();
        if (angvels_enorm_thold && dq_err && *dq_err > *angvels_enorm_thold) return false;

        // 5. Cartesian 선/각속도 체크
        auto v_err = vel_enorm();
        if (vel_enorm_thold && v_err && *v_err > *vel_enorm_thold) return false;
        
        auto w_err = w_enorm();
        if (w_enorm_thold && w_err && *w_err > *w_enorm_thold) return false;

        return true;
    }

    // --- Getters & Status ---
    [[nodiscard]] const angles_t& angles() const noexcept { return m_angles; }
    [[nodiscard]] const angles_t& angvels() const noexcept { return m_angvels; }
    [[nodiscard]] const angles_t& angaccs() const noexcept { return m_angaccs; }
    [[nodiscard]] const tmat_t& tmat() const noexcept { return m_tmat; }
    [[nodiscard]] value_t duration() const noexcept { return (m_traj_state == traj_state_t::TRAPJ) ? m_gen_trapj.duration() : 0.0; }
    [[nodiscard]] const std::vector<TrajTrapJ::Profile>& get_trapj_profiles() const { return m_gen_trapj.get_profiles(); }

  protected:
    void update_stopping(value_t dt) { m_gen_stop.update(dt); copy_state(m_gen_stop); }
    void update_trapj(value_t dt) { m_gen_trapj.update(dt); copy_state(m_gen_trapj); }
    void update_attrj_step(value_t dt) { (void)m_gen_attrj.update(dt); copy_state(m_gen_attrj); }
    void update_attrl_step(value_t dt) { (void)m_gen_attrl.update(dt); copy_state(m_gen_attrl); }
    void update_playj(value_t dt) { m_gen_playj.update(dt); copy_state(m_gen_playj); }

    template<typename T>
    void copy_state(const T& gen) { m_angles = gen.angles(); m_angvels = gen.angvels(); m_angaccs = gen.angaccs(); }

    void update_clip() noexcept {
        m_angles = m_angles.cwiseMax(m_model.get_min_angles()).cwiseMin(m_model.get_max_angles());
    }

    void update_subordinates() noexcept {
        m_tmat = m_model.forward_kinematics(m_angles);
        // m_a 업데이트 로직 필요 (Jacobian 기반)
    }

  protected:
    rt_control::model::RobotModel m_model;
    traj_state_t m_traj_state = traj_state_t::STOP;
    
    TrajStop m_gen_stop; TrajTrapJ m_gen_trapj; TrajAttrJ m_gen_attrj;
    TrajAttrL m_gen_attrl; TrajPlayJ m_gen_playj;

    angles_t m_angles, m_angvels, m_angaccs;
    tmat_t m_tmat;
    a_t m_a = a_t::Zero(); 
};

} // namespace trajectory
} // namespace rt_control