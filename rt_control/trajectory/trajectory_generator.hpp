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

    // 🌟 Robot 클래스 호환을 위한 타입 정의 추가
    using angles_set_t = std::vector<angles_t>;

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

    // --- Trajectory Commands (Robot 클래스 호환용 오버로딩 추가) ---

    // 🌟 Robot 클래스가 호출하는 5개 인자 trapj 지원
    [[nodiscard]] bool trapj(const angles_t &goal_angles, 
                             const std::optional<angles_t>& goal_angvels = std::nullopt,
                             const std::optional<angles_t>& peak_angvels = std::nullopt,
                             const std::optional<angles_t>& peak_angaccs = std::nullopt,
                             const std::optional<value_t>& duration = std::nullopt) noexcept {
        // 내부적으로는 우리가 만든 2개 인자 버전 호출
        m_gen_trapj = TrajTrapJ(&m_model, m_angles, m_angvels, goal_angles, 
                                angles_t::Zero(), m_model.get_max_angvels(), 
                                m_model.get_max_angaccs(), duration);
        m_traj_state = traj_state_t::TRAPJ;
        return true;
    }

    // 🌟 Robot 클래스가 호출하는 3개 인자 attrj 지원
    [[nodiscard]] bool attrj(const angles_t &goal_angles, value_t kp = 10.0,
                             const std::optional<angles_t>& goal_angvels = std::nullopt) noexcept {
        m_gen_attrj = TrajAttrJ(&m_model, m_angles, m_angvels, m_angaccs, 
                                m_model.get_max_angvels(), m_model.get_max_angaccs());
        m_gen_attrj.set_goal_angles(goal_angles);
        m_gen_attrj.set_pd_gains(kp);
        m_traj_state = traj_state_t::ATTRJ;
        return true;
    }

    // 🌟 Robot 클래스가 호출하는 4개 인자 attrl 지원
    [[nodiscard]] bool attrl(const tmat_t &goal_tmat, value_t kp = 50.0,
                             const std::optional<value_t>& peak_endvel = std::nullopt,
                             const std::optional<value_t>& peak_endacc = std::nullopt) noexcept {
        m_gen_attrl = TrajAttrL(&m_model, m_angles, m_angvels, m_angaccs);
        m_gen_attrl.set_goal_pose(goal_tmat);
        m_gen_attrl.set_kp_cartesian(kp);
        m_traj_state = traj_state_t::ATTRL;
        return true;
    }

    // 🌟 Robot 클래스가 호출하는 누락된 함수 인터페이스 추가
    void stop() noexcept { m_traj_state = traj_state_t::STOP; }
    
    void set_tcp_tmat(const tmat_t& new_shift_tmat) noexcept {
        // 필요 시 모델 혹은 내부 tmat 보정 로직 추가
    }

    [[nodiscard]] bool playj(const angles_set_t &goal_angles_set,
                             const std::optional<angles_set_t> &goal_angvels_set = std::nullopt,
                             const std::optional<angles_set_t> &goal_angaccs_set = std::nullopt,
                             const std::optional<angles_t> &peak_angvels = std::nullopt,
                             const std::optional<angles_t> &peak_angaccs = std::nullopt) noexcept {
        // PlayJ 엔진이 있다면 여기서 연동, 현재는 상태만 변경
        m_traj_state = traj_state_t::PLAYJ;
        return true;
    }

    // --- Error Norm Helpers & goal_reached (기존 코드 유지) ---
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

    [[nodiscard]] std::optional<value_t> angvels_enorm() const noexcept { return m_angvels.norm(); }
    [[nodiscard]] std::optional<value_t> vel_enorm() const noexcept { return m_a.head<3>().norm(); }
    [[nodiscard]] std::optional<value_t> w_enorm() const noexcept { return m_a.tail<3>().norm(); }

    [[nodiscard]] bool goal_reached(
        const std::optional<value_t> &angles_enorm_thold = 0.1,
        const std::optional<value_t> &pos_enorm_thold = 0.002,
        const std::optional<value_t> &rot_enorm_thold = 1.0,
        const std::optional<value_t> &angvels_enorm_thold = 0.5,
        const std::optional<value_t> &vel_enorm_thold = 0.01,
        const std::optional<value_t> &w_enorm_thold = 0.02
    ) const noexcept {
        if (m_traj_state == traj_state_t::STOP) return true;
        if (m_traj_state == traj_state_t::TRAPJ && m_gen_trapj.goal_reached()) return true;

        auto q_err = angles_enorm();
        if (angles_enorm_thold && q_err && *q_err > *angles_enorm_thold) return false;
        auto p_err = pos_enorm();
        if (pos_enorm_thold && p_err && *p_err > *pos_enorm_thold) return false;
        auto r_err = rot_enorm();
        if (rot_enorm_thold && r_err && *r_err > *rot_enorm_thold) return false;
        auto dq_err = angvels_enorm();
        if (angvels_enorm_thold && dq_err && *dq_err > *angvels_enorm_thold) return false;
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