#pragma once

#include <iostream>
#include <optional>
#include <tuple>
#include <vector>
#include <Eigen/Dense>
#include <cmath>

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
    using angles_set_t = Eigen::Matrix<value_t, Eigen::Dynamic, 6>;
    using pos_t = Eigen::Matrix<value_t, 3, 1>;
    using w_t = Eigen::Matrix<value_t, 3, 1>;
    using rmat_t = Eigen::Matrix<value_t, 3, 3>;
    using tmat_t = Eigen::Isometry3d;
    using jmat_t = rt_control::model::RobotModel::jacobian_t;
    using a_t = Eigen::Matrix<value_t, 6, 1>;

  public:
    TrajGenerator() = default;

    TrajGenerator(const rt_control::model::RobotModel& robot_model,
                  const angles_t &start_angles,
                  const angles_t &start_angvels,
                  const angles_t &start_angaccs) noexcept
        : m_model(robot_model),
          m_angles(start_angles), 
          m_angvels(start_angvels),
          m_angaccs(start_angaccs),
          m_tcp_tmat(Eigen::Isometry3d::Identity()) {
        update_clip();
        update_subordinates();
        stop();
    }

  public:
    void initialize(const rt_control::model::RobotModel& robot_model,
                    const angles_t &q, 
                    const angles_t &dq, 
                    const angles_t &ddq) noexcept {
        m_model = robot_model;
        m_angles = q;
        m_angvels = dq;
        m_angaccs = ddq;
        m_traj_state = traj_state_t::STOP; 
        
        update_clip();
        update_subordinates();
    }

    /**
     * @brief 통합 게인 설정 (AttrJ, AttrL 공용)
     */
    void set_combined_kp(value_t kp) noexcept {
        // AttrJ(Joint) 게인 업데이트
        m_gen_attrj.set_kp(angles_t::Constant(kp));
        // AttrL(Cartesian) 게인 업데이트
        m_gen_attrl.set_combined_kp(kp);
    }

    void set_tcp_tmat(const tmat_t &new_tcp_tmat) noexcept {
        m_tcp_tmat = new_tcp_tmat;
        update_subordinates(); 
    }

    [[nodiscard]] tmat_t get_tcp_tmat() const noexcept {
        return m_tcp_tmat;
    }

  public:
    void update(const value_t &dt) noexcept {
        switch (traj_state()) {
        case traj_state_t::STOP:
            m_angvels.setZero();
            m_angaccs.setZero();
            break;
        case traj_state_t::STOPPING: update_stopping(dt); break;
        case traj_state_t::TRAPJ:    update_trapj(dt);    break;
        case traj_state_t::ATTRJ:    update_attrj(dt);    break;
        case traj_state_t::ATTRL:    update_attrl(dt);    break;
        case traj_state_t::PLAYJ:    update_playj(dt);    break;
        }

        update_clip();
        update_subordinates();
    }

  public:
    void stop() noexcept {
        m_gen_stop = TrajStop(&m_model, angles(), angvels(), angaccs());
        m_traj_state = traj_state_t::STOPPING;
    }

    [[nodiscard]] bool trapj(
        const angles_t &goal_angles,
        const std::optional<angles_t> &goal_angvels_opt = std::nullopt,
        const std::optional<angles_t> &peak_angvels_opt = std::nullopt,
        const std::optional<angles_t> &peak_angaccs_opt = std::nullopt,
        const std::optional<value_t> &duration = std::nullopt) noexcept 
    {
        angles_t goal_angvels = goal_angvels_opt.value_or(angles_t::Zero());
        angles_t peak_angvels = peak_angvels_opt.value_or(m_model.get_max_angvels());
        angles_t peak_angaccs = peak_angaccs_opt.value_or(m_model.get_max_angaccs());

        m_gen_trapj = TrajTrapJ(&m_model, angles(), angvels(), goal_angles, goal_angvels,
                                peak_angvels, peak_angaccs, duration);

        m_traj_state = traj_state_t::TRAPJ;
        return true;
    }

    [[nodiscard]] bool attrj(
        const angles_t &goal_angles, value_t kp_in,
        const std::optional<angles_t> &goal_angvels_opt = std::nullopt) noexcept 
    {
        angles_t goal_angvels = goal_angvels_opt.value_or(angles_t::Zero());
        
        m_gen_attrj = TrajAttrJ(&m_model, angles(), angvels(), angaccs(), 
                                m_model.get_max_angvels(), m_model.get_max_angaccs());
        
        m_gen_attrj.set_goal_angles(goal_angles);
        m_gen_attrj.set_goal_angvels(goal_angvels);
        
        set_combined_kp(kp_in); // 통합 게인 적용

        m_traj_state = traj_state_t::ATTRJ;
        return true;
    }

    [[nodiscard]] bool attrl(
        const tmat_t &goal_tmat, 
        const value_t &kp_combined = 50.0,
        const std::optional<value_t> &peak_endvel_opt = std::nullopt,
        const std::optional<value_t> &peak_endacc_opt = std::nullopt) noexcept 
    {
        value_t peak_endvel = peak_endvel_opt.value_or(0.5); 
        value_t peak_endacc = peak_endacc_opt.value_or(2.0);

        // 🌟 RobotModel로부터 AttrL 전용 가속도 리밋을 명시적으로 가져와서 전달
        m_gen_attrl = TrajAttrL(&m_model, angles(), angvels(), angaccs(),
                                peak_endvel, peak_endacc);

        m_gen_attrl.set_goal_pose(goal_tmat);
        set_combined_kp(kp_combined);

        m_traj_state = traj_state_t::ATTRL;
        return true;
    }

    [[nodiscard]] bool playj(
        const angles_set_t &goal_angles_set,
        const std::optional<angles_set_t> &goal_angvels_set = std::nullopt,
        const std::optional<angles_set_t> &goal_angaccs_set = std::nullopt,
        const std::optional<angles_t> &peak_angvels_opt = std::nullopt,
        const std::optional<angles_t> &peak_angaccs_opt = std::nullopt) noexcept 
    {
        angles_t peak_angvels = peak_angvels_opt.value_or(m_model.get_max_angvels());
        angles_t peak_angaccs = peak_angaccs_opt.value_or(m_model.get_max_angaccs());

        m_gen_playj = TrajPlayJ(&m_model, angles(), angvels(), angaccs(), goal_angles_set,
                                goal_angvels_set, goal_angaccs_set, 
                                peak_angvels, peak_angaccs);

        m_traj_state = traj_state_t::PLAYJ;
        return true;
    }

  public:
    [[nodiscard]] const traj_state_t &traj_state() const noexcept { return m_traj_state; }
    [[nodiscard]] const angles_t &angles() const noexcept { return m_angles; }
    [[nodiscard]] const angles_t &angvels() const noexcept { return m_angvels; }
    [[nodiscard]] const angles_t &angaccs() const noexcept { return m_angaccs; }
    [[nodiscard]] const tmat_t &tmat() const noexcept { return m_tmat; }
    [[nodiscard]] const jmat_t &jmat() const noexcept { return m_jmat; }
    [[nodiscard]] const a_t &a() const noexcept { return m_a; }

    [[nodiscard]] std::optional<angles_t> goal_angles() const noexcept {
        switch (traj_state()) {
        case traj_state_t::STOP:     return angles();
        case traj_state_t::STOPPING: return m_gen_stop.goal_angles();
        case traj_state_t::TRAPJ:    return m_gen_trapj.goal_angles();
        case traj_state_t::ATTRJ:    return m_gen_attrj.goal_angles();
        default:                     return std::nullopt;
        }
    }

    [[nodiscard]] std::optional<tmat_t> goal_tmat() const noexcept {
        if (traj_state() == traj_state_t::ATTRL) return m_gen_attrl.goal_pose();
        const auto gangles = goal_angles();
        if (!gangles.has_value()) return std::nullopt;
        return m_model.forward_kinematics(gangles.value()) * m_tcp_tmat;
    }

    // --- Enorm (Error Norm) 계산기 ---
    [[nodiscard]] std::optional<value_t> angles_enorm() const noexcept {
        auto g_angles = goal_angles();
        if (!g_angles.has_value()) return std::nullopt;
        return (m_angles - g_angles.value()).norm();
    }

    [[nodiscard]] std::optional<value_t> pos_enorm() const noexcept {
        if (m_traj_state != traj_state_t::ATTRL) return std::nullopt;
        return (m_tmat.translation() - m_gen_attrl.goal_pose().translation()).norm();
    }

    [[nodiscard]] std::optional<value_t> rot_enorm() const noexcept {
        if (m_traj_state != traj_state_t::ATTRL) return std::nullopt;
        Eigen::AngleAxisd diff(m_tmat.linear() * m_gen_attrl.goal_pose().linear().transpose());
        return std::abs(diff.angle()) * (180.0 / M_PI);
    }

    [[nodiscard]] std::optional<value_t> angvels_enorm() const noexcept { return m_angvels.norm(); }
    [[nodiscard]] std::optional<value_t> vel_enorm() const noexcept { return m_a.head<3>().norm(); }
    [[nodiscard]] std::optional<value_t> w_enorm() const noexcept { return m_a.tail<3>().norm() * (180.0 / M_PI); }

    // --- Goal Reached 판정 ---
    [[nodiscard]] constexpr bool goal_reached(
        const std::optional<value_t> &angles_enorm_thold = 2.0,
        const std::optional<value_t> &pos_enorm_thold = 0.002,
        const std::optional<value_t> &rot_enorm_thold = 3.0,
        const std::optional<value_t> &angvels_enorm_thold = 4.0,
        const std::optional<value_t> &vel_enorm_thold = 0.004,
        const std::optional<value_t> &w_enorm_thold = 6.0) const noexcept 
    {
        if (m_traj_state == traj_state_t::STOP) return true;
        auto reached = false;

        if (angles_enorm_thold.has_value()) {
            const auto trg = angles_enorm();
            if (trg.has_value() && trg.value() > angles_enorm_thold.value()) return false;
            reached = true;
        }
        if (pos_enorm_thold.has_value()) {
            const auto trg = pos_enorm();
            if (trg.has_value() && trg.value() > pos_enorm_thold.value()) return false;
            reached = true;
        }
        if (rot_enorm_thold.has_value()) {
            const auto trg = rot_enorm();
            if (trg.has_value() && trg.value() > rot_enorm_thold.value()) return false;
            reached = true;
        }
        if (angvels_enorm_thold.has_value()) {
            const auto trg = angvels_enorm();
            if (trg.has_value() && trg.value() > angvels_enorm_thold.value()) return false;
            reached = true;
        }
        if (vel_enorm_thold.has_value()) {
            const auto trg = vel_enorm();
            if (trg.has_value() && trg.value() > vel_enorm_thold.value()) return false;
            reached = true;
        }
        if (w_enorm_thold.has_value()) {
            const auto trg = w_enorm();
            if (trg.has_value() && trg.value() > w_enorm_thold.value()) return false;
            reached = true;
        }
        return reached;
    }

  protected:
    void update_stopping(const value_t &dt) noexcept {
        m_gen_stop.update(dt);
        m_angles = m_gen_stop.angles(); m_angvels = m_gen_stop.angvels(); m_angaccs = m_gen_stop.angaccs();
    }
    void update_trapj(const value_t &dt) noexcept {
        m_gen_trapj.update(dt);
        m_angles = m_gen_trapj.angles(); m_angvels = m_gen_trapj.angvels(); m_angaccs = m_gen_trapj.angaccs();
    }
    void update_attrj(const value_t &dt) noexcept {
        m_gen_attrj.update(dt);
        m_angles = m_gen_attrj.angles(); m_angvels = m_gen_attrj.angvels(); m_angaccs = m_gen_attrj.angaccs();
    }
    void update_attrl(const value_t &dt) noexcept {
        m_gen_attrl.update(dt);
        m_angles = m_gen_attrl.angles(); m_angvels = m_gen_attrl.angvels(); m_angaccs = m_gen_attrl.angaccs();
    }
    void update_playj(const value_t &dt) noexcept {
        m_gen_playj.update(dt);
        m_angles = m_gen_playj.angles(); m_angvels = m_gen_playj.angvels(); m_angaccs = m_gen_playj.angaccs();
    }

    void update_clip() noexcept {
        m_angles = m_angles.cwiseMax(m_model.get_min_angles()).cwiseMin(m_model.get_max_angles());
        m_angvels = m_angvels.cwiseMax(m_model.get_min_angvels()).cwiseMin(m_model.get_max_angvels());
        m_angaccs = m_angaccs.cwiseMax(m_model.get_min_angaccs()).cwiseMin(m_model.get_max_angaccs());
    }

    void update_subordinates() noexcept {
        m_tmat = m_model.forward_kinematics(m_angles) * m_tcp_tmat;
        angles_t q_rad = m_angles * rt_control::model::RobotModel::DEG2RAD;
        m_jmat = m_model.jacobian(q_rad);
        m_a = m_jmat * (m_angvels * rt_control::model::RobotModel::DEG2RAD);
        m_a_d1 = m_jmat * (m_angaccs * rt_control::model::RobotModel::DEG2RAD);
    }

  protected:
    rt_control::model::RobotModel m_model;
    traj_state_t m_traj_state = traj_state_t::STOP;
    
    TrajStop m_gen_stop;
    TrajTrapJ m_gen_trapj;
    TrajAttrJ m_gen_attrj;
    TrajAttrL m_gen_attrl;
    TrajPlayJ m_gen_playj;

    angles_t m_angles = angles_t::Zero();
    angles_t m_angvels = angles_t::Zero();
    angles_t m_angaccs = angles_t::Zero();
    
    tmat_t m_tcp_tmat = tmat_t::Identity(); 
    tmat_t m_tmat = tmat_t::Identity();
    jmat_t m_jmat = jmat_t::Zero();
    a_t m_a = a_t::Zero();
    a_t m_a_d1 = a_t::Zero();
};

} // namespace trajectory
} // namespace rt_control