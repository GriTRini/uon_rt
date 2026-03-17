#pragma once

#include <iostream>
#include <optional>
#include <tuple>
#include <vector>
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
    void update(const value_t &dt) noexcept {
        switch (traj_state()) {
        case traj_state_t::STOP:
            m_angvels.setZero();
            m_angaccs.setZero();
            break;

        case traj_state_t::STOPPING:
            update_stopping(dt);
            break;

        case traj_state_t::TRAPJ:
            update_trapj(dt);
            break;

        case traj_state_t::ATTRJ:
            update_attrj(dt);
            break;

        case traj_state_t::ATTRL:
            update_attrl(dt);
            break;

        case traj_state_t::PLAYJ:
            update_playj(dt);
            break;
        }

        update_clip();
        update_subordinates();
    }

    //////////////////////////////////////////////////////////////

  public:
    void stop() noexcept {
        m_gen_stop = TrajStop(&m_model, angles(), angvels(), angaccs());
        m_traj_state = traj_state_t::STOPPING;
    }

    // [1] TRAPJ: 가속도 한계를 모델에서 가져옴
    [[nodiscard]] bool trapj(
        const angles_t &goal_angles,
        const std::optional<angles_t> &goal_angvels_opt = std::nullopt,
        const std::optional<angles_t> &peak_angvels_opt = std::nullopt,
        const std::optional<angles_t> &peak_angaccs_opt = std::nullopt,
        const std::optional<value_t> &duration = std::nullopt) noexcept 
    {
        angles_t goal_angvels = goal_angvels_opt.value_or(angles_t::Zero());
        
        // 🌟 모델 데이터로 연동
        angles_t peak_angvels = peak_angvels_opt.value_or(m_model.get_max_angvels());
        angles_t peak_angaccs = peak_angaccs_opt.value_or(m_model.get_max_angaccs());

        m_gen_trapj = TrajTrapJ(&m_model, angles(), angvels(), goal_angles, goal_angvels,
                                peak_angvels, peak_angaccs, duration);

        m_traj_state = traj_state_t::TRAPJ;
        return true;
    }

    // [2] ATTRJ: 가속도 한계를 모델에서 가져옴
    [[nodiscard]] bool attrj(
        const angles_t &goal_angles, const value_t &kp_in,
        const std::optional<angles_t> &goal_angvels_opt = std::nullopt) noexcept 
    {
        angles_t goal_angvels = goal_angvels_opt.value_or(angles_t::Zero());

        // TrajAttrJ 내부에서 m_model->get_max_angaccs()를 참조하도록 설계되어 있다면
        // 생성자 인자만 정확히 넘겨주면 됩니다.
        m_gen_attrj = TrajAttrJ(&m_model, angles(), angvels(), angaccs(), 
                                goal_angles, goal_angvels);
        
        angles_t kp_vec = angles_t::Constant(kp_in);
        m_gen_attrj.set_kp(kp_vec);

        m_traj_state = traj_state_t::ATTRJ;
        return true;
    }

    // [3] ATTRL: Task Space 가속도 설정
    [[nodiscard]] bool attrl(
        const tmat_t &goal_tmat, const value_t &kp_cartesian = 10.0,
        const std::optional<value_t> &peak_endvel_opt = std::nullopt,
        const std::optional<value_t> &peak_endacc_opt = std::nullopt) noexcept 
    {
        // 선속도 한계 (m/s), 선가속도 한계 (m/s^2)
        // 모델에 Task 한계가 정의되어 있지 않다면 안전한 기본값 사용
        value_t peak_endvel = peak_endvel_opt.value_or(0.5); 
        value_t peak_endacc = peak_endacc_opt.value_or(1.0); // 1.0 m/s^2 (부드러움)

        m_gen_attrl = TrajAttrL(&m_model, angles(), angvels(), angaccs(),
                                peak_endvel, M_PI, peak_endacc, M_PI*2);

        m_gen_attrl.set_goal_pose(goal_tmat);
        m_gen_attrl.set_kp_cartesian(kp_cartesian);

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

    //////////////////////////////////////////////////////////////

  public:
    [[nodiscard]] const traj_state_t &traj_state() const noexcept { return m_traj_state; }
    [[nodiscard]] const angles_t &angles() const noexcept { return m_angles; }
    [[nodiscard]] const angles_t &angvels() const noexcept { return m_angvels; }
    [[nodiscard]] const angles_t &angaccs() const noexcept { return m_angaccs; }
    [[nodiscard]] const tmat_t &tmat() const noexcept { return m_tmat; }
    [[nodiscard]] const jmat_t &jmat() const noexcept { return m_jmat; }
    [[nodiscard]] const a_t &a() const noexcept { return m_a; }
    [[nodiscard]] const a_t &a_d1() const noexcept { return m_a_d1; }

    [[nodiscard]] std::optional<angles_t> goal_angles() const noexcept {
        switch (traj_state()) {
        case traj_state_t::STOP:     return angles();
        case traj_state_t::STOPPING: return m_gen_stop.goal_angles();
        case traj_state_t::TRAPJ:    return m_gen_trapj.goal_angles();
        case traj_state_t::ATTRJ:    return m_gen_attrj.goal_angles();
        case traj_state_t::ATTRL:    return std::nullopt;
        case traj_state_t::PLAYJ:    return std::nullopt;
        }
        return std::nullopt;
    }

    [[nodiscard]] std::optional<angles_t> goal_angvels() const noexcept {
        return std::nullopt; 
    }

    [[nodiscard]] std::optional<tmat_t> goal_tmat() const noexcept {
        // 🌟 ATTRL 모드일 경우 새로운 Getter인 goal_pose() 호출
        if (traj_state() == traj_state_t::ATTRL) {
            return m_gen_attrl.goal_pose();
        }

        const auto gangles = goal_angles();
        if (!gangles.has_value()) {
            return std::nullopt;
        }
        return m_model.forward_kinematics(gangles.value()) * m_tcp_tmat;
    }

    [[nodiscard]] std::optional<a_t> goal_a() const noexcept {
        return std::nullopt;
    }

    [[nodiscard]] bool goal_reached() const noexcept {
        auto g_angles = goal_angles();
        if (g_angles.has_value()) {
            return (m_angles - g_angles.value()).norm() < 1.0; 
        }
        return false;
    }

  protected:
    void update_stopping(const value_t &dt) noexcept {
        m_gen_stop.update(dt);
        m_angles = m_gen_stop.angles();
        m_angvels = m_gen_stop.angvels();
        m_angaccs = m_gen_stop.angaccs();
    }

    void update_trapj(const value_t &dt) noexcept {
        m_gen_trapj.update(dt);
        m_angles = m_gen_trapj.angles();
        m_angvels = m_gen_trapj.angvels();
        m_angaccs = m_gen_trapj.angaccs();
    }

    void update_attrj(const value_t &dt) noexcept {
        m_gen_attrj.update(dt);
        m_angles = m_gen_attrj.angles();
        m_angvels = m_gen_attrj.angvels();
        m_angaccs = m_gen_attrj.angaccs();
    }

    void update_attrl(const value_t &dt) noexcept {
        m_gen_attrl.update(dt);
        m_angles = m_gen_attrl.angles();
        m_angvels = m_gen_attrl.angvels();
        m_angaccs = m_gen_attrl.angaccs();
    }

    void update_playj(const value_t &dt) noexcept {
        m_gen_playj.update(dt);
        m_angles = m_gen_playj.angles();
        m_angvels = m_gen_playj.angvels();
        m_angaccs = m_gen_playj.angaccs();
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

    angles_t m_angles;
    angles_t m_angvels;
    angles_t m_angaccs;
    
    tmat_t m_tcp_tmat; 
    tmat_t m_tmat;
    jmat_t m_jmat;
    a_t m_a;
    a_t m_a_d1;
};

} // namespace trajectory
} // namespace rt_control