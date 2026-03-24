#pragma once

#include "../core/core.hpp"
#include <Eigen/Geometry>
#include <array>
#include <iostream>
#include <optional>
#include <vector>
#include <cmath>

#include "../model/model.hpp"
#include "trajectory_IK/ik_solver.hpp"
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
    // DSR 스타일의 타입 Alias 유지
    using traj_state_t = TrajState;
    using value_t = rt_control::value_t;
    using angles_t = rt_control::angles_t;
    using pos_t = Eigen::Vector3d;
    using w_t = Eigen::Vector3d;
    using rmat_t = Eigen::Matrix3d;
    using tmat_t = Eigen::Isometry3d;
    using jmat_t = Eigen::Matrix<double, 6, 6>;
    using a_t = Eigen::Matrix<value_t, 6, 1>;
    using angles_set_t = std::vector<angles_t>;

  public:
    // 생성자: DSR의 초기화 로직 유지
    TrajGenerator() {
        m_tcp_offset = tmat_t::Identity();
        m_traj_state = traj_state_t::STOP;
        m_a.setZero();
        m_a_d1.setZero();
    }

    void initialize(const rt_control::model::RobotModel& robot_model,
                    const angles_t &start_angles, 
                    const angles_t &start_angvels, 
                    const angles_t &start_angaccs) noexcept {
        m_model = robot_model;
        m_angles = start_angles;
        m_angvels = start_angvels;
        m_angaccs = start_angaccs;

        update_clip();
        update_subordinates();
        stop(); // 초기 상태는 STOPPING/STOP
    }

    // --- Update Loop ---
    void update(const value_t &dt) noexcept {
        switch (m_traj_state) {
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

    // --- TCP Management ---
    void set_tcp(value_t x, value_t y, value_t z, value_t r_deg, value_t p_deg, value_t yaw_deg) noexcept {
        m_tcp_offset = tmat_t::Identity();
        m_tcp_offset.translation() << x, y, z;
        const value_t d2r = M_PI / 180.0;
        m_tcp_offset.linear() = (Eigen::AngleAxisd(yaw_deg * d2r, Eigen::Vector3d::UnitZ()) *
                                Eigen::AngleAxisd(p_deg * d2r, Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(r_deg * d2r, Eigen::Vector3d::UnitX())).toRotationMatrix();
        update_subordinates();
    }

    [[nodiscard]] tmat_t get_tcp_tmat() const noexcept {
        return m_tcp_offset;
    }

    // --- Trajectory Commands ---
    void stop() noexcept {
        m_gen_stop = TrajStop(angles(), angvels(), angaccs());
        m_traj_state = traj_state_t::STOPPING;
    }

    [[nodiscard]] bool trapj(const angles_t &goal_angles,
                             const angles_t &goal_angvels = angles_t::Zero(),
                             const std::optional<angles_t> &peak_angvels = std::nullopt,
                             const std::optional<angles_t> &peak_angaccs = std::nullopt,
                             const std::optional<value_t> &duration = std::nullopt) noexcept {
        m_gen_trapj = TrajTrapJ(&m_model, angles(), angvels(), goal_angles, goal_angvels,
                                peak_angvels.value_or(m_model.get_max_angvels()),
                                peak_angaccs.value_or(m_model.get_max_angaccs()), duration);
        m_traj_state = traj_state_t::TRAPJ;
        return true;
    }

    [[nodiscard]] bool attrl(const tmat_t &goal_tmat, const value_t &kp = 50,
                             const a_t &goal_a = a_t::Zero(),
                             const value_t &peak_endvel = 0.5, const value_t &peak_endangvel = 180) noexcept {
        m_gen_attrl = TrajAttrL(&m_model, angles(), angvels(), angaccs(), m_tcp_offset);
        m_gen_attrl.set_goal_pose(goal_tmat);
        m_gen_attrl.set_kp_cartesian(kp);
        // 필요 시 peak_endvel 등 설정 로직 추가
        m_traj_state = traj_state_t::ATTRL;
        return true;
    }

    // --- DSR 스타일 Kinematics Solvers ---
    [[nodiscard]] tmat_t solve_forward(const angles_t &angles) const noexcept {
        // forward_kinematics.hpp에 정의된 함수 호출
        auto [pose, J] = ik::compute_forward_and_jacobian(&m_model, angles, m_tcp_offset);
        return pose;
    }

    [[nodiscard]] auto solve_inverse(const angles_t &initial_angles, const tmat_t &target_tmat) const noexcept {
        // ik_solver.hpp의 IKSolver 클래스 사용
        return ik::IKSolver::solve(&m_model, target_tmat, initial_angles, m_tcp_offset, 100, 1e-4, 0.01);
    }

    // --- Status & Getters ---
    [[nodiscard]] const traj_state_t& traj_state() const noexcept { return m_traj_state; }
    [[nodiscard]] const angles_t& angles() const noexcept { return m_angles; }
    [[nodiscard]] const angles_t& angvels() const noexcept { return m_angvels; }
    [[nodiscard]] const angles_t& angaccs() const noexcept { return m_angaccs; }
    [[nodiscard]] const tmat_t& tmat() const noexcept { return m_tmat; }
    [[nodiscard]] const jmat_t& jmat() const noexcept { return m_jmat; }
    [[nodiscard]] const a_t& a() const noexcept { return m_a; }

    // --- DSR의 세밀한 Error & Threshold 로직 ---
    [[nodiscard]] std::optional<angles_t> goal_angles() const noexcept {
        if (m_traj_state == traj_state_t::STOP) return angles();
        if (m_traj_state == traj_state_t::STOPPING) return m_gen_stop.goal_angles();
        if (m_traj_state == traj_state_t::TRAPJ) return m_gen_trapj.goal_angles();
        return std::nullopt;
    }

    [[nodiscard]] std::optional<tmat_t> goal_tmat() const noexcept {
        if (m_traj_state == traj_state_t::ATTRL) return m_gen_attrl.goal_pose();
        auto gangles = goal_angles();
        if (gangles) return solve_forward(*gangles);
        return std::nullopt;
    }

    [[nodiscard]] std::optional<value_t> angles_enorm() const noexcept {
        auto gang = goal_angles();
        if (gang) return (*gang - m_angles).norm();
        return std::nullopt;
    }

    [[nodiscard]] std::optional<value_t> pos_enorm() const noexcept {
        auto gtmat = goal_tmat();
        if (gtmat) return (gtmat->translation() - m_tmat.translation()).norm();
        return std::nullopt;
    }

    [[nodiscard]] std::optional<value_t> rot_enorm() const noexcept {
        auto gtmat = goal_tmat();
        if (gtmat) {
            Eigen::AngleAxisd err_aa(gtmat->linear() * m_tmat.linear().transpose());
            return std::abs(err_aa.angle()) * (180.0 / M_PI);
        }
        return std::nullopt;
    }

    // DSR의 통합 goal_reached 판정 로직
    [[nodiscard]] bool goal_reached(
        const std::optional<value_t> &angles_enorm_thold = 2.0,
        const std::optional<value_t> &pos_enorm_thold = 0.002,
        const std::optional<value_t> &rot_enorm_thold = 3.0,
        const std::optional<value_t> &angvels_enorm_thold = 4.0,
        const std::optional<value_t> &vel_enorm_thold = 0.004,
        const std::optional<value_t> &w_enorm_thold = 6.0
    ) const noexcept {
        if (m_traj_state == traj_state_t::STOP) return true;

        if (angles_enorm_thold) {
            auto err = angles_enorm();
            if (err && *err > *angles_enorm_thold) return false;
        }
        if (pos_enorm_thold) {
            auto err = pos_enorm();
            if (err && *err > *pos_enorm_thold) return false;
        }
        if (rot_enorm_thold) {
            auto err = rot_enorm();
            if (err && *err > *rot_enorm_thold) return false;
        }
        // 속도 및 가속도 기반 체크 추가 가능 (m_angvels.norm() 등)
        return true;
    }

  protected:
    // 각 엔진 업데이트 시 상태 복사 로직 (DSR 스타일)
    void update_stopping(const value_t &dt) noexcept {
        m_gen_stop.update(dt);
        copy_state(m_gen_stop);
        if (m_gen_stop.goal_reached()) m_traj_state = traj_state_t::STOP;
    }

    void update_trapj(const value_t &dt) noexcept {
        m_gen_trapj.update(dt);
        copy_state(m_gen_trapj);
        if (m_gen_trapj.goal_reached()) m_traj_state = traj_state_t::STOP;
    }

    void update_attrj(const value_t &dt) noexcept {
        m_gen_attrj.update(dt);
        copy_state(m_gen_attrj);
    }

    void update_attrl(const value_t &dt) noexcept {
        if (!m_gen_attrl.update(dt)) stop();
        else copy_state(m_gen_attrl);
    }

    void update_playj(const value_t &dt) noexcept {
        m_gen_playj.update(dt); // bool 체크 제거
        copy_state(m_gen_playj);
        // if (m_gen_playj.is_finished()) stop(); // 필요시 완료 체크 추가
    }

    template<typename T>
    void copy_state(const T& gen) {
        m_angles = gen.angles();
        m_angvels = gen.angvels();
        m_angaccs = gen.angaccs();
    }

    void update_clip() noexcept {
        m_angles = m_angles.cwiseMax(m_model.get_min_angles()).cwiseMin(m_model.get_max_angles());
    }

    void update_subordinates() noexcept {
        auto [pose, J] = ik::compute_forward_and_jacobian(&m_model, m_angles, m_tcp_offset);
        m_tmat = pose;
        m_jmat = J;
        double d2r = M_PI / 180.0;
        m_a = m_jmat * (m_angvels * d2r);
    }

  protected:
    rt_control::model::RobotModel m_model;
    traj_state_t m_traj_state;
    
    TrajStop m_gen_stop; 
    TrajTrapJ m_gen_trapj; 
    TrajAttrJ m_gen_attrj;
    TrajAttrL m_gen_attrl; 
    TrajPlayJ m_gen_playj;

    angles_t m_angles, m_angvels, m_angaccs;
    tmat_t m_tmat;
    tmat_t m_tcp_offset; 
    jmat_t m_jmat;
    a_t m_a;
    a_t m_a_d1;
};

} // namespace trajectory
} // namespace rt_control