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
    using traj_state_t = TrajState;
    using value_t = rt_control::value_t;
    using angles_t = rt_control::angles_t;
    using tmat_t = Eigen::Isometry3d;
    using jmat_t = Eigen::Matrix<double, 6, 6>;
    using a_t = Eigen::Matrix<value_t, 6, 1>;

  public:
    TrajGenerator() {
        m_tcp_offset = tmat_t::Identity();
        m_traj_state = traj_state_t::STOP;
        m_angles.setZero();
        m_angvels.setZero();
        m_angaccs.setZero();
        m_a.setZero();
    }

    void initialize(const rt_control::model::RobotModel& robot_model,
                    const angles_t &start_angles, 
                    const angles_t &start_angvels, 
                    const angles_t &start_angaccs) noexcept {
        m_model = robot_model;
        m_angles = start_angles;
        m_angvels = start_angvels;
        m_angaccs = start_angaccs;

        update_subordinates();
        m_traj_state = traj_state_t::STOP;
    }

    void update(const value_t &dt) noexcept {
        switch (m_traj_state) {
            case traj_state_t::STOP:
                m_angvels.setZero();
                m_angaccs.setZero();
                break;
            case traj_state_t::STOPPING: update_stopping(dt); break;
            case traj_state_t::TRAPJ:    update_trapj(dt);    break;
            case traj_state_t::ATTRJ:    update_attrj(dt);    break; // 🌟 함수 확인
            case traj_state_t::ATTRL:    update_attrl(dt);    break;
            case traj_state_t::PLAYJ:    update_playj(dt);    break; // 🌟 함수 확인
        }
        update_clip();
        update_subordinates();
    }

    // --- TCP Management ---
    void set_tcp_tmat(const tmat_t &new_shift_tmat) noexcept { m_tcp_offset = new_shift_tmat; }
    void set_tcp(value_t x, value_t y, value_t z, value_t r_deg, value_t p_deg, value_t yaw_deg) noexcept {
        double d2r = M_PI / 180.0;
        tmat_t T = tmat_t::Identity();
        T.translation() << x, y, z;
        T.linear() = (Eigen::AngleAxisd(yaw_deg * d2r, Eigen::Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(p_deg * d2r, Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(r_deg * d2r, Eigen::Vector3d::UnitX())).toRotationMatrix();
        set_tcp_tmat(T);
    }

    // --- Trajectory Commands ---
    void stop() noexcept {
        m_gen_stop = TrajStop(m_angles, m_angvels, m_angaccs);
        m_traj_state = traj_state_t::STOPPING;
    }
    // 1. 관절 각도 오차 (L2 Norm, deg)
    [[nodiscard]] std::optional<value_t> angles_enorm() const noexcept {
        auto gq = goal_angles();
        if (!gq.has_value()) return std::nullopt;
        return (gq.value() - m_angles).norm();
    }

    // 2. 위치 오차 (L2 Norm, m)
    [[nodiscard]] std::optional<value_t> pos_enorm() const noexcept {
        auto gt = goal_tmat();
        if (!gt.has_value()) return std::nullopt;
        return (gt.value().translation() - m_tmat.translation()).norm();
    }

    // 3. 회전 오차 (Angle 차이, deg)
    [[nodiscard]] std::optional<value_t> rot_enorm() const noexcept {
        auto gt = goal_tmat();
        if (!gt.has_value()) return std::nullopt;
        // R_err = R_target * R_curr^T
        Eigen::AngleAxisd err_aa(gt.value().linear() * m_tmat.linear().transpose());
        return std::abs(err_aa.angle()) * (180.0 / M_PI);
    }

    // 4. 관절 속도 오차 (deg/s)
    [[nodiscard]] std::optional<value_t> angvels_enorm() const noexcept {
        return m_angvels.norm(); // 정지 목표이므로 현재 속도 자체가 오차
    }

    // 5. 작업 공간 선속도 오차 (m/s)
    [[nodiscard]] std::optional<value_t> vel_enorm() const noexcept {
        return m_a.segment<3>(0).norm();
    }

    // 6. 작업 공간 각속도 오차 (deg/s)
    [[nodiscard]] std::optional<value_t> w_enorm() const noexcept {
        return m_a.segment<3>(3).norm();
    }

    [[nodiscard]] std::optional<angles_t> goal_angles() const noexcept {
        if (m_traj_state == traj_state_t::STOP) return m_angles;
        if (m_traj_state == traj_state_t::STOPPING) return m_gen_stop.goal_angles();
        if (m_traj_state == traj_state_t::TRAPJ) return m_gen_trapj.goal_angles();
        // ATTRL 상태에서는 실시간으로 목표 각도가 변하므로 특정 목표 각도가 고정되어 있지 않음
        return std::nullopt;
    }

    /**
     * @brief 현재 진행 중인 궤적의 최종 목표 포즈(Tmat)를 반환
     */
    [[nodiscard]] std::optional<tmat_t> goal_tmat() const noexcept {
        if (m_traj_state == traj_state_t::ATTRL) {
            return m_gen_attrl.goal_pose();
        }
        // Joint 기반 제어 중일 때는 목표 각도를 Forward Kinematics로 변환해서 반환
        auto gq = goal_angles();
        if (gq.has_value()) {
            return solve_forward(gq.value());
        }
        return std::nullopt;
    }

    [[nodiscard]] bool trapj(const angles_t &goal_angles, const angles_t &goal_angvels = angles_t::Zero()) noexcept {
        m_gen_trapj = TrajTrapJ(&m_model, m_angles, m_angvels, goal_angles, goal_angvels,
                                m_model.get_max_angvels(), m_model.get_max_angaccs());
        m_traj_state = traj_state_t::TRAPJ;
        return true;
    }

    [[nodiscard]] bool attrl(const tmat_t &goal_tmat, const value_t &kp = 50.0) noexcept {
        m_gen_attrl = TrajAttrL(&m_model, m_angles, m_angvels, m_angaccs, m_tcp_offset);
        m_gen_attrl.set_goal_pose(goal_tmat);
        m_gen_attrl.set_kp(kp);
        m_traj_state = traj_state_t::ATTRL;
        return true;
    }

    [[nodiscard]] bool align_tcp_to_floor(const value_t &kp = 100.0) noexcept {
        tmat_t target = m_tmat; // 현재 위치(translation)는 그대로 유지
        target.linear() << 1,  0,  0,
                           0, -1,  0,
                           0,  0, -1; // 월드 좌표계 기준 바닥 방향
        return attrl(target, kp);     // 내부적으로 attrl을 재활용하여 구동
    }

    // 🌟 추가 2 (보너스): 정면(X) 방향으로 TCP 정렬 (벽면 작업용)
    [[nodiscard]] bool align_tcp_to_front(const value_t &kp = 100.0) noexcept {
        tmat_t target = m_tmat;
        target.linear() << 0,  0,  1,
                           0,  1,  0,
                          -1,  0,  0; // 월드 좌표계 기준 정면(X축) 방향
        return attrl(target, kp);
    }

    // --- Kinematics Solvers ---
    [[nodiscard]] tmat_t solve_forward(const angles_t &q) const noexcept {
        auto [pose, J] = ik::compute_forward_and_jacobian(&m_model, q, m_tcp_offset);
        return pose;
    }

    // --- Getters ---
    [[nodiscard]] const angles_t& angvels() const noexcept { return m_angvels; }
    [[nodiscard]] const angles_t& angaccs() const noexcept { return m_angaccs; }
    [[nodiscard]] const jmat_t& jmat() const noexcept { return m_jmat; }
    [[nodiscard]] const a_t& a() const noexcept { return m_a; }
    [[nodiscard]] const angles_t& angles() const noexcept { return m_angles; }
    [[nodiscard]] const tmat_t& tmat() const noexcept { return m_tmat; }
    [[nodiscard]] constexpr bool goal_reached(
        const std::optional<value_t> &angles_enorm_thold = 2.0,
        const std::optional<value_t> &pos_enorm_thold = 0.002,
        const std::optional<value_t> &rot_enorm_thold = 3.0,
        const std::optional<value_t> &angvels_enorm_thold = 4.0,
        const std::optional<value_t> &vel_enorm_thold = 0.004,
        const std::optional<value_t> &w_enorm_thold = 6.0) const noexcept 
    {
        // 이미 정지 상태인 경우 즉시 true 반환
        if (m_traj_state == traj_state_t::STOP) return true;

        auto reached = false;

        // 1. Joint Angle Error Check
        if (angles_enorm_thold.has_value()) {
            const auto trg = this->angles_enorm(); // 멤버 함수 호출
            if (trg.has_value() && trg.value() > angles_enorm_thold.value()) {
                return false;
            }
            reached = true;
        }

        // 2. Position Error Check (m)
        if (pos_enorm_thold.has_value()) {
            const auto trg = this->pos_enorm();
            if (trg.has_value() && trg.value() > pos_enorm_thold.value()) {
                return false;
            }
            reached = true;
        }

        // 3. Rotation Error Check (deg)
        if (rot_enorm_thold.has_value()) {
            const auto trg = this->rot_enorm();
            if (trg.has_value() && trg.value() > rot_enorm_thold.value()) {
                return false;
            }
            reached = true;
        }

        // 4. Joint Velocity Error Check
        if (angvels_enorm_thold.has_value()) {
            const auto trg = this->angvels_enorm();
            if (trg.has_value() && trg.value() > angvels_enorm_thold.value()) {
                return false;
            }
            reached = true;
        }

        // 5. Cartesian Velocity Error Check
        if (vel_enorm_thold.has_value()) {
            const auto trg = this->vel_enorm();
            if (trg.has_value() && trg.value() > vel_enorm_thold.value()) {
                return false;
            }
            reached = true;
        }

        // 6. Angular Velocity Error Check
        if (w_enorm_thold.has_value()) {
            const auto trg = this->w_enorm();
            if (trg.has_value() && trg.value() > w_enorm_thold.value()) {
                return false;
            }
            reached = true;
        }

        return reached;
    }
    

  protected:
    // 🌟 에러 해결: 모든 하위 업데이트 함수 정의
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
        m_gen_playj.update(dt);
        copy_state(m_gen_playj);
    }

    template<typename T> void copy_state(const T& gen) {
        m_angles = gen.angles(); m_angvels = gen.angvels(); m_angaccs = gen.angaccs();
    }
    void update_clip() noexcept {
        m_angles = m_angles.cwiseMax(m_model.get_min_angles()).cwiseMin(m_model.get_max_angles());
    }
    void update_subordinates() noexcept {
        auto [pose, J] = ik::compute_forward_and_jacobian(&m_model, m_angles, m_tcp_offset);
        m_tmat = pose; m_jmat = J;
        m_a = m_jmat * (m_angvels * (M_PI / 180.0));
    }

  protected:
    rt_control::model::RobotModel m_model;
    traj_state_t m_traj_state;
    TrajStop m_gen_stop; TrajTrapJ m_gen_trapj; TrajAttrJ m_gen_attrj;
    TrajAttrL m_gen_attrl; TrajPlayJ m_gen_playj;
    angles_t m_angles, m_angvels, m_angaccs;
    tmat_t m_tmat, m_tcp_offset;
    jmat_t m_jmat; a_t m_a;
};

} // namespace trajectory
} // namespace rt_control