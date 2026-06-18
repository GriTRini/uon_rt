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

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
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
        
        std::cout << "[Debug] 1. Initialize Start" << std::endl;
        
        // 모델 복사
        m_model = robot_model;
        std::cout << "[Debug] 2. Model Copied. Name: " << m_model.get_model_name() << std::endl;

        // 초기값 설정
        m_angles = start_angles;
        m_angvels = start_angvels;
        m_angaccs = start_angaccs;
        std::cout << "[Debug] 3. Angles set: " << m_angles.transpose() << std::endl;

        // 🌟 여기서 Segfault가 나는지 확인해야 합니다!
        std::cout << "[Debug] 4. Calling update_subordinates()..." << std::endl;
        update_subordinates(); 
        
        std::cout << "[Debug] 5. Initialize Done" << std::endl;
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
        // ATTRL, PLAYJ 상태에서는 실시간으로 목표 각도가 변하므로 특정 목표 각도가 고정되어 있지 않음
        return std::nullopt;
    }

    /**
     * @brief 현재 진행 중인 궤적의 최종 목표 포즈(Tmat)를 반환
     */
    [[nodiscard]] std::optional<tmat_t> goal_tmat() const noexcept {
        if (m_traj_state == traj_state_t::ATTRL) {
            return m_gen_attrl.goal_pose();
        }
        if (m_traj_state == traj_state_t::PLAYJ) {
            return std::nullopt; // PLAYJ 진행 중에는 위치 기반 오차 검사 무시
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

    // ---------------------------------------------------------------------
    // 행렬(tmat_t)을 직접 받는 attrl
    // ---------------------------------------------------------------------
    [[nodiscard]] bool attrl(const tmat_t &goal_tmat, const value_t &attrl_kp = 50.0, const value_t &attrj_kp = 150.0, double target_speed = 0.20) noexcept {
            
        // 1. 객체를 생성할 때 새로 만든 생성자 규격에 맞춰 
        //    attrl_kp, attrj_kp, target_speed를 각각 따로 넘겨주어 초기화합니다.
        m_gen_attrl = TrajAttrL(&m_model, m_angles, m_angvels, m_angaccs, m_tcp_offset, 
                                attrl_kp, attrj_kp, target_speed);
        
        // 2. 목표 도달 가능 여부 검증 및 설정
        //    앞서 수정한 set_goal_pose에도 동일하게 분리된 값을 전달합니다.
        bool is_valid_target = m_gen_attrl.set_goal_pose(goal_tmat, attrl_kp, attrj_kp, target_speed);
        
        if (!is_valid_target) {
            std::cerr << "[Error] attrl: 목표 지점(Pose) 도달이 불가능하여 명령이 거부되었습니다." << std::endl;
            return false;
        }

        // 3. 상태 변경 및 실행
        m_traj_state = traj_state_t::ATTRL;
        return true;
    }

    // ---------------------------------------------------------------------
    // 🌟 웨이포인트 리스트를 받아 실행하는 playj 활성화
    // ---------------------------------------------------------------------
    [[nodiscard]] bool playj(const std::vector<WaypointJ> &waypoints,
                             const angles_t &user_peak_angvels,
                             const angles_t &user_peak_angaccs,
                             value_t p_gain = 5.0) noexcept {
        if (waypoints.empty()) {
            std::cerr << "[Error] playj: 웨이포인트 리스트가 비어 있습니다." << std::endl;
            return false;
        }

        m_gen_playj = TrajPlayJ(&m_model, m_angles, m_angvels, m_angaccs, waypoints,
                                user_peak_angvels, user_peak_angaccs, p_gain);
        m_traj_state = traj_state_t::PLAYJ;
        return true;
    }
    
    // 🌟 1. 바닥(-Z) 방향으로 TCP 정렬 (손목 위치 고정 방식)
    [[nodiscard]] bool align_tcp_to_floor(double yaw_deg = 0.0, const value_t &kp = 100.0) noexcept {
        // 1. 현재 손목(Flange) 포즈 역산
        tmat_t T_flange_current = m_tmat * m_tcp_offset.inverse();

        // 2. 기본 바닥 방향 (Z는 바닥(-Z), X는 월드 정면(+X))
        Eigen::Matrix3d R_base;
        R_base << 1,  0,  0,
                  0, -1,  0,
                  0,  0, -1; 

        // 3. 사용자가 입력한 각도(yaw_deg)만큼 Z축을 기준으로 회전시켜 X, Y 방향 조절!
        Eigen::Matrix3d R_down = Eigen::AngleAxisd(yaw_deg * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix() * R_base;

        // 4. 새로운 손목 포즈 (위치는 그대로 고정, 자세는 역산)
        tmat_t T_flange_new = Eigen::Isometry3d::Identity();
        T_flange_new.translation() = T_flange_current.translation();
        T_flange_new.linear() = R_down * m_tcp_offset.linear().inverse();

        // 5. 로봇이 추종할 최종 TCP 목표 포즈 생성
        tmat_t target_tcp = T_flange_new * m_tcp_offset;

        return attrl(target_tcp, kp);
    }

    // 🌟 2. 정면(X) 방향으로 TCP 정렬 (벽면 작업용, 손목 위치 고정 방식)
    [[nodiscard]] bool align_tcp_to_front(const value_t &kp = 100.0) noexcept {
        // 1. 현재 손목(Flange) 포즈 역산
        tmat_t T_flange_current = m_tmat * m_tcp_offset.inverse();

        // 2. 정면을 향하는 목표 회전 행렬
        Eigen::Matrix3d R_front;
        R_front << 0,  0,  1,
                   0,  1,  0,
                  -1,  0,  0; 

        // 3. 새로운 손목 포즈
        tmat_t T_flange_new = Eigen::Isometry3d::Identity();
        T_flange_new.translation() = T_flange_current.translation();
        T_flange_new.linear() = R_front * m_tcp_offset.linear().inverse();

        // 4. 로봇이 추종할 최종 TCP 목표 포즈 생성
        tmat_t target_tcp = T_flange_new * m_tcp_offset;

        return attrl(target_tcp, kp);
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
    [[nodiscard]] tmat_t flange_tmat() const noexcept { return m_tmat * m_tcp_offset.inverse(); }
    
    [[nodiscard]] constexpr bool goal_reached(
        const std::optional<value_t> &angles_enorm_thold = 2.0,
        const std::optional<value_t> &pos_enorm_thold = 0.002,
        const std::optional<value_t> &rot_enorm_thold = 3.0,
        const std::optional<value_t> &angvels_enorm_thold = 4.0,
        const std::optional<value_t> &vel_enorm_thold = 0.004,
        const std::optional<value_t> &w_enorm_thold = 6.0) const noexcept 
    {
        // 1. 이미 궤적이 끝나서 STOP 상태가 된 경우 정상적으로 완료(true) 반환
        if (m_traj_state == traj_state_t::STOP) return true;

        // 🌟 2. 버그 수정: PLAYJ 모드 진행 중에는 무조건 false 반환
        // (마지막 웨이포인트에 도달하면 update() 함수 내부에서 자동으로 STOP 상태로 전환됨)
        if (m_traj_state == traj_state_t::PLAYJ) return false;

        auto reached = true; // 기본적으로 true로 두고, 오차 범위를 하나라도 벗어나면 false로 변경

        // 3. Joint Angle Error Check
        if (angles_enorm_thold.has_value()) {
            const auto trg = this->angles_enorm();
            if (trg.has_value() && trg.value() > angles_enorm_thold.value()) return false;
        }

        // 4. Position Error Check (m)
        if (pos_enorm_thold.has_value()) {
            const auto trg = this->pos_enorm();
            if (trg.has_value() && trg.value() > pos_enorm_thold.value()) return false;
        }

        // 5. Rotation Error Check (deg)
        if (rot_enorm_thold.has_value()) {
            const auto trg = this->rot_enorm();
            if (trg.has_value() && trg.value() > rot_enorm_thold.value()) return false;
        }

        // 6. Joint Velocity Error Check
        if (angvels_enorm_thold.has_value()) {
            const auto trg = this->angvels_enorm();
            if (trg.has_value() && trg.value() > angvels_enorm_thold.value()) return false;
        }

        // 7. Cartesian Velocity Error Check
        if (vel_enorm_thold.has_value()) {
            const auto trg = this->vel_enorm();
            if (trg.has_value() && trg.value() > vel_enorm_thold.value()) return false;
        }

        // 8. Angular Velocity Error Check
        if (w_enorm_thold.has_value()) {
            const auto trg = this->w_enorm();
            if (trg.has_value() && trg.value() > w_enorm_thold.value()) return false;
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
    
    // 🌟 PLAYJ 상태 전이 추가 (모든 웨이포인트를 다 돌면 STOP으로 전이)
    void update_playj(const value_t &dt) noexcept {
        m_gen_playj.update(dt);
        copy_state(m_gen_playj);
        if (m_gen_playj.goal_reached()) {
            m_traj_state = traj_state_t::STOP;
        }
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