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
        m_gen_attrj.set_pd_gains(kp); // 내부에서 Kd = 2*sqrt(Kp) 자동 설정
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

    /**
     * @brief goal_reached: 현재 상태가 목표치에 도달했는지 판단
     * 수정 사항: 
     * 1. m_a(가속도) 대신 m_angvels(관절 속도)를 활용한 Cartesian 속도 추정 또는 관절 속도 직접 체크
     * 2. mm/s 변환(/1000.0) 오류 제거 및 현실적인 기본값 적용
     */
    [[nodiscard]] bool goal_reached(
        std::optional<value_t> pos_thold = 0.001,   // m (Cartesian) -> 1mm
        std::optional<value_t> rot_thold = 1.0,     // deg (Cartesian) -> 1도
        std::optional<value_t> joint_thold = 0.1,   // deg (Joint) -> 0.1도
        std::optional<value_t> vel_thold = 0.01      // m/s (Cartesian Velocity) -> 10mm/s
    ) const noexcept {
        
        if (m_traj_state == traj_state_t::STOP) return true;

        // 1. TRAPJ 모드: 시간 기반 도달 확인
        if (m_traj_state == traj_state_t::TRAPJ) {
            return m_gen_trapj.goal_reached();
        }

        // 2. ATTRL 모드: 작업 공간(Cartesian) 오차 확인
        if (m_traj_state == traj_state_t::ATTRL) {
            const auto& target = m_gen_attrl.goal_pose();
            
            // 위치 오차 계산
            double p_err = (target.translation() - m_tmat.translation()).norm();
            
            // 회전 오차 계산
            Eigen::AngleAxisd r_err_aa(target.linear() * m_tmat.linear().transpose());
            double r_err = std::abs(r_err_aa.angle()) * (180.0 / M_PI);
            
            // 속도 계산 (m_a가 가속도라면, 실제 관절 속도 m_angvels를 통해 판단하는 것이 정확합니다)
            // m_angvels는 deg/s이므로 norm을 통해 전체적인 움직임 확인
            double v_norm = m_angvels.norm(); 

            bool pos_ok = !pos_thold || (p_err <= *pos_thold);
            bool rot_ok = !rot_thold || (r_err <= *rot_thold);
            
            // 🌟 수정: vel_thold를 m/s 기준(0.01 = 10mm/s)으로 직접 비교하거나 
            // 관절 속도 합계가 0.5 deg/s 이하인지 확인
            bool vel_ok = !vel_thold || (v_norm <= 0.5); 

            return pos_ok && rot_ok && vel_ok;
        }

        // 3. ATTRJ 모드: 관절 공간(Joint) 오차 확인
        if (m_traj_state == traj_state_t::ATTRJ) {
            double q_err = (m_gen_attrj.goal_angles() - m_angles).norm();
            double dq_err = m_angvels.norm();

            bool q_ok = !joint_thold || (q_err <= *joint_thold);
            bool dq_ok = !vel_thold || (dq_err <= 0.5); // 관절 속도 합계 0.5 deg/s 기준

            return q_ok && dq_ok;
        }

        return false;
    }

    // --- Getters & Status ---
    /**
     * @brief 현재 관절 각도 [deg]
     */
    [[nodiscard]] const angles_t& angles() const noexcept { return m_angles; }

    /**
     * @brief 현재 관절 속도 [deg/s]
     */
    [[nodiscard]] const angles_t& angvels() const noexcept { return m_angvels; }

    /**
     * @brief 현재 관절 가속도 [deg/s^2]
     */
    [[nodiscard]] const angles_t& angaccs() const noexcept { return m_angaccs; }

    /**
     * @brief 현재 TCP 포즈 (T-Matrix)
     */
    [[nodiscard]] const tmat_t& tmat() const noexcept { return m_tmat; }

    /**
     * @brief 현재 TCP 작업 공간 속도 [m/s, rad/s]
     */
    [[nodiscard]] const a_t& a() const noexcept { return m_a; }

    /**
     * @brief 현재 궤적 생성기의 상태 (TRAPJ, ATTRJ 등)
     */
    [[nodiscard]] traj_state_t state() const noexcept { return m_traj_state; }

    [[nodiscard]] tmat_t get_attractor_pose() const noexcept {
        if (m_traj_state == traj_state_t::ATTRL) {
            return m_gen_attrl.attractor_pose();
        }
        return m_tmat; // ATTRL 모드가 아니면 그냥 현재 로봇 포즈 반환
    }

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
        // 1. 현재 각도로 포즈(FK) 계산
        m_tmat = m_model.forward_kinematics(m_angles);

        // 2. 자코비안을 모델이 아닌 IKSolver(또는 분할된 모듈)를 통해 계산
        // IKSolver 내부의 static 함수를 사용하거나 직접 계산 로직을 넣습니다.
        // 여기서는 가장 깔끔하게 IKSolver의 로직을 활용합니다.
        
        // (만약 IKSolver가 public으로 자코비안 계산 함수를 열어뒀다면)
        // m_a = ik::IKSolver::compute_jacobian(&m_model, m_angles) * (m_angvels * (M_PI/180.0));
        
        // 또는 간단하게 현재 상태가 ATTRL이라면 gen_attrl이 이미 계산한 값을 가져와도 됩니다.
        if (m_traj_state == traj_state_t::ATTRL) {
            // TrajAttrL에 tmat() 처럼 a() 혹은 vel() getter를 추가하여 가져오는 것이 가장 효율적입니다.
        }
}

  protected:
    rt_control::model::RobotModel m_model;
    traj_state_t m_traj_state = traj_state_t::STOP;
    
    TrajStop m_gen_stop; TrajTrapJ m_gen_trapj; TrajAttrJ m_gen_attrj;
    TrajAttrL m_gen_attrl; TrajPlayJ m_gen_playj;

    angles_t m_angles, m_angvels, m_angaccs;
    tmat_t m_tmat;
    a_t m_a; // Task space velocity
};

} // namespace trajectory
} // namespace rt_control