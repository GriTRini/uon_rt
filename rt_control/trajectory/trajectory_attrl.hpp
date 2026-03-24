#pragma once

#include "trajectory_attrj.hpp"
#include "../model/model.hpp"
#include "trajectory_IK/ik_solver.hpp"

namespace rt_control {
namespace trajectory {

/**
 * @brief TrajAttrL 클래스
 * Cartesian Space(작업 공간)에서 목표 Pose를 추종하는 궤적 생성기입니다.
 * 내부적으로 IK Solver를 호출하여 조인트 목표값을 갱신합니다.
 */
class TrajAttrL : public TrajAttrJ {
  public:
    using Base = TrajAttrJ;
    using angles_t = Base::angles_t;
    using value_t = Base::value_t;
    using tmat_t = Eigen::Isometry3d;
    using a_t = Eigen::Matrix<value_t, 6, 1>;

  public:
    TrajAttrL() = default;

    /**
     * @brief TrajGenerator에서 호출하는 DSR 스타일 생성자
     */
    TrajAttrL(const model::RobotModel* model, 
              const angles_t &q, const angles_t &dq, const angles_t &ddq,
              const tmat_t& tcp_offset) noexcept
        : Base(q, dq, ddq, 
               model->get_min_angles(), model->get_max_angles(),
               model->get_min_angvels(), model->get_max_angvels(),
               model->get_min_angaccs(), model->get_max_angaccs()),
          m_model(model), m_tcp_offset(tcp_offset) 
    {
        // 초기 TCP 포즈 계산 및 저장
        auto [initial_pose, J] = ik::compute_forward_and_jacobian(m_model, q, m_tcp_offset);
        m_tmat = initial_pose;
        m_goal_tmat = initial_pose;
        m_goal_a.setZero();
    }

    /**
     * @brief 매 제어 주기마다 호출되는 업데이트 루프
     */
    [[nodiscard]] bool update(const value_t &dt) noexcept override {
        if (dt <= 0.0) return false;

        // 🌟 제공해주신 IKSolver::solve 인터페이스 호출
        auto [target_q, converged] = ik::IKSolver::solve(
            m_model, 
            m_goal_tmat, 
            Base::angles(), 
            m_tcp_offset, 
            100,    // step_max
            1e-4,   // enorm_threshold
            0.01    // damping
        );
        
        if (!converged) {
            // 수렴 실패 시 멈추거나 이전 상태 유지 (디버깅을 위해 로그 출력 추천)
            return false; 
        }

        Base::set_goal_angles(target_q);
        if (!Base::update(dt)) return false;

        // 현재 상태 갱신
        auto [cp, J] = ik::compute_forward_and_jacobian(m_model, Base::angles(), m_tcp_offset);
        m_tmat = cp;

        return true;
    }

    // --- DSR 호환 Setters & Getters ---
    
    /** @brief 목표 포즈 설정 (TrajGenerator::attrl에서 호출) */
    void set_tmat_goal(const tmat_t& goal) noexcept { m_goal_tmat = goal; }
    void set_goal_pose(const tmat_t& goal) noexcept { m_goal_tmat = goal; } // Alias

    /** @brief 목표 작업공간 속도 설정 */
    void set_a_goal(const a_t& goal_a) noexcept { m_goal_a = goal_a; }

    /** @brief 카테시안 게인 설정 */
    void set_kp_cartesian(value_t kp) noexcept { 
        // 조인트 엔진의 게인을 조정하여 추종 강도를 제어
        Base::set_kp(kp); 
    }
    void set_kp(value_t kp) noexcept { Base::set_kp(kp); } // Alias

    [[nodiscard]] const tmat_t& goal_tmat() const noexcept { return m_goal_tmat; }
    [[nodiscard]] const tmat_t& goal_pose() const noexcept { return m_goal_tmat; }
    [[nodiscard]] const tmat_t& tmat() const noexcept { return m_tmat; }
    [[nodiscard]] const a_t& goal_a() const noexcept { return m_goal_a; }

  protected:
    const model::RobotModel* m_model = nullptr;
    tmat_t m_tmat;         // 현재 TCP Pose
    tmat_t m_goal_tmat;    // 목표 TCP Pose
    tmat_t m_tcp_offset;   // Tool Offset
    a_t m_goal_a;          // 목표 작업공간 가속도/속도(DSR 확장용)
};

} // namespace trajectory
} // namespace rt_control