#pragma once

#include "trajectory_attrj.hpp"
#include "trajectory_planning/cartesian_attractor.hpp"
#include "trajectory_IK/ik_solver.hpp"

namespace rt_control {
namespace trajectory {

class TrajAttrL : public TrajAttrJ {
private:
    using Base = TrajAttrJ;

public:
    using angles_t = Base::angles_t;
    using value_t = Base::value_t;
    using tmat_t = Eigen::Isometry3d;

    // IK 설정 (내부 상수)
    constexpr static value_t IK_TOL = 1e-4;
    constexpr static value_t IK_DAMPING = 0.01;
    constexpr static size_t IK_MAX_ITER = 20;

public:
    TrajAttrL() = default;

    /**
     * @brief Generator의 호출 형식에 맞춘 생성자
     * 인자로 받지 않는 리밋값들은 내부에서 모델 데이터를 참조하여 스스로 결정합니다.
     */
    TrajAttrL(const model::RobotModel* model,
            const angles_t &q, const angles_t &dq, const angles_t &ddq)
        : Base(model, q, dq, ddq, 
            model->get_max_angvels(), 
            model->get_max_angaccs(true)), // 🌟 ATTRL 전용 가속도 리밋 사용
        m_model(model)
    {
        // 1. 현재 로봇의 TCP 포즈 계산
        m_tmat = m_model->forward_kinematics(q);

        // 2. 모델 기반 리밋 설정 추출
        // 관절 속도/가속도 리밋 중 대표값(Max)을 추출하여 작업 공간(Cartesian) 리밋의 가이드로 사용
        // 보통 가장 빠른 관절의 리밋을 기준으로 유령의 최대 속도를 설정합니다.
        double max_joint_vel = m_model->get_max_angvels().maxCoeff(); // deg/s
        double max_joint_acc = m_model->get_max_angaccs(true).maxCoeff(); // deg/s^2

        // 3. 궤적 생성기(유령) 초기화
        m_attractor.init(
            m_tmat, 
            50.0,                                   // 기본 Kp_cartesian (유령의 복원력)
            0.5,                                    // peak_endvel (m/s) -> 모델 사양에 맞춰 조정 가능
            max_joint_vel * (M_PI / 180.0),         // peak_endangvel (rad/s) -> 모델 데이터 기반
            10.0,                                   // peak_endacc (m/s^2) -> 모델 사양에 맞춰 조정 가능
            max_joint_acc * (M_PI / 180.0)          // peak_endangacc (rad/s^2) -> 모델 데이터 기반
        );
        
        // 🌟 관절 공간 추종성 확보를 위한 Kp 설정
        // 유령의 Kp(50.0)보다 충분히 큰 값을 주어 유령을 놓치지 않게 합니다.
        this->set_kp(angles_t::Constant(200.0));
    }

    // --- Generator에서 호출하는 인터페이스 ---

    /** @brief 목표 포즈 설정 */
    void set_goal_pose(const tmat_t& goal) { 
        m_attractor.goal_pose = goal; 
    }

    /** @brief 작업 공간 게인(Kp) 설정 */
    void set_kp_cartesian(value_t kp) { 
        m_attractor.kp = kp; 
    }

    /** @brief 매 주기 업데이트 로직 */
    [[nodiscard]] bool update(const value_t &dt) noexcept override {
        if (dt <= 0.0) return false;

        // 1. 유령 전진
        m_attractor.update(dt);

        // 2. IK 번역 (직전 루프의 목표 각도를 Seed로 사용)
        auto [new_q_deg, converged] = ik::IKSolver::solve(
            m_model, 
            m_attractor.pose, 
            Base::goal_angles(), 
            IK_MAX_ITER, 
            IK_TOL, 
            IK_DAMPING
        );

        if (!converged) {
            // 실패 시 급격한 거동 방지를 위해 현재 상태 유지
            this->set_goal_angles(this->m_angles);
            return false;
        }

        // 3. 조인트 제어기 명령
        Base::set_goal_angles(new_q_deg);
        if (!Base::update(dt)) return false;

        // 실제 TCP 상태 갱신
        m_tmat = m_model->forward_kinematics(this->m_angles);
        return true;
    }

    [[nodiscard]] const tmat_t& goal_pose() const noexcept { return m_attractor.goal_pose; }
    [[nodiscard]] const tmat_t& tmat() const noexcept { return m_tmat; }
    [[nodiscard]] const tmat_t& attractor_pose() const noexcept { return m_attractor.pose; }

protected:
    const model::RobotModel* m_model = nullptr;
    planning::CartesianAttractor m_attractor;
    tmat_t m_tmat;
};

} // namespace trajectory
} // namespace rt_control