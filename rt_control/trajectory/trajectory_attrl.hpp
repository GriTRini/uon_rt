#pragma once

#include "trajectory_attrj.hpp"
#include "../model/model.hpp"
#include "trajectory_IK/ik_solver.hpp"

namespace rt_control {
namespace trajectory {

/**
 * @brief TrajAttrL 클래스
 * Cartesian Space(작업 공간)에서 목표 Pose를 추종하며 실시간으로 조인트를 업데이트합니다.
 */
class TrajAttrL : public TrajAttrJ {
  public:
    using Base = TrajAttrJ;
    using angles_t = Base::angles_t;
    using value_t = Base::value_t;
    using tmat_t = Eigen::Isometry3d;
    using a_t = Eigen::Matrix<value_t, 6, 1>;

  public:
    TrajAttrL() : m_kp(50.0) {}

    TrajAttrL(const model::RobotModel* model, 
              const angles_t &q, const angles_t &dq, const angles_t &ddq,
              const tmat_t& tcp_offset) noexcept
        : Base(q, dq, ddq, 
               model->get_min_angles(), model->get_max_angles(),
               model->get_min_angvels(), model->get_max_angvels(),
               model->get_min_angaccs(), model->get_max_angaccs()),
          m_model(model), m_tcp_offset(tcp_offset), m_kp(50.0) 
    {
        auto [initial_pose, J] = ik::compute_forward_and_jacobian(m_model, q, m_tcp_offset);
        m_goal_tmat = initial_pose;
    }

    [[nodiscard]] bool update(const value_t &dt) noexcept override {
        if (dt <= 0.0) return false;

        // 1. 현재 TCP 포즈 및 Jacobian 획득
        auto [curr_pose, J_curr] = ik::compute_forward_and_jacobian(m_model, Base::angles(), m_tcp_offset);
        
        // 2. 위치 에러 (Position Error)
        Eigen::Vector3d p_err = m_goal_tmat.translation() - curr_pose.translation();
        
        // 3. 🌟 회전 에러 (Orientation Error) - 개선된 방식
        // R_err = R_goal * R_curr^T (현재 자세에서 목표 자세로 가기 위한 회전 행렬)
        Eigen::Matrix3d R_err = m_goal_tmat.linear() * curr_pose.linear().transpose();
        
        // 회전 행렬을 Angle-Axis로 변환
        Eigen::AngleAxisd aa_err(R_err);
        
        // 🌟 수치적 안정성 처리 (오차가 거의 없을 때 axis가 튀는 현상 방지)
        Eigen::Vector3d w_err;
        if (aa_err.angle() < 1e-6) {
            w_err.setZero();
        } else {
            // aa_err.angle()은 항상 양수이므로 axis가 방향을 결정함
            w_err = aa_err.axis() * aa_err.angle();
        }

        // 4. Task Velocity Command (v = Kp * error)
        Eigen::Matrix<double, 6, 1> x_dot;
        x_dot.segment<3>(0) = p_err * m_kp;
        x_dot.segment<3>(3) = w_err * m_kp;

        // 5. Jacobian Damped Least Squares (DLS)
        // 회전이 잘 안 먹는다면 lambda를 살짝 줄여보거나(0.005), 
        // Jacobian의 하부(Rotation 부분) 가중치를 확인해야 합니다.
        double lambda = 0.01; // Damping factor
        Eigen::Matrix<double, 6, 6> JJT = J_curr * J_curr.transpose();
        JJT += (lambda * lambda) * Eigen::Matrix<double, 6, 6>::Identity();
        
        // dq(rad) 계산
        Eigen::Matrix<double, 6, 1> dq_rad = J_curr.transpose() * JJT.ldlt().solve(x_dot);

        // 6. 다음 조인트 각도 계산 및 업데이트 (Deg 변환)
        angles_t next_q = Base::angles() + (dq_rad * (180.0 / M_PI) * dt);
        
        Base::set_goal_angles(next_q);
        return Base::update(dt);
    }

    // --- Setters ---
    void set_goal_pose(const tmat_t& goal) noexcept { m_goal_tmat = goal; }
    void set_kp(value_t kp) noexcept { m_kp = kp; }

    // --- Getters ---
    [[nodiscard]] const tmat_t& goal_pose() const noexcept { return m_goal_tmat; }

  protected:
    const model::RobotModel* m_model = nullptr;
    tmat_t m_goal_tmat;
    tmat_t m_tcp_offset;
    value_t m_kp; // Cartesian Gain
};

} // namespace trajectory
} // namespace rt_control