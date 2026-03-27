#pragma once
#include <iostream>
#include <algorithm>
#include <Eigen/Dense>
#include "forward_kinematics.hpp"

namespace rt_control::trajectory::ik {

// 1. QP 문제를 정의할 구조체 추가
struct QPProblem {
    Eigen::Matrix<double, 6, 6> H;
    Eigen::Matrix<double, 6, 1> g;
    Eigen::Matrix<double, 6, 1> lb;
    Eigen::Matrix<double, 6, 1> ub;
};

// 2. Projected Gauss-Seidel QP 솔버 추가
inline Eigen::Matrix<double, 6, 1> solve_qp_coordinate_descent(const QPProblem& qp, int max_iter = 50) {
    Eigen::Matrix<double, 6, 1> x = Eigen::Matrix<double, 6, 1>::Zero();

    for (int iter = 0; iter < max_iter; ++iter) {
        for (int i = 0; i < 6; ++i) {
            double sigma = qp.H.row(i).dot(x) - qp.H(i, i) * x(i);
            double x_unconstrained = (-qp.g(i) - sigma) / qp.H(i, i);
            x(i) = std::clamp(x_unconstrained, qp.lb(i), qp.ub(i)); // Box Constraint 적용
        }
    }
    return x;
}

// 3. 기존 solve_step_dls를 대체할 QP 기반 함수
inline std::pair<rt_control::angles_t, bool> 
solve_step_qp(const model::RobotModel* model,
              const Eigen::Isometry3d& target_pose,
              const rt_control::angles_t& seed_q_deg,
              const Eigen::Isometry3d& tcp_offset, // 🌟 TCP 추가
              size_t max_iter, double tol, double damping) {
    
    rt_control::angles_t q_deg = seed_q_deg;
    bool converged = false;
    const double MAX_STEP_RAD = 5.0 * (M_PI / 180.0); // 한 스텝당 최대 이동량 (속도 제한 역할)

    // 로봇의 하드웨어 조인트 리미트 (Radian 변환)
    Eigen::Matrix<double, 6, 1> min_q_rad = model->get_min_angles() * (M_PI / 180.0);
    Eigen::Matrix<double, 6, 1> max_q_rad = model->get_max_angles() * (M_PI / 180.0);

    for (size_t i = 0; i < max_iter; ++i) {
        // 🌟 TCP가 반영된 현재 포즈와 자코비안 획득
        auto [curr_tcp_pose, J] = compute_forward_and_jacobian(model, q_deg, tcp_offset);

        // 에러 계산 (Target Pose vs Current TCP Pose)
        Eigen::Vector3d p_err = target_pose.translation() - curr_tcp_pose.translation();

        Eigen::Quaterniond q_curr(curr_tcp_pose.linear());
        Eigen::Quaterniond q_target(target_pose.linear());
        Eigen::Quaterniond q_diff = q_target * q_curr.conjugate();
        Eigen::AngleAxisd aa_err(q_diff);
        
        double angle = aa_err.angle();
        if (angle > M_PI) angle -= 2.0 * M_PI;
        Eigen::Vector3d w_err = angle * aa_err.axis();

        Eigen::Matrix<double, 6, 1> error;
        error << p_err, w_err;

        if (error.norm() < tol) { converged = true; break; }

        double adaptive_lambda = (error.norm() > 0.05) ? std::max(damping, 0.1) : 0.01;
        double lambda_sq = adaptive_lambda * adaptive_lambda;

        // ==========================================
        // 🌟 이 부분이 기존 DLS에서 QP로 변경된 핵심 로직입니다.
        // ==========================================
        Eigen::Matrix<double, 6, 6> JT = J.transpose();
        
        QPProblem qp;
        // H = J^T * J + lambda^2 * I
        qp.H = JT * J + lambda_sq * Eigen::Matrix<double, 6, 6>::Identity();
        // g = -J^T * error
        qp.g = -JT * error;

        Eigen::Matrix<double, 6, 1> curr_q_rad = q_deg * (M_PI / 180.0);

        // Box Constraints 설정
        for (int j = 0; j < 6; ++j) {
            // 조인트 리미트 기반 최대/최소 이동 가능량
            double max_dq_pos = max_q_rad(j) - curr_q_rad(j);
            double min_dq_pos = min_q_rad(j) - curr_q_rad(j);

            // 하드웨어 한계와 MAX_STEP_RAD 중 더 안전한(작은) 값 선택
            qp.ub(j) = std::min(max_dq_pos, MAX_STEP_RAD);
            qp.lb(j) = std::max(min_dq_pos, -MAX_STEP_RAD);
        }

        // QP 연산으로 제약조건이 모두 반영된 dq_rad 도출
        Eigen::Matrix<double, 6, 1> dq_rad = solve_qp_coordinate_descent(qp, 50);
        // ==========================================

        // 각도 업데이트
        q_deg += dq_rad * (180.0 / M_PI);
        q_deg = q_deg.cwiseMax(model->get_min_angles()).cwiseMin(model->get_max_angles());
    }

    return {q_deg, converged};
}

} // namespace rt_control::trajectory::ik