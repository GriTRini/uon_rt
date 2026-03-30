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

inline std::pair<rt_control::angles_t, bool> 
solve_step_qp(const model::RobotModel* model,
              const Eigen::Isometry3d& target_pose,
              const rt_control::angles_t& seed_q_deg,
              const Eigen::Isometry3d& tcp_offset,
              size_t max_iter, double tol, double damping) {
    
    rt_control::angles_t q_deg = seed_q_deg;
    bool converged = false;
    const double MAX_STEP_RAD = 3.0 * (M_PI / 180.0); // 한 스텝당 최대 이동량 (안정성을 위해 3도로 하향)

    Eigen::Matrix<double, 6, 1> min_q_rad = model->get_min_angles() * (M_PI / 180.0);
    Eigen::Matrix<double, 6, 1> max_q_rad = model->get_max_angles() * (M_PI / 180.0);
    Eigen::Matrix<double, 6, 1> center_q_rad = (min_q_rad + max_q_rad) / 2.0; // 조인트 중앙값

    for (size_t i = 0; i < max_iter; ++i) {
        auto [curr_tcp_pose, J] = compute_forward_and_jacobian(model, q_deg, tcp_offset);

        // 1. 에러 계산
        Eigen::Vector3d p_err = target_pose.translation() - curr_tcp_pose.translation();

        Eigen::Quaterniond q_curr(curr_tcp_pose.linear());
        Eigen::Quaterniond q_target(target_pose.linear());
        Eigen::Quaterniond q_diff = q_target * q_curr.conjugate();
        Eigen::AngleAxisd aa_err(q_diff);
        
        double angle = aa_err.angle();
        if (angle > M_PI) angle -= 2.0 * M_PI;
        Eigen::Vector3d w_err = angle * aa_err.axis();

        // =========================================================
        // ⭐ 개선 1: Cartesian Error Clamping (공간상 점진적 이동 유도)
        // =========================================================
        const double MAX_P_ERR = 0.05; // 한 번의 IK에서 최대 5cm까지만 해석
        const double MAX_W_ERR = 0.1;  // 한 번의 IK에서 최대 약 5.7도까지만 해석

        if (p_err.norm() > MAX_P_ERR) p_err = p_err.normalized() * MAX_P_ERR;
        if (w_err.norm() > MAX_W_ERR) w_err = w_err.normalized() * MAX_W_ERR;

        Eigen::Matrix<double, 6, 1> error;
        error << p_err, w_err;

        if (error.norm() < tol) { converged = true; break; }

        // =========================================================
        // ⭐ 개선 2: 연속적이고 부드러운 Damping 적용
        // =========================================================
        // 에러가 클수록 댐핑이 커지고, 작아지면 기본 damping 값으로 부드럽게 수렴
        double adaptive_lambda = damping + 0.1 * (error.norm() / (error.norm() + 0.1));
        double lambda_sq = adaptive_lambda * adaptive_lambda;

        Eigen::Matrix<double, 6, 6> JT = J.transpose();
        
        QPProblem qp;
        qp.H = JT * J + lambda_sq * Eigen::Matrix<double, 6, 6>::Identity();
        qp.g = -JT * error;

        // =========================================================
        // ⭐ 개선 3: Null-Space 조인트 중앙화 (Singularity 회피 보조)
        // =========================================================
        Eigen::Matrix<double, 6, 1> curr_q_rad = q_deg * (M_PI / 180.0);
        Eigen::Matrix<double, 6, 1> dist_from_center = curr_q_rad - center_q_rad;
        
        // 조인트가 한계에 가까워질수록 중앙으로 당기는 아주 작은 패널티 부여
        double k_center = 0.005; 
        qp.g += k_center * dist_from_center; 

        // Box Constraints 설정
        for (int j = 0; j < 6; ++j) {
            double max_dq_pos = max_q_rad(j) - curr_q_rad(j);
            double min_dq_pos = min_q_rad(j) - curr_q_rad(j);

            qp.ub(j) = std::min(max_dq_pos, MAX_STEP_RAD);
            qp.lb(j) = std::max(min_dq_pos, -MAX_STEP_RAD);
        }

        // QP 연산 도출
        Eigen::Matrix<double, 6, 1> dq_rad = solve_qp_coordinate_descent(qp, 50);

        // 각도 업데이트
        q_deg += dq_rad * (180.0 / M_PI);
        q_deg = q_deg.cwiseMax(model->get_min_angles()).cwiseMin(model->get_max_angles());
    }

    return {q_deg, converged};
}

} // namespace rt_control::trajectory::ik