#pragma once
#include <iostream>
#include <algorithm>
#include "forward_kinematics.hpp"

namespace rt_control::trajectory::ik {

inline std::pair<rt_control::angles_t, bool> 
solve_step_dls(const model::RobotModel* model,
               const Eigen::Isometry3d& target_pose,
               const rt_control::angles_t& seed_q_deg,
               const Eigen::Isometry3d& tcp_offset, // 🌟 TCP 추가
               size_t max_iter, double tol, double damping) {
    
    rt_control::angles_t q_deg = seed_q_deg;
    bool converged = false;
    const double MAX_STEP_RAD = 5.0 * (M_PI / 180.0);

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

        Eigen::Matrix<double, 6, 6> JT = J.transpose();
        Eigen::Matrix<double, 6, 6> JJT_damped = J * JT + lambda_sq * Eigen::Matrix<double, 6, 6>::Identity();
        Eigen::Matrix<double, 6, 1> dq_rad = JT * JJT_damped.ldlt().solve(error);

        double dq_norm = dq_rad.norm();
        if (dq_norm > MAX_STEP_RAD) dq_rad *= (MAX_STEP_RAD / dq_norm);

        q_deg += dq_rad * (180.0 / M_PI);
        q_deg = q_deg.cwiseMax(model->get_min_angles()).cwiseMin(model->get_max_angles());
    }

    return {q_deg, converged};
}

}