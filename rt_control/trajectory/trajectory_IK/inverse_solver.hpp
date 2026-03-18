#pragma once
#include <iostream>
#include <algorithm>
#include "forward_kinematics.hpp"

namespace rt_control::trajectory::ik {

inline std::pair<rt_control::angles_t, bool> 
solve_step_dls(const model::RobotModel* model,
               const Eigen::Isometry3d& target_pose,
               const rt_control::angles_t& seed_q_deg,
               size_t max_iter, double tol, double damping) {
    
    rt_control::angles_t q_deg = seed_q_deg;
    bool converged = false;
    const double MAX_STEP_RAD = 5.0 * (M_PI / 180.0); // 한 번에 최대 5도 제한

    for (size_t i = 0; i < max_iter; ++i) {
        auto [curr_pose, J] = compute_forward_and_jacobian(model, q_deg);

        // 1. 위치 에러
        Eigen::Vector3d p_err = target_pose.translation() - curr_pose.translation();

        // 2. 회전 에러 (쿼터니언 기반 최단 경로)
        Eigen::Quaterniond q_curr(curr_pose.linear());
        Eigen::Quaterniond q_target(target_pose.linear());
        Eigen::Quaterniond q_diff = q_target * q_curr.conjugate();
        Eigen::AngleAxisd aa_err(q_diff);
        
        double angle = aa_err.angle();
        if (angle > M_PI) angle -= 2.0 * M_PI;
        Eigen::Vector3d w_err = angle * aa_err.axis();

        Eigen::Matrix<double, 6, 1> error;
        error << p_err, w_err;

        double error_norm = error.norm();
        if (error_norm < tol) { converged = true; break; }

        // 3. 🌟 가변 댐핑 (Adaptive Damping)
        // 에러가 작을 때는 댐핑을 줄여 정밀도를 높이고, 클 때는 높여서 안정성 확보
        double adaptive_lambda = (error_norm > 0.05) ? std::max(damping, 0.1) : 0.01;
        double lambda_sq = adaptive_lambda * adaptive_lambda;

        // 4. DLS 연산 (LDLT 분해)
        Eigen::Matrix<double, 6, 6> JT = J.transpose();
        Eigen::Matrix<double, 6, 6> JJT_damped = J * JT + lambda_sq * Eigen::Matrix<double, 6, 6>::Identity();
        Eigen::Matrix<double, 6, 1> dq_rad = JT * JJT_damped.ldlt().solve(error);

        // 5. 🌟 보폭 제한 (Step Limiting)
        // 에러가 클 때만 제한을 걸어 폭주 방지
        double current_max_step = (error_norm > 0.05) ? MAX_STEP_RAD : MAX_STEP_RAD * 2.0;
        double dq_norm = dq_rad.norm();
        if (dq_norm > current_max_step) {
            dq_rad *= (current_max_step / dq_norm);
        }

        // 6. 업데이트 및 리밋 적용
        q_deg += dq_rad * (180.0 / M_PI);
        q_deg = q_deg.cwiseMax(model->get_min_angles()).cwiseMin(model->get_max_angles());
    }

    return {q_deg, converged};
}

}