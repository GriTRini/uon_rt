#pragma once
#include <Eigen/Dense>
#include "../../model/model.hpp"

namespace rt_control::trajectory::ik {

/**
 * @brief TCP 오프셋을 반영한 포즈와 자코비안 계산
 */
inline std::pair<Eigen::Isometry3d, Eigen::Matrix<double, 6, 6>> 
compute_forward_and_jacobian(const model::RobotModel* model, 
                             const rt_control::angles_t& q_deg,
                             const Eigen::Isometry3d& tcp_offset) { // 🌟 TCP 오프셋 추가
    Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity(); 
    const auto& joints = model->get_joints();
    
    std::array<Eigen::Vector3d, 6> z_axes, p_origins;

    // 1. 각 관절의 위치 및 축 계산 (Base 기준)
    for (int i = 0; i < 6; ++i) {
        T = T * joints[i].origin; 
        z_axes[i] = T.linear() * joints[i].axis;
        p_origins[i] = T.translation();

        double q_rad = q_deg(i) * (M_PI / 180.0);
        T = T * Eigen::AngleAxisd(q_rad, joints[i].axis);
    }

    // 🌟 2. 최종 TCP 위치 계산 (Flange T * TCP_Offset)
    Eigen::Isometry3d T_tcp = T * tcp_offset;
    Eigen::Vector3d p_tcp = T_tcp.translation();

    // 🌟 3. TCP 기준 Geometric Jacobian 구성
    for (int i = 0; i < 6; ++i) {
        // 선속도 자코비안: v = z x (p_tcp - p_joint)
        J.block<3, 1>(0, i) = z_axes[i].cross(p_tcp - p_origins[i]); 
        // 각속도 자코비안: w = z
        J.block<3, 1>(3, i) = z_axes[i];
    }

    return {T_tcp, J};
}

}