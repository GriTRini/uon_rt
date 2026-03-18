#pragma once
#include <Eigen/Dense>
#include "../../model/model.hpp"

namespace rt_control::trajectory::ik {

/**
 * @brief 현재 각도에서 포즈와 자코비안을 동시에 추출
 */
inline std::pair<Eigen::Isometry3d, Eigen::Matrix<double, 6, 6>> 
compute_forward_and_jacobian(const model::RobotModel* model, const rt_control::angles_t& q_deg) {
    Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity(); // Base 좌표계 시작
    const auto& joints = model->get_joints();
    
    std::array<Eigen::Vector3d, 6> z_axes, p_origins;

    // 1. 모든 관절의 Base 기준 위치와 회전축을 먼저 계산
    for (int i = 0; i < 6; ++i) {
        // 관절 i의 설치 위치(Base 기준)까지 변환
        T = T * joints[i].origin; 
        
        // 관절 i의 회전축(Base 기준) 추출
        z_axes[i] = T.linear() * joints[i].axis;
        p_origins[i] = T.translation();

        // 관절 i의 회전 적용 (다음 관절 계산을 위해 T 업데이트)
        double q_rad = q_deg(i) * (M_PI / 180.0);
        T = T * Eigen::AngleAxisd(q_rad, joints[i].axis);
    }

    // 최종 손끝(EE) 위치
    Eigen::Vector3d p_end = T.translation();

    // 2. Jacobian 행렬 구성 (Geometric Jacobian)
    for (int i = 0; i < 6; ++i) {
        // 선속도 자코비안: v = z x (p_ee - p_joint)
        // 💡 p_end와 p_origins[i]의 순서가 바뀌면 방향이 반대로 가니 주의!
        J.block<3, 1>(0, i) = z_axes[i].cross(p_end - p_origins[i]); 
        
        // 각속도 자코비안: w = z
        J.block<3, 1>(3, i) = z_axes[i];
    }

    return {T, J};
}

}