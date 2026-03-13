#pragma once

#include "core/core.hpp"
#include <Eigen/Dense>

namespace rt_control {

// =================================================================
// 1. 궤적 상태 Enum (파이썬 dsenum.TrajState 와 1:1 매칭)
// =================================================================
enum class TrajState {
    STOP = 0,
    STOPPING = 1,
    TRAPJ = 2,
    ATTRJ = 3,
    ATTRL = 4,
    PLAYJ = 5
};

// =================================================================
// 2. 로봇 데이터 상태 구조체 (파이썬 RobotState 에 전달할 묶음 데이터)
// =================================================================
struct RobotStateData {
    angles_t angles;                // 현재 관절 각도 (Radian)
    angles_t angvels;               // 현재 관절 속도 (Radian/s)
    angles_t angaccs;               // 현재 관절 가속도
    angles_t joint_torque;          // 관절 토크 (Nm)
    angles_t joint_ext_torque;      // 외부 관절 토크 (충돌 감지 등)
    angles_t end_ext_force;         // 끝단 외부 힘 (Fx, Fy, Fz, Tx, Ty, Tz)
    
    Eigen::Matrix4d tmat;           // 끝단 위치/회전 행렬 (4x4)
    Eigen::Matrix<double, 6, 6> jmat; // 자코비안 행렬 (6x6)
};

} // namespace rt_control