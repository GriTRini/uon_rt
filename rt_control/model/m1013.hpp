#pragma once

#include "../core/core.hpp"
#include <Eigen/Geometry>
#include <array>

namespace rt_control {
namespace model {

// 로봇 링크 및 조인트 ID 정의
enum class LinkID { base, link1, link2, link3, link4, link5, link6 };
enum class JointID { joint1, joint2, joint3, joint4, joint5, joint6 };

// XYZ 거리와 Roll-Pitch-Yaw 회전을 Eigen 4x4 변환 행렬로 만들어주는 헬퍼 함수
inline Eigen::Isometry3d xyzrpy(double x, double y, double z, double r, double p, double yaw) {
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.translation() << x, y, z;
    // ZYX 순서 회전 (로보틱스 표준)
    t.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                  Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX())).toRotationMatrix();
    return t;
}

// 개별 조인트의 기구학 구조체
struct Joint {
    JointID id;
    LinkID parent;
    LinkID child;
    Eigen::Isometry3d origin; // 부모 링크로부터의 위치/자세 오프셋
    Eigen::Vector3d axis;     // 회전 축
};

namespace m1013 {

using angles_t = rt_control::angles_t;

// --- 모터 한계값 (Limits) ---
inline const angles_t HW_MIN_ANGLES = (angles_t() << -360, -95, -145, -360, -135, -360).finished();
inline const angles_t HW_MAX_ANGLES = (angles_t() << 360, 95, 145, 360, 135, 360).finished();
inline const angles_t MIN_ANGLES = (HW_MIN_ANGLES.array() + 5.0).matrix();
inline const angles_t MAX_ANGLES = (HW_MAX_ANGLES.array() - 5.0).matrix();
inline const angles_t MAX_ANGVELS = (angles_t() << 120, 120, 180, 225, 225, 255).finished();
inline const angles_t MIN_ANGVELS = -MAX_ANGVELS;
inline const angles_t MAX_ANGACCS = (angles_t() << 1200, 1200, 1800, 2250, 2250, 2550).finished();
inline const angles_t MIN_ANGACCS = -MAX_ANGACCS;

// --- 3D 기구학 파라미터 (DH 파라미터 대체) ---
inline const std::array<Joint, 6> joints = {
    Joint{JointID::joint1, LinkID::base, LinkID::link1,
          xyzrpy(0, 0, 0.1525, 0, 0, 0), Eigen::Vector3d::UnitZ()},
          
    Joint{JointID::joint2, LinkID::link1, LinkID::link2,
          xyzrpy(0, 0.0345, 0, 0, -M_PI_2, -M_PI_2), Eigen::Vector3d::UnitZ()},
          
    Joint{JointID::joint3, LinkID::link2, LinkID::link3,
          xyzrpy(0.62, 0, 0, 0, 0, M_PI_2), Eigen::Vector3d::UnitZ()},
          
    Joint{JointID::joint4, LinkID::link3, LinkID::link4,
          xyzrpy(0, -0.559, 0, M_PI_2, 0, 0), Eigen::Vector3d::UnitZ()},
          
    Joint{JointID::joint5, LinkID::link4, LinkID::link5,
          xyzrpy(0, 0, 0, -M_PI_2, 0, 0), Eigen::Vector3d::UnitZ()},
          
    Joint{JointID::joint6, LinkID::link5, LinkID::link6,
          xyzrpy(0, -0.121, 0, M_PI_2, 0, 0), Eigen::Vector3d::UnitZ()}
};

} // namespace m1013
} // namespace model
} // namespace rt_control