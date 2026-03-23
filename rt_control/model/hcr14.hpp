#pragma once

#include "../core/core.hpp"
#include <Eigen/Geometry>
#include <array>
#include <cmath>

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

namespace hcr14 {

using angles_t = rt_control::angles_t;

// --- 모터 한계값 (Limits) ---
// URDF의 limit 값(radian)을 degree로 변환하여 입력 (1 rad ≈ 57.2958 deg)
// joint 1, 2, 4, 5, 6: ±6.2832 rad (±360 deg)
// joint 3: ±2.8798 rad (±165 deg)
inline const angles_t HW_MIN_ANGLES = (angles_t() << -360.0, -360.0, -165.0, -360.0, -360.0, -360.0).finished();
inline const angles_t HW_MAX_ANGLES = (angles_t() << 360.0, 360.0, 165.0, 360.0, 360.0, 360.0).finished();

// 소프트웨어 안전 마진(5도)을 뺀 한계값
inline const angles_t MIN_ANGLES = (HW_MIN_ANGLES.array() + 5.0).matrix();
inline const angles_t MAX_ANGLES = (HW_MAX_ANGLES.array() - 5.0).matrix();

// URDF의 velocity 값(rad/s)을 deg/s로 변환
inline const angles_t MAX_ANGVELS = (angles_t() << 155.0, 155.0, 230.0, 270.0, 270.0, 270.0).finished();
inline const angles_t MIN_ANGVELS = -MAX_ANGVELS;

// 가속도 한계값 (속도와 동일하게 설정하거나 시스템 응답에 맞춰 조정)
inline const angles_t MAX_ANGACCS = (angles_t() << 155.0, 155.0, 230.0, 270.0, 270.0, 270.0).finished();
inline const angles_t MIN_ANGACCS = -MAX_ANGACCS;

// --- 3D 기구학 파라미터 (URDF Origin 기반) ---
inline const std::array<Joint, 6> joints = {
    // Joint 1: base_link -> link_1 (xyz="0 0 0", rpy="0 0 0")
    Joint{JointID::joint1, LinkID::base, LinkID::link1,
          xyzrpy(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), Eigen::Vector3d::UnitZ()},
          
    // Joint 2: link_1 -> link_2 (xyz="0 0 0.207", rpy="1.5708 0 0")
    Joint{JointID::joint2, LinkID::link1, LinkID::link2,
          xyzrpy(0.0, 0.0, 0.207, 1.5708, 0.0, 0.0), Eigen::Vector3d::UnitZ()},
          
    // Joint 3: link_2 -> link_3 (xyz="-0.73 0 0", rpy="0 0 0")
    Joint{JointID::joint3, LinkID::link2, LinkID::link3,
          xyzrpy(-0.73, 0.0, 0.0, 0.0, 0.0, 0.0), Eigen::Vector3d::UnitZ()},
          
    // Joint 4: link_3 -> link_4 (xyz="-0.5388 0 0", rpy="0 0 0")
    Joint{JointID::joint4, LinkID::link3, LinkID::link4,
          xyzrpy(-0.5388, 0.0, 0.0, 0.0, 0.0, 0.0), Eigen::Vector3d::UnitZ()},
          
    // Joint 5: link_4 -> link_5 (xyz="0 0 0.1847", rpy="1.5708 0 0")
    Joint{JointID::joint5, LinkID::link4, LinkID::link5,
          xyzrpy(0.0, 0.0, 0.1847, 1.5708, 0.0, 0.0), Eigen::Vector3d::UnitZ()},
          
    // Joint 6: link_5 -> link_6 (xyz="0 0 0.1512", rpy="-1.5708 0 0")
    Joint{JointID::joint6, LinkID::link5, LinkID::link6,
          xyzrpy(0.0, 0.0, 0.1512, -1.5708, 0.0, 0.0), Eigen::Vector3d::UnitZ()}
};

} // namespace hcr14
} // namespace model
} // namespace rt_control