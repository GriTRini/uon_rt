#pragma once

namespace rt_control {
namespace model {
namespace hcr14 {

using angles_t = rt_control::angles_t;

inline const angles_t HW_MIN_ANGLES = (angles_t() << -360.0, -360.0, -165.0, -360.0, -360.0, -360.0).finished();
inline const angles_t HW_MAX_ANGLES = (angles_t() << 360.0, 360.0, 165.0, 360.0, 360.0, 360.0).finished();
inline const angles_t MIN_ANGLES = (HW_MIN_ANGLES.array() + 5.0).matrix();
inline const angles_t MAX_ANGLES = (HW_MAX_ANGLES.array() - 5.0).matrix();

inline const angles_t MAX_ANGVELS = (angles_t() << 155.0, 155.0, 230.0, 270.0, 270.0, 270.0).finished();
inline const angles_t MIN_ANGVELS = -MAX_ANGVELS;

inline const angles_t MAX_ANGACCS = (angles_t() << 155.0, 155.0, 230.0, 270.0, 270.0, 270.0).finished();
inline const angles_t MIN_ANGACCS = -MAX_ANGACCS;

inline const std::array<Joint, 6> joints = {
    Joint{JointID::joint1, LinkID::base, LinkID::link1, xyzrpy(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), Eigen::Vector3d::UnitZ()},
    Joint{JointID::joint2, LinkID::link1, LinkID::link2, xyzrpy(0.0, 0.0, 0.207, 1.5708, 0.0, 0.0), Eigen::Vector3d::UnitZ()},
    Joint{JointID::joint3, LinkID::link2, LinkID::link3, xyzrpy(-0.73, 0.0, 0.0, 0.0, 0.0, 0.0), Eigen::Vector3d::UnitZ()},
    Joint{JointID::joint4, LinkID::link3, LinkID::link4, xyzrpy(-0.5388, 0.0, 0.0, 0.0, 0.0, 0.0), Eigen::Vector3d::UnitZ()},
    Joint{JointID::joint5, LinkID::link4, LinkID::link5, xyzrpy(0.0, 0.0, 0.1847, 1.5708, 0.0, 0.0), Eigen::Vector3d::UnitZ()},
    Joint{JointID::joint6, LinkID::link5, LinkID::link6, xyzrpy(0.0, 0.0, 0.1512, -1.5708, 0.0, 0.0), Eigen::Vector3d::UnitZ()}
};

} // namespace hcr14
} // namespace model
} // namespace rt_control