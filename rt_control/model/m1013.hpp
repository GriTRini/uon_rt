#pragma once

namespace rt_control {
namespace model {
namespace m1013 {

using angles_t = rt_control::angles_t;

inline const angles_t HW_MIN_ANGLES = (angles_t() << -360, -95, -145, -360, -135, -360).finished();
inline const angles_t HW_MAX_ANGLES = (angles_t() << 360, 95, 145, 360, 135, 360).finished();
inline const angles_t MIN_ANGLES = (HW_MIN_ANGLES.array() + 5.0).matrix();
inline const angles_t MAX_ANGLES = (HW_MAX_ANGLES.array() - 5.0).matrix();

inline const angles_t MAX_ANGVELS = (angles_t() << 120, 120, 180, 225, 225, 255).finished();
// inline const angles_t MAX_ANGVELS = (angles_t() << 100, 100, 140, 225, 180, 180).finished();
inline const angles_t MIN_ANGVELS = -MAX_ANGVELS;

inline const angles_t MAX_ANGACCS = (angles_t() << 120, 120, 180, 225, 225, 255).finished();
// inline const angles_t MAX_ANGACCS = (angles_t() << 100, 100, 140, 225, 180, 180).finished();
inline const angles_t MIN_ANGACCS = -MAX_ANGACCS;

inline const std::array<Joint, 6> joints = {
    Joint{JointID::joint1, LinkID::base, LinkID::link1, xyzrpy(0, 0, 0.1525, 0, 0, 0), Eigen::Vector3d::UnitZ()},
    Joint{JointID::joint2, LinkID::link1, LinkID::link2, xyzrpy(0, 0.0345, 0, 0, -M_PI_2, -M_PI_2), Eigen::Vector3d::UnitZ()},
    Joint{JointID::joint3, LinkID::link2, LinkID::link3, xyzrpy(0.62, 0, 0, 0, 0, M_PI_2), Eigen::Vector3d::UnitZ()},
    Joint{JointID::joint4, LinkID::link3, LinkID::link4, xyzrpy(0, -0.559, 0, M_PI_2, 0, 0), Eigen::Vector3d::UnitZ()},
    Joint{JointID::joint5, LinkID::link4, LinkID::link5, xyzrpy(0, 0, 0, -M_PI_2, 0, 0), Eigen::Vector3d::UnitZ()},
    Joint{JointID::joint6, LinkID::link5, LinkID::link6, xyzrpy(0, -0.121, 0, M_PI_2, 0, 0), Eigen::Vector3d::UnitZ()}
};

} // namespace m1013
} // namespace model
} // namespace rt_control