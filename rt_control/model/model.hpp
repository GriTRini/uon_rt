#pragma once

#include <string>
#include <array>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "../core/core.hpp"

namespace rt_control {
namespace model {

// ==========================================
// [공통 정의] 모든 로봇 모델이 공유하는 구조체 및 함수
// ==========================================

enum class LinkID { base, link1, link2, link3, link4, link5, link6 };
enum class JointID { joint1, joint2, joint3, joint4, joint5, joint6 };

struct Joint {
    JointID id;
    LinkID parent;
    LinkID child;
    Eigen::Isometry3d origin; 
    Eigen::Vector3d axis;     
};

inline Eigen::Isometry3d xyzrpy(double x, double y, double z, double r, double p, double yaw) {
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.translation() << x, y, z;
    t.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                  Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX())).toRotationMatrix();
    return t;
}

// ==========================================
// [로봇별 파라미터 로드]
// ==========================================
} // namespace model
} // namespace rt_control

#include "m1013.hpp"
#include "hcr14.hpp"

namespace rt_control {
namespace model {

class RobotModel {
public:
    using angles_t = rt_control::angles_t;
    using value_t = rt_control::value_t;
    using jacobian_t = Eigen::Matrix<double, 6, 6>;

private:
    std::string m_model_name;
    angles_t m_min_angles, m_max_angles;
    angles_t m_min_angvels, m_max_angvels;
    angles_t m_min_angaccs, m_max_angaccs;
    std::array<Joint, 6> m_joints;

public:
    RobotModel() = default;
    explicit RobotModel(const std::string& model_name) { load_model(model_name); }

    void load_model(const std::string& model_name) {
        m_model_name = model_name;
        if (model_name == "m1013") {
            m_min_angles = m1013::MIN_ANGLES; 
            m_max_angles = m1013::MAX_ANGLES;
            m_min_angvels = m1013::MIN_ANGVELS; 
            m_max_angvels = m1013::MAX_ANGVELS;
            m_min_angaccs = m1013::MAX_ANGACCS; 
            m_max_angaccs = m1013::MAX_ANGACCS;
            m_joints = m1013::joints;
        } else if (model_name == "hcr14") {
            m_min_angles = hcr14::MIN_ANGLES; 
            m_max_angles = hcr14::MAX_ANGLES;
            m_min_angvels = hcr14::MAX_ANGVELS; 
            m_max_angvels = hcr14::MAX_ANGVELS;
            m_min_angaccs = hcr14::MAX_ANGACCS; 
            m_max_angaccs = hcr14::MAX_ANGACCS;
            m_joints = hcr14::joints;
        }
    }

    [[nodiscard]] Eigen::Isometry3d forward_kinematics(const angles_t& q_deg) const {
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        for (int i = 0; i < 6; ++i) {
            double q_rad = q_deg(i) * (M_PI / 180.0);
            T = T * m_joints[i].origin * Eigen::AngleAxisd(q_rad, m_joints[i].axis);
        }
        return T;
    }

    [[nodiscard]] std::string get_model_name() const { return m_model_name; }
    [[nodiscard]] const std::array<Joint, 6>& get_joints() const { return m_joints; }
    [[nodiscard]] const angles_t& get_min_angles() const { return m_min_angles; }
    [[nodiscard]] const angles_t& get_max_angles() const { return m_max_angles; }
    [[nodiscard]] const angles_t& get_max_angvels() const { return m_max_angvels; }
    [[nodiscard]] const angles_t& get_max_angaccs() const { return m_max_angaccs; }
    [[nodiscard]] angles_t get_min_angvels() const noexcept { return -m_max_angvels; }
    [[nodiscard]] angles_t get_min_angaccs() const noexcept { return -m_max_angaccs; }  
};

} // namespace model
} // namespace rt_control