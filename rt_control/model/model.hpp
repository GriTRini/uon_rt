#pragma once

#include <string>
#include <array>
#include <Eigen/Dense>
#include "m1013.hpp"

namespace rt_control {
namespace model {

class RobotModel {
public:
    using angles_t = rt_control::angles_t;
    using value_t = rt_control::value_t;

private:
    std::string m_model_name;
    angles_t m_min_angles, m_max_angles;
    angles_t m_min_angvels, m_max_angvels;
    angles_t m_min_angaccs, m_max_angaccs;
    angles_t m_min_attrl_angaccs, m_max_attrl_angaccs;
    
    // 🌟 조인트 구성 정보 (IK 모듈에서 참조함)
    std::array<Joint, 6> m_joints;

public:
    RobotModel() = default;
    explicit RobotModel(const std::string& model_name) { load_model(model_name); }

    void load_model(const std::string& model_name) {
        m_model_name = model_name;
        if (model_name == "m1013") {
            m_min_angles = m1013::MIN_ANGLES; m_max_angles = m1013::MAX_ANGLES;
            m_min_angvels = m1013::MIN_ANGVELS; m_max_angvels = m1013::MAX_ANGVELS;
            m_min_angaccs = m1013::MIN_ANGACCS; m_max_angaccs = m1013::MAX_ANGACCS;
            m_min_attrl_angaccs = m1013::MIN_ATTRL_ANGACCS; 
            m_max_attrl_angaccs = m1013::MAX_ATTRL_ANGACCS;
            m_joints = m1013::joints;
        }
    }

    // FK는 가장 기초적인 정보이므로 유지하거나, 원하시면 이마저도 IK 모듈로 옮길 수 있습니다.
    [[nodiscard]] Eigen::Isometry3d forward_kinematics(const angles_t& q_deg) const {
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        for (int i = 0; i < 6; ++i) {
            double q_rad = q_deg(i) * (M_PI / 180.0);
            T = T * m_joints[i].origin * Eigen::AngleAxisd(q_rad, m_joints[i].axis);
        }
        return T;
    }

    // --- Getters ---
    const std::array<Joint, 6>& get_joints() const { return m_joints; }
    const angles_t& get_min_angles() const { return m_min_angles; }
    const angles_t& get_max_angles() const { return m_max_angles; }
    const angles_t& get_max_angvels() const { return m_max_angvels; }
    const angles_t& get_max_angaccs(bool is_attrl = false) const { 
        return is_attrl ? m_max_attrl_angaccs : m_max_angaccs; 
    }
};

} // namespace model
} // namespace rt_control