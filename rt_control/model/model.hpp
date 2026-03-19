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
    
    // 🌟 Robot 클래스(rt_robot.hpp) 호환을 위한 타입 정의 추가
    using jacobian_t = Eigen::Matrix<double, 6, 6>;

private:
    std::string m_model_name;
    angles_t m_min_angles, m_max_angles;
    angles_t m_min_angvels, m_max_angvels;
    angles_t m_min_angaccs, m_max_angaccs;
    
    // 조인트 구성 정보 (IK 모듈에서 참조함)
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
            m_min_angaccs = m1013::MIN_ANGACCS; 
            m_max_angaccs = m1013::MAX_ANGACCS;
            m_joints = m1013::joints;
        }
    }

    // FK 계산
    [[nodiscard]] Eigen::Isometry3d forward_kinematics(const angles_t& q_deg) const {
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        for (int i = 0; i < 6; ++i) {
            double q_rad = q_deg(i) * (M_PI / 180.0);
            T = T * m_joints[i].origin * Eigen::AngleAxisd(q_rad, m_joints[i].axis);
        }
        return T;
    }

    // --- Getters (Robot 클래스 및 TrajGenerator 호환용) ---
    
    // 🌟 rt_robot.hpp에서 private m_model_name에 접근할 수 있도록 Getter 추가
    [[nodiscard]] std::string get_model_name() const { return m_model_name; }

    [[nodiscard]] const std::array<Joint, 6>& get_joints() const { return m_joints; }
    [[nodiscard]] const angles_t& get_min_angles() const { return m_min_angles; }
    [[nodiscard]] const angles_t& get_max_angles() const { return m_max_angles; }
    [[nodiscard]] const angles_t& get_max_angvels() const { return m_max_angvels; }
    
    // 🌟 is_attrl 인자를 유지하되, 내부 로직은 일반 가속도 리밋 반환으로 통일
    [[nodiscard]] const angles_t& get_max_angaccs(bool is_attrl = false) const { 
        return m_max_angaccs; 
    }
};

} // namespace model
} // namespace rt_control