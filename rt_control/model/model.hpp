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
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using angles_t = rt_control::angles_t;
    using value_t = rt_control::value_t;
    using jacobian_t = Eigen::Matrix<double, 6, 6>;

private:
    std::string m_model_name = "none"; // 기본값 설정
    angles_t m_min_angles = angles_t::Zero();
    angles_t m_max_angles = angles_t::Zero();
    angles_t m_min_angvels = angles_t::Zero();
    angles_t m_max_angvels = angles_t::Zero();
    angles_t m_min_angaccs = angles_t::Zero();
    angles_t m_max_angaccs = angles_t::Zero();
    std::array<Joint, 6> m_joints;

public:
    RobotModel() = default;

    // 생성자에서 로드 실패 시 예외를 던짐
    explicit RobotModel(const std::string& model_name) {
        if (!load_model(model_name)) {
            throw std::runtime_error("[RobotModel Error] 지원하지 않는 모델명입니다: " + model_name);
        }
    }

    // 반환 타입을 bool로 변경하여 성공 여부 확인 가능하게 함
    bool load_model(const std::string& model_name) {
        if (model_name == "m1013") {
            m_model_name = model_name;
            m_min_angles = m1013::MIN_ANGLES; 
            m_max_angles = m1013::MAX_ANGLES;
            m_min_angvels = m1013::MIN_ANGVELS; 
            m_max_angvels = m1013::MAX_ANGVELS;
            m_min_angaccs = m1013::MAX_ANGACCS; 
            m_max_angaccs = m1013::MAX_ANGACCS;
            m_joints = m1013::joints;
            return true;
        } else if (model_name == "hcr14") {
            m_model_name = model_name;
            m_min_angles = hcr14::MIN_ANGLES; 
            m_max_angles = hcr14::MAX_ANGLES;
            m_min_angvels = hcr14::MAX_ANGVELS; 
            m_max_angvels = hcr14::MAX_ANGVELS;
            m_min_angaccs = hcr14::MAX_ANGACCS; 
            m_max_angaccs = hcr14::MAX_ANGACCS;
            m_joints = hcr14::joints;
            return true;
        }

        // 일치하는 모델이 없을 경우
        return false;
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