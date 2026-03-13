#pragma once

#include <string>
#include <stdexcept>
#include <array>
#include <vector>
#include <cmath>

#include "../core/core.hpp" 
#include "m1013.hpp"

namespace rt_control {
namespace model {

// IK 연산 결과를 담을 구조체
struct IKResult {
    rt_control::angles_t q;       // 연산된 관절 각도 (Degree)
    bool converged;               // 목표 오차 내로 수렴했는지 여부
};

class RobotModel {
public:
    using angles_t = rt_control::angles_t; 
    using value_t = rt_control::value_t;
    using jacobian_t = Eigen::Matrix<value_t, 6, 6>; // 6x6 자코비안 행렬 타입

    // 🌟 내부 수학 연산을 위한 단위 변환 상수
    static constexpr double DEG2RAD = M_PI / 180.0;
    static constexpr double RAD2DEG = 180.0 / M_PI;

private:
    std::string m_model_name;
    angles_t m_min_angles, m_max_angles;
    angles_t m_min_angvels, m_max_angvels;
    angles_t m_min_angaccs, m_max_angaccs;
    
    // 🌟 핵심: 로봇의 뼈대 구조 (팔 길이, 회전축 방향 등이 담긴 배열)
    std::array<Joint, 6> m_joints;

public:
    RobotModel() = default;

    explicit RobotModel(const std::string& model_name) {
        load_model(model_name);
    }

    void load_model(const std::string& model_name) {
        m_model_name = model_name;
        if (model_name == "m1013") {
            m_min_angles = m1013::MIN_ANGLES; m_max_angles = m1013::MAX_ANGLES;
            m_min_angvels = m1013::MIN_ANGVELS; m_max_angvels = m1013::MAX_ANGVELS;
            m_min_angaccs = m1013::MIN_ANGACCS; m_max_angaccs = m1013::MAX_ANGACCS;
            
            // 여기서 해당 로봇만의 물리적 거리(Link)와 회전축(Joint) 정보가 복사됩니다!
            m_joints = m1013::joints; 
        } else {
            throw std::invalid_argument("지원하지 않는 로봇 모델입니다: " + model_name);
        }
    }

    // --- 순기구학 (사용자는 Degree로 넣지만, 내부는 Radian으로 계산) ---
    [[nodiscard]] Eigen::Isometry3d forward_kinematics(const angles_t& q_deg) const {
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        angles_t q_rad = q_deg * DEG2RAD; // 🌟 Radian 변환
        for (int i = 0; i < 6; ++i) {
            // m_joints[i].origin 에 다음 관절까지의 거리(팔 길이)가 곱해집니다.
            T = T * m_joints[i].origin * Eigen::AngleAxisd(q_rad(i), m_joints[i].axis);
        }
        return T;
    }

    [[nodiscard]] std::vector<Eigen::Isometry3d> forward_kinematics_all(const angles_t& q_deg) const {
        std::vector<Eigen::Isometry3d> poses;
        poses.reserve(7);
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        poses.push_back(T);
        
        angles_t q_rad = q_deg * DEG2RAD; // 🌟 Radian 변환
        for (int i = 0; i < 6; ++i) {
            T = T * m_joints[i].origin * Eigen::AngleAxisd(q_rad(i), m_joints[i].axis);
            poses.push_back(T);
        }
        return poses;
    }

    // --- 기하학적 자코비안 계산 (Radian 기준 연산) ---
    [[nodiscard]] jacobian_t jacobian(const angles_t& q_rad) const {
        jacobian_t J = jacobian_t::Zero();
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        
        std::array<Eigen::Vector3d, 6> z_axes;
        std::array<Eigen::Vector3d, 6> p_origins;
        
        // 순기구학을 돌면서 각 조인트의 회전축(z)과 위치(p)를 저장
        for (int i = 0; i < 6; ++i) {
            T = T * m_joints[i].origin;
            z_axes[i] = T.linear() * m_joints[i].axis; // 월드 좌표계 기준 회전축
            p_origins[i] = T.translation();            // 회전 전의 조인트 원점 위치
            T = T * Eigen::AngleAxisd(q_rad(i), m_joints[i].axis);
        }
        
        Eigen::Vector3d p_end = T.translation(); // 로봇 끝단(End-effector) 위치
        
        // 자코비안 행렬 구성 (열 벡터 조립)
        for (int i = 0; i < 6; ++i) {
            Eigen::Vector3d j_v = z_axes[i].cross(p_end - p_origins[i]); // 선속도 성분
            Eigen::Vector3d j_w = z_axes[i];                             // 각속도 성분
            J.block<3, 1>(0, i) = j_v;
            J.block<3, 1>(3, i) = j_w;
        }
        return J;
    }

    // --- 역기구학 (DLS 알고리즘 적용, Degree/Radian 변환 포함) ---
    [[nodiscard]] IKResult inverse_kinematics(
        const Eigen::Isometry3d& target_pose, 
        const angles_t& initial_q_deg,   // 🌟 입력은 Degree
        size_t max_iter = 100,           
        value_t tol = 1e-4,              
        value_t damping = 0.01) const    
    {
        angles_t q_rad = initial_q_deg * DEG2RAD; // 🌟 내부 연산은 모두 Radian으로 진행
        bool converged = false;
        
        for (size_t iter = 0; iter < max_iter; ++iter) {
            // 현재 q_rad에 대한 FK 직접 계산 (함수 호출 오버헤드 방지)
            Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();
            for (int i = 0; i < 6; ++i) {
                current_pose = current_pose * m_joints[i].origin * Eigen::AngleAxisd(q_rad(i), m_joints[i].axis);
            }
            
            // 1. 위치 오차 
            Eigen::Vector3d p_err = target_pose.translation() - current_pose.translation();
            
            // 2. 회전 오차 
            Eigen::AngleAxisd r_err_aa(target_pose.linear() * current_pose.linear().transpose());
            Eigen::Vector3d w_err = r_err_aa.angle() * r_err_aa.axis();
            
            // 3. 에러 벡터
            Eigen::Matrix<value_t, 6, 1> err;
            err << p_err, w_err;
            
            // 4. 수렴 확인
            if (err.norm() < tol) {
                converged = true;
                break;
            }
            
            // 5. 자코비안 계산 (Radian 전달)
            jacobian_t J = jacobian(q_rad);
            jacobian_t J_T = J.transpose();
            jacobian_t I = jacobian_t::Identity();
            
            // 6. 감쇠 최소제곱법 (DLS)
            angles_t dq_rad = (J_T * J + damping * damping * I).ldlt().solve(J_T * err);
            
            // 7. 각도 업데이트
            q_rad += dq_rad;
        }
        
        // 🌟 최종 결과 반환 시 다시 Degree로 변환 및 Limit 클리핑
        angles_t q_deg = q_rad * RAD2DEG;
        q_deg = q_deg.cwiseMax(m_min_angles).cwiseMin(m_max_angles);
        
        return {q_deg, converged};
    }

    // --- 기존 Getters ---
    [[nodiscard]] std::string get_model_name() const noexcept { return m_model_name; }
    [[nodiscard]] const angles_t& get_min_angles() const noexcept { return m_min_angles; }
    [[nodiscard]] const angles_t& get_max_angles() const noexcept { return m_max_angles; }
    [[nodiscard]] const angles_t& get_min_angvels() const noexcept { return m_min_angvels; }
    [[nodiscard]] const angles_t& get_max_angvels() const noexcept { return m_max_angvels; }
    [[nodiscard]] const angles_t& get_min_angaccs() const noexcept { return m_min_angaccs; }
    [[nodiscard]] const angles_t& get_max_angaccs() const noexcept { return m_max_angaccs; }
};

} // namespace model
} // namespace rt_control