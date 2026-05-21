#pragma once

#include "trajectory_attrj.hpp" // Base 클래스 헤더 (프로젝트 구조에 맞게 유지)
#include "../model/model.hpp"
#include "trajectory_IK/forward_kinematics.hpp"
#include <cmath>
#include <iostream>
#include <Eigen/Dense>

namespace rt_control {
namespace trajectory {

/**
 * @brief TrajAttrL 클래스
 * Cartesian Space(작업 공간)에서 목표 Pose를 추종하며 실시간으로 조인트를 업데이트합니다.
 * (🌟 안전 설계 적용: 물리적 한계를 벗어난 명령은 시작 전 false를 반환하여 거부)
 */
class TrajAttrL : public TrajAttrJ {
  public:
    using Base = TrajAttrJ;
    using angles_t = Base::angles_t;
    using value_t = Base::value_t;
    using tmat_t = Eigen::Isometry3d;
    using a_t = Eigen::Matrix<value_t, 6, 1>;

  public:
    // 🌟 기본 생성자: 변수 초기화 리스트에 m_task_kp와 m_target_speed 추가
    TrajAttrL() : m_task_kp(50.0), m_target_speed(0.20), m_traj_time(0.0), m_traj_duration(0.0), m_max_reach(0.0) {}

    // 🌟 메인 생성자: 서명(인자 구조)은 그대로 유지하고 초기화만 업데이트
    TrajAttrL(const model::RobotModel* model, 
              const angles_t &q, const angles_t &dq, const angles_t &ddq,
              const tmat_t& tcp_offset) noexcept
        : Base(q, dq, ddq, 
               model->get_min_angles(), model->get_max_angles(),
               model->get_min_angvels(), model->get_max_angvels(),
               model->get_min_angaccs(), model->get_max_angaccs()),
          m_model(model), m_tcp_offset(tcp_offset), 
          m_task_kp(50.0), m_target_speed(0.20), // 기존 m_kp(50.0)를 대체하고 속도 추가
          m_traj_time(0.0), m_traj_duration(0.0), m_max_reach(0.0)
    {
        auto [initial_pose, J] = ik::compute_forward_and_jacobian(m_model, q, m_tcp_offset);
        m_goal_tmat = initial_pose;
        m_start_tmat = initial_pose;
        m_final_tmat = initial_pose;

        // model.hpp에서 찾은 기하학적 진짜 최대 반경 로드
        m_max_reach = m_model->get_max_reach();

        // 🌟 객체 생성 직후 Task와 Joint의 Kp를 50.0으로 동기화
        set_kp(m_task_kp);
    }

    [[nodiscard]] bool update(const value_t &dt) noexcept override {
        if (dt <= 0.0) return false;

        // 1. 1ms 마다 궤적 쪼개기
        if (m_traj_duration > 0.0 && m_traj_time < m_traj_duration) {
            m_traj_time += dt;
            double s = m_traj_time / m_traj_duration;
            if (s > 1.0) s = 1.0;

            double s_smooth = s * s * (3.0 - 2.0 * s);
            Eigen::Vector3d interp_pos = m_start_tmat.translation() * (1.0 - s_smooth) + m_final_tmat.translation() * s_smooth;

            Eigen::Quaterniond q_start(m_start_tmat.linear());
            Eigen::Quaterniond q_end(m_final_tmat.linear());
            Eigen::Quaterniond interp_rot = q_start.slerp(s_smooth, q_end);

            m_goal_tmat.translation() = interp_pos;
            m_goal_tmat.linear() = interp_rot.toRotationMatrix();
        }

        auto [curr_pose, J_curr] = ik::compute_forward_and_jacobian(m_model, Base::angles(), m_tcp_offset);
        
        Eigen::Vector3d p_err = m_goal_tmat.translation() - curr_pose.translation();
        Eigen::Matrix3d R_err = m_goal_tmat.linear() * curr_pose.linear().transpose();
        Eigen::AngleAxisd aa_err(R_err);
        
        Eigen::Vector3d w_err;
        if (aa_err.angle() < 1e-6) {
            w_err.setZero();
        } else {
            w_err = aa_err.axis() * aa_err.angle();
        }

        Eigen::Matrix<double, 6, 1> x_dot;
        // 🌟 m_kp 대신 이름이 명확해진 m_task_kp 적용
        x_dot.segment<3>(0) = p_err * m_task_kp;
        x_dot.segment<3>(3) = w_err * m_task_kp;

        // 🌟 이중 안전장치: 혹시라도 이동 중에 특이점에 걸릴 경우 DLS로 브레이크
        double max_safe_reach = m_max_reach * 0.98; // 98% 지점부터 브레이크
        double target_distance = m_goal_tmat.translation().norm(); 
        double lambda = 0.01; 
        
        if (target_distance > max_safe_reach) {
            lambda = 0.01 + (target_distance - max_safe_reach) * 5.0;
            if (lambda > 0.4) lambda = 0.4;
        }

        Eigen::Matrix<double, 6, 6> JJT = J_curr * J_curr.transpose();
        JJT += (lambda * lambda) * Eigen::Matrix<double, 6, 6>::Identity();
        Eigen::Matrix<double, 6, 1> dq_rad = J_curr.transpose() * JJT.ldlt().solve(x_dot);

        angles_t next_q = Base::angles() + (dq_rad * (180.0 / M_PI) * dt);
        Base::set_goal_angles(next_q);
        
        return Base::update(dt);
    }

    // --- Setters ---
    
    // 🌟 명령 시마다 속도를 덮어쓸 수 있도록 파라미터 추가 (기본값 -1.0)
    [[nodiscard]] bool set_goal_pose(const tmat_t& goal, double override_speed = -1.0) noexcept { 
        
        // ==============================================================
        // 🚨 1. 명령 사전 검증: 로봇의 물리적 한계 반경 초과 여부 체크
        // ==============================================================
        double requested_distance = goal.translation().norm();
        if (requested_distance > m_max_reach) {
            // [에러 발생] 사용자가 로봇 팔 길이보다 더 먼 곳을 명령함
            // (내부 데이터인 m_final_tmat 등을 전혀 건드리지 않고 즉시 거부)
            std::cout << "over " << std::endl;
            return false; 
        }
        // ==============================================================

        // 2. 이미 등록된 목표와 같으면 시간 초기화 없이 통과 (정상 수락)
        if ((m_final_tmat.translation() - goal.translation()).norm() < 1e-4 &&
            m_final_tmat.linear().isApprox(goal.linear(), 1e-4)) {
            return true; 
        }

        // 3. 정상적이고 새로운 목표라면 궤적 초기화 진행
        auto [curr_pose, J] = ik::compute_forward_and_jacobian(m_model, Base::angles(), m_tcp_offset);
        m_start_tmat = curr_pose;
        m_final_tmat = goal;
        m_traj_time = 0.0; 

        double distance = (m_final_tmat.translation() - m_start_tmat.translation()).norm();
        
        // 🌟 입력된 속도가 양수면 그 속도를 쓰고, 아니면 멤버 변수에 저장된 기본 속도 사용
        double apply_speed = (override_speed > 0.0) ? override_speed : m_target_speed;
        m_traj_duration = distance / apply_speed;

        if (m_traj_duration < 0.1) m_traj_duration = 0.1;

        return true; // 명령 정상 수락 완료
    }

    // 🌟 Kp 동기화 (Task와 Joint의 게인을 동시에 업데이트)
    using Base::set_kp; 
    void set_kp(value_t kp) noexcept { 
        m_task_kp = kp;       // Task Space 제어 게인 업데이트
        Base::set_kp(kp);     // Joint Space 제어 게인 업데이트
    }

    // 🌟 목표 속도 변경
    void set_target_speed(double speed) noexcept {
        m_target_speed = std::max(0.01, speed);
    }

    // 기존 Setter 유지
    void set_max_reach(double max_reach) noexcept { m_max_reach = max_reach; }

    // --- Getters --- (기존 형태 완벽 유지)
    [[nodiscard]] const tmat_t& goal_pose() const noexcept { return m_final_tmat; }
    [[nodiscard]] double max_reach() const noexcept { return m_max_reach; }

  protected:
    const model::RobotModel* m_model = nullptr;
    tmat_t m_goal_tmat;     
    tmat_t m_tcp_offset;
    
    value_t m_task_kp;      // 기존 m_kp 대체
    double m_target_speed;  // 🌟 새롭게 추가된 속도 멤버 변수

    tmat_t m_start_tmat;    
    tmat_t m_final_tmat;    
    double m_traj_time;     
    double m_traj_duration; 
    double m_max_reach;
};

} // namespace trajectory
} // namespace rt_control