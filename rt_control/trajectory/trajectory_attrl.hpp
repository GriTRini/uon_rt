#pragma once

#include "trajectory_attrj.hpp" // Base 클래스 헤더 (프로젝트 구조에 맞게 유지)
#include "../model/model.hpp"
#include "trajectory_IK/ik_solver.hpp"

namespace rt_control {
namespace trajectory {

/**
 * @brief TrajAttrL 클래스
 * Cartesian Space(작업 공간)에서 목표 Pose를 추종하며 실시간으로 조인트를 업데이트합니다.
 * (🌟 완벽한 직선 이동을 위한 Cartesian Interpolation 및 방어 로직 적용 버전)
 */
class TrajAttrL : public TrajAttrJ {
  public:
    using Base = TrajAttrJ;
    using angles_t = Base::angles_t;
    using value_t = Base::value_t;
    using tmat_t = Eigen::Isometry3d;
    using a_t = Eigen::Matrix<value_t, 6, 1>;

  public:
    TrajAttrL() : m_kp(50.0), m_traj_time(0.0), m_traj_duration(0.0) {}

    TrajAttrL(const model::RobotModel* model, 
              const angles_t &q, const angles_t &dq, const angles_t &ddq,
              const tmat_t& tcp_offset) noexcept
        : Base(q, dq, ddq, 
               model->get_min_angles(), model->get_max_angles(),
               model->get_min_angvels(), model->get_max_angvels(),
               model->get_min_angaccs(), model->get_max_angaccs()),
          m_model(model), m_tcp_offset(tcp_offset), m_kp(50.0),
          m_traj_time(0.0), m_traj_duration(0.0) 
    {
        auto [initial_pose, J] = ik::compute_forward_and_jacobian(m_model, q, m_tcp_offset);
        m_goal_tmat = initial_pose;
        m_start_tmat = initial_pose;
        m_final_tmat = initial_pose;
    }

    [[nodiscard]] bool update(const value_t &dt) noexcept override {
        if (dt <= 0.0) return false;

        // ==============================================================
        // 🌟 1. 1ms 마다 궤적 쪼개기 (Cartesian Interpolation)
        // ==============================================================
        if (m_traj_duration > 0.0 && m_traj_time < m_traj_duration) {
            m_traj_time += dt;
            double s = m_traj_time / m_traj_duration;
            if (s > 1.0) s = 1.0;

            // 부드러운 출발/도착을 위한 S자 커브 (Smoothstep)
            double s_smooth = s * s * (3.0 - 2.0 * s);

            // 위치 선형 보간 (Linear Interpolation)
            Eigen::Vector3d interp_pos = m_start_tmat.translation() * (1.0 - s_smooth) + m_final_tmat.translation() * s_smooth;

            // 회전 구면 선형 보간 (SLERP)
            Eigen::Quaterniond q_start(m_start_tmat.linear());
            Eigen::Quaterniond q_end(m_final_tmat.linear());
            Eigen::Quaterniond interp_rot = q_start.slerp(s_smooth, q_end);

            // 이번 1ms 루프의 "진짜" 목표점 업데이트 (제어기에는 이 미세한 에러만 전달됨)
            m_goal_tmat.translation() = interp_pos;
            m_goal_tmat.linear() = interp_rot.toRotationMatrix();
        }
        // ==============================================================

        // 2. 현재 TCP 포즈 및 Jacobian 획득
        auto [curr_pose, J_curr] = ik::compute_forward_and_jacobian(m_model, Base::angles(), m_tcp_offset);
        
        // 3. 위치 에러 (Position Error)
        Eigen::Vector3d p_err = m_goal_tmat.translation() - curr_pose.translation();
        
        // 4. 회전 에러 (Orientation Error)
        Eigen::Matrix3d R_err = m_goal_tmat.linear() * curr_pose.linear().transpose();
        Eigen::AngleAxisd aa_err(R_err);
        
        Eigen::Vector3d w_err;
        if (aa_err.angle() < 1e-6) {
            w_err.setZero();
        } else {
            w_err = aa_err.axis() * aa_err.angle();
        }

        // 5. Task Velocity Command (v = Kp * error)
        Eigen::Matrix<double, 6, 1> x_dot;
        x_dot.segment<3>(0) = p_err * m_kp;
        x_dot.segment<3>(3) = w_err * m_kp;

        // 6. Jacobian Damped Least Squares (DLS)
        double lambda = 0.01; 
        Eigen::Matrix<double, 6, 6> JJT = J_curr * J_curr.transpose();
        JJT += (lambda * lambda) * Eigen::Matrix<double, 6, 6>::Identity();
        
        Eigen::Matrix<double, 6, 1> dq_rad = J_curr.transpose() * JJT.ldlt().solve(x_dot);

        // 7. 다음 조인트 각도 계산 및 업데이트 (Deg 변환)
        angles_t next_q = Base::angles() + (dq_rad * (180.0 / M_PI) * dt);
        
        Base::set_goal_angles(next_q);
        return Base::update(dt);
    }

    // --- Setters ---
    void set_goal_pose(const tmat_t& goal) noexcept { 
        // ⭐ 핵심 방어 로직: 파이썬 루프에서 같은 목적지를 계속 던져도 리셋되지 않게 방어
        // 위치 오차가 0.1mm 이하이고 회전 오차가 거의 없다면 무시
        if ((m_final_tmat.translation() - goal.translation()).norm() < 1e-4 &&
            m_final_tmat.linear().isApprox(goal.linear(), 1e-4)) {
            return; 
        }

        // 1. 새로운 명령이 들어온 순간의 현재 위치를 시작점으로 고정
        auto [curr_pose, J] = ik::compute_forward_and_jacobian(m_model, Base::angles(), m_tcp_offset);
        m_start_tmat = curr_pose;
        m_final_tmat = goal;
        m_traj_time = 0.0;

        // 2. 이동해야 할 직선 거리 계산 (미터 단위)
        double distance = (m_final_tmat.translation() - m_start_tmat.translation()).norm();

        // 3. 속도 설정 (0.2 = 초당 20cm 이동). 
        double target_speed = 0.20; 
        m_traj_duration = distance / target_speed;

        // 제자리거나 너무 짧은 거리를 명령받았을 때 최소 0.1초는 보장
        if (m_traj_duration < 0.1) m_traj_duration = 0.1;
    }

    void set_kp(value_t kp) noexcept { m_kp = kp; }

    // --- Getters ---
    [[nodiscard]] const tmat_t& goal_pose() const noexcept { return m_final_tmat; }

  protected:
    const model::RobotModel* m_model = nullptr;
    tmat_t m_goal_tmat;     // 매 1ms 마다 갱신되는 징검다리 목표점
    tmat_t m_tcp_offset;
    value_t m_kp; 

    tmat_t m_start_tmat;    // 출발점
    tmat_t m_final_tmat;    // 사용자가 입력한 최종 도착지점
    double m_traj_time;     // 현재 진행 시간
    double m_traj_duration; // 총 소요 시간
};

} // namespace trajectory
} // namespace rt_control