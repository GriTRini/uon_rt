#pragma once

#include "trajectory_attrj.hpp" 
#include "../model/model.hpp"
#include "trajectory_IK/forward_kinematics.hpp"
#include <cmath>
#include <iostream>
#include <Eigen/Dense>

namespace rt_control {
namespace trajectory {

class TrajAttrL : public TrajAttrJ {
  public:
    using Base = TrajAttrJ;
    using angles_t = Base::angles_t;
    using value_t = Base::value_t;
    using tmat_t = Eigen::Isometry3d;
    using a_t = Eigen::Matrix<value_t, 6, 1>;

  public:
    TrajAttrL(value_t attrl_kp = 50.0, value_t attrj_kp = 150.0, value_t attrj_zeta = 1.2, double target_speed = 0.20) 
        : m_task_kp(attrl_kp), m_target_speed(target_speed), 
          m_traj_time(0.0), m_traj_duration(0.0), m_max_reach(0.0) 
    {
        Base::set_kp(attrj_kp, attrj_zeta); 
    }

    TrajAttrL(const model::RobotModel* model, 
              const angles_t &q, const angles_t &dq, const angles_t &ddq,
              const tmat_t& tcp_offset,
              value_t attrl_kp = 50.0,      
              value_t attrj_kp = 150.0,     
              value_t attrj_zeta = 1.2,     
              double target_speed = 0.20) noexcept
        : Base(q, dq, ddq, 
               model->get_min_angles(), model->get_max_angles(),
               model->get_min_angvels(), model->get_max_angvels(),
               model->get_min_angaccs(), model->get_max_angaccs(),
               attrj_kp, attrj_zeta),       
          m_model(model), m_tcp_offset(tcp_offset), 
          m_task_kp(attrl_kp),              
          m_target_speed(target_speed),
          m_traj_time(0.0), m_traj_duration(0.0), m_max_reach(0.0)
    {
        auto [initial_pose, J] = ik::compute_forward_and_jacobian(m_model, q, m_tcp_offset);
        m_goal_tmat = initial_pose;
        m_start_tmat = initial_pose;
        m_final_tmat = initial_pose;
        m_max_reach = m_model->get_max_reach();
    }

    [[nodiscard]] bool update(const value_t &dt) noexcept override {
        if (dt <= 0.0) return false;

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
        x_dot.segment<3>(0) = p_err * m_task_kp;
        x_dot.segment<3>(3) = w_err * m_task_kp;

        double max_safe_reach = m_max_reach * 0.98;
        double target_distance = m_goal_tmat.translation().norm(); 
        double lambda = 0.01; 
        
        if (target_distance > max_safe_reach) {
            lambda = 0.01 + (target_distance - max_safe_reach) * 5.0;
            if (lambda > 0.4) lambda = 0.4;
        }

        Eigen::Matrix<double, 6, 6> JJT = J_curr * J_curr.transpose();
        JJT += (lambda * lambda) * Eigen::Matrix<double, 6, 6>::Identity();
        Eigen::Matrix<double, 6, 1> dq_rad = J_curr.transpose() * JJT.ldlt().solve(x_dot);

        // 🌟 목표 각도뿐만 아니라 목표 속도도 Base(TrajAttrJ)로 전달하여 출렁임 방지
        angles_t target_dq_deg = dq_rad * (180.0 / M_PI);
        angles_t next_q = Base::angles() + (target_dq_deg * dt);
        
        Base::set_goal_angles(next_q);
        Base::set_goal_angvels(target_dq_deg); 
        
        return Base::update(dt);
    }

    [[nodiscard]] bool set_goal_pose(const tmat_t& goal, 
                                     double attrl_kp = 50.0, 
                                     double attrj_kp = 150.0, 
                                     double attrj_zeta = 1.2, 
                                     double target_speed = 0.20) noexcept { 
        
        m_task_kp = attrl_kp;                 
        Base::set_kp(attrj_kp, attrj_zeta);               
        m_target_speed = std::max(0.01, target_speed); 
        
        double requested_distance = goal.translation().norm();
        if (requested_distance > m_max_reach) {
            std::cout << "[Warning] 목표 지점이 로봇 최대 반경을 초과했습니다!" << std::endl;
            return false; 
        }

        if ((m_final_tmat.translation() - goal.translation()).norm() < 1e-4 &&
            m_final_tmat.linear().isApprox(goal.linear(), 1e-4)) {
            return true; 
        }

        auto [curr_pose, J] = ik::compute_forward_and_jacobian(m_model, Base::angles(), m_tcp_offset);
        m_start_tmat = curr_pose;
        m_final_tmat = goal;
        m_traj_time = 0.0; 

        double distance = (m_final_tmat.translation() - m_start_tmat.translation()).norm();
        m_traj_duration = distance / m_target_speed;

        if (m_traj_duration < 0.1) m_traj_duration = 0.1;

        return true; 
    }

    void set_max_reach(double max_reach) noexcept { m_max_reach = max_reach; }

    [[nodiscard]] const tmat_t& goal_pose() const noexcept { return m_final_tmat; }
    [[nodiscard]] double max_reach() const noexcept { return m_max_reach; }

  protected:
    const model::RobotModel* m_model = nullptr;
    tmat_t m_goal_tmat;     
    tmat_t m_tcp_offset;
    
    value_t m_task_kp;      
    double m_target_speed;  

    tmat_t m_start_tmat;    
    tmat_t m_final_tmat;    
    double m_traj_time;     
    double m_traj_duration; 
    double m_max_reach;
};

} // namespace trajectory
} // namespace rt_control