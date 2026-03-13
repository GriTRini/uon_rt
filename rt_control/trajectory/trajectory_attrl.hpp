#pragma once

#include "trajectory_attrj.hpp"
#include "../model/model.hpp"
#include <Eigen/Geometry>
#include <iostream>

namespace rt_control {
namespace trajectory {

class TrajAttrL : public TrajAttrJ {
  public:
    using Base = TrajAttrJ;
    using value_t = rt_control::value_t;
    using angles_t = rt_control::angles_t;

  public:
    TrajAttrL(
        const model::RobotModel* robot_model,
        const angles_t &start_angles,
        const angles_t &start_angvels,
        const angles_t &start_angaccs,
        value_t peak_endvel = 2.0,       // m/s
        value_t peak_endangvel = M_PI,   // rad/s
        value_t peak_endacc = 10.0,      // m/s^2
        value_t peak_endangacc = M_PI*2) // rad/s^2
        // AttrL은 내부 Joint 구동을 위해 부모(AttrJ)에게 로봇의 최대 하드웨어 한계를 넘겨줌
        : Base(robot_model, start_angles, start_angvels, start_angaccs,
               robot_model->get_max_angvels(), robot_model->get_max_angaccs()),
          m_model(robot_model),
          m_peak_endvel(peak_endvel),
          m_peak_endangvel(peak_endangvel),
          m_peak_endacc(peak_endacc),
          m_peak_endangacc(peak_endangacc) 
    {
        m_current_pose = m_model->forward_kinematics(start_angles);
        m_goal_pose = m_current_pose; 
        m_kp_cartesian = 10.0;        
    }

  public:
    void update(const value_t &dt) noexcept {
        if (dt <= 0.0) return;

        m_current_pose = m_model->forward_kinematics(Base::angles());

        Eigen::Vector3d p_err = m_goal_pose.translation() - m_current_pose.translation();
        Eigen::AngleAxisd r_err_aa(m_goal_pose.linear() * m_current_pose.linear().transpose());
        Eigen::Vector3d w_err = r_err_aa.angle() * r_err_aa.axis();

        Eigen::Matrix<value_t, 6, 1> V_target;
        V_target << p_err * m_kp_cartesian, w_err * m_kp_cartesian;

        // Cartesian 속도 한계 클리핑 (방향성은 유지하고 크기만 스케일링)
        value_t linear_vel_norm = V_target.head<3>().norm();
        if (linear_vel_norm > m_peak_endvel) {
            V_target.head<3>() *= (m_peak_endvel / linear_vel_norm);
        }
        value_t angular_vel_norm = V_target.tail<3>().norm();
        if (angular_vel_norm > m_peak_endangvel) {
            V_target.tail<3>() *= (m_peak_endangvel / angular_vel_norm);
        }

        Eigen::Isometry3d next_pose = m_current_pose;
        next_pose.translation() += V_target.head<3>() * dt;
        
        // .toRotationMatrix() 제거된 정상 코드
        if (V_target.tail<3>().norm() > 1e-6) {
            next_pose.linear() = Eigen::AngleAxisd(V_target.tail<3>().norm() * dt, 
                                 V_target.tail<3>().normalized()) * next_pose.linear();
        }

        // 역기구학을 통해 목표 조인트 각도 도출
        auto [new_goal_angles, converged] = m_model->inverse_kinematics(next_pose, Base::angles(), 10, 1e-4, 0.01);

        // 도출된 각도를 부모 클래스(AttrJ)에 넘겨서 모터 한계 내에서 부드럽게 이동시킴
        Base::set_goal_angles(new_goal_angles);
        Base::update(dt);
    }

    void set_goal_pose(const Eigen::Isometry3d &new_goal_pose) noexcept {
        m_goal_pose = new_goal_pose;
    }

    void set_kp_cartesian(value_t kp) noexcept {
        m_kp_cartesian = std::clamp(kp, (value_t)0.0, (value_t)2000.0);
    }

    [[nodiscard]] const Eigen::Isometry3d& current_pose() const noexcept { return m_current_pose; }
    [[nodiscard]] const Eigen::Isometry3d& goal_pose() const noexcept { return m_goal_pose; }

  protected:
    const model::RobotModel* m_model;
    Eigen::Isometry3d m_current_pose;
    Eigen::Isometry3d m_goal_pose;
    value_t m_kp_cartesian;

    value_t m_peak_endvel, m_peak_endangvel;
    value_t m_peak_endacc, m_peak_endangacc;
};

} // namespace trajectory
} // namespace rt_control