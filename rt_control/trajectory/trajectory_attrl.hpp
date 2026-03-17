#pragma once
#include "trajectory_attrj.hpp"
#include <Eigen/Geometry>

namespace rt_control {
namespace trajectory {

class TrajAttrL : public TrajAttrJ {
public:
    using Base = TrajAttrJ;
    using value_t = Base::value_t;
    using angles_t = Base::angles_t;
    using tmat_t = Eigen::Isometry3d;
    using vec6_t = Eigen::Matrix<value_t, 6, 1>;

    // 🌟 에러 해결 1: 기본 생성자 추가
    TrajAttrL() : m_model(nullptr) {
        m_current_pose.setIdentity();
        m_goal_pose.setIdentity();
        m_current_vel.setZero();
    }

    TrajAttrL(const model::RobotModel* robot_model,
              const angles_t &start_angles,
              const angles_t &start_angvels,
              const angles_t &start_angaccs,
              value_t peak_endvel = 0.5,
              value_t peak_endacc = 2.0)
        : Base(robot_model, start_angles, start_angvels, start_angaccs,
               robot_model->get_max_angvels(), robot_model->get_max_attrl_angaccs()),
          m_model(robot_model),
          m_peak_endvel(peak_endvel),
          m_peak_endacc(peak_endacc) 
    {
        if(m_model) m_current_pose = m_model->forward_kinematics(start_angles);
        m_goal_pose = m_current_pose;
        m_current_vel.setZero();
        set_combined_kp(50.0);
        m_zeta = 1.0;
    }

    // 🌟 에러 해결 2: 인터페이스 공개 (Public)
    void set_goal_pose(const tmat_t &goal) { m_goal_pose = goal; }
    const tmat_t& goal_pose() const { return m_goal_pose; }

    void set_combined_kp(value_t new_kp) noexcept {
        m_kp_combined = new_kp;
        this->set_kp(angles_t::Constant(new_kp));
    }

    [[nodiscard]] bool update(const value_t &dt) noexcept override {
        if (dt <= 0.0 || m_model == nullptr) return false;
        // ... (기존 update 로직 동일) ...
        m_current_pose = m_model->forward_kinematics(this->angles());
        Eigen::Vector3d p_err = m_goal_pose.translation() - m_current_pose.translation();
        Eigen::AngleAxisd r_err_aa(m_goal_pose.linear() * m_current_pose.linear().transpose());
        Eigen::Vector3d w_err = r_err_aa.angle() * r_err_aa.axis();
        vec6_t X_err; X_err << p_err, w_err;

        value_t kv = 2.0 * std::sqrt(m_kp_combined) * m_zeta;
        vec6_t X_accel = m_kp_combined * X_err - kv * m_current_vel;

        value_t acc_norm = X_accel.head<3>().norm();
        if (acc_norm > m_peak_endacc) X_accel.head<3>() *= (m_peak_endacc / acc_norm);

        m_current_vel += X_accel * dt;
        value_t vel_norm = m_current_vel.head<3>().norm();
        if (vel_norm > m_peak_endvel) m_current_vel.head<3>() *= (m_peak_endvel / vel_norm);

        tmat_t next_pose = m_current_pose;
        next_pose.translation() += m_current_vel.head<3>() * dt;
        
        value_t w_norm = m_current_vel.tail<3>().norm();
        if (w_norm > 1e-6) {
            next_pose.linear() = Eigen::AngleAxisd(w_norm * dt, m_current_vel.tail<3>().normalized()) 
                                 * next_pose.linear();
        }

        auto [new_q, converged] = m_model->inverse_kinematics(next_pose, this->angles());
        if (!converged) return false;
        this->set_goal_angles(new_q);
        return Base::update(dt);
    }

protected:
    const model::RobotModel* m_model;
    tmat_t m_current_pose, m_goal_pose;
    vec6_t m_current_vel;
    value_t m_kp_combined = 50.0;
    value_t m_zeta = 1.0;
    value_t m_peak_endvel = 0.5;
    value_t m_peak_endacc = 2.0;
};
}
}