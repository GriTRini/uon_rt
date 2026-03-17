#pragma once

#include "trajectory_attrj.hpp"
#include "../model/model.hpp"
#include <Eigen/Geometry>

namespace rt_control {
namespace trajectory {

class TrajAttrL : public TrajAttrJ {
  private:
    using Base = TrajAttrJ;

  public:
    using angles_t = Base::angles_t;
    using value_t = Base::value_t;
    using tmat_t = Eigen::Isometry3d;

    constexpr static size_t IK_STEP_MAX = 1000;
    constexpr static value_t IK_ENORM_THOLD = 1e-4;

  public:
    TrajAttrL() = default;

    // 🌟 생성자 인자를 8개로 업데이트 (테스트 코드와 일치)
    TrajAttrL(const model::RobotModel* model,
              const angles_t &q, const angles_t &dq, const angles_t &ddq,
              value_t peak_endvel = 0.5,
              value_t peak_endangvel = M_PI,
              value_t peak_endacc = 2.0,
              value_t peak_endangacc = M_PI*2) 
        : Base(model, q, dq, ddq), 
          m_model(model),
          m_peak_endvel(peak_endvel),
          m_peak_endangvel(peak_endangvel) 
    {
        m_current_pose = m_model->forward_kinematics(q);
        m_goal_pose = m_current_pose;
        this->set_kp(angles_t::Constant(800.0)); 
        m_kp_cartesian = 200.0; 
    }

  public:
    // 🌟 부모와 동일하게 bool 반환 타입 유지
    [[nodiscard]] bool update(const value_t &dt) noexcept override {
        if (dt <= 0.0) return false;

        m_current_pose = m_model->forward_kinematics(this->m_angles);

        Eigen::Vector3d p_err = m_goal_pose.translation() - m_current_pose.translation();
        Eigen::AngleAxisd r_err_aa(m_goal_pose.linear() * m_current_pose.linear().transpose());
        Eigen::Vector3d w_err = r_err_aa.angle() * r_err_aa.axis();

        Eigen::Matrix<value_t, 6, 1> V_cmd;
        V_cmd << p_err * m_kp_cartesian, w_err * m_kp_cartesian;

        value_t v_norm = V_cmd.head<3>().norm();
        if (v_norm > m_peak_endvel) V_cmd.head<3>() *= (m_peak_endvel / v_norm);
        
        value_t w_norm = V_cmd.tail<3>().norm();
        if (w_norm > m_peak_endangvel) V_cmd.tail<3>() *= (m_peak_endangvel / w_norm);

        Eigen::Isometry3d next_pose = m_current_pose;
        next_pose.translation() += V_cmd.head<3>() * dt;
        if (w_norm > 1e-6) {
            next_pose.linear() = Eigen::AngleAxisd(w_norm * dt, V_cmd.tail<3>().normalized()) * next_pose.linear();
        }

        auto [new_goal_q, converge] = m_model->inverse_kinematics(
            next_pose, this->m_angles, IK_STEP_MAX, IK_ENORM_THOLD, 1.0);

        if (!converge) return false;

        this->set_goal_angles(new_goal_q);
        return Base::update(dt); // 부모 업데이트 호출 결과 반환
    }

    // 🌟 테스트 코드에서 사용하는 Getter 추가
    [[nodiscard]] const tmat_t& current_pose() const noexcept { return m_current_pose; }
    [[nodiscard]] const tmat_t& goal_pose() const noexcept { return m_goal_pose; }

    void set_goal_pose(const tmat_t &new_goal) noexcept { m_goal_pose = new_goal; }
    void set_kp_cartesian(value_t kp) noexcept { m_kp_cartesian = kp; }

  protected:
    const model::RobotModel* m_model = nullptr;
    tmat_t m_current_pose, m_goal_pose;
    value_t m_kp_cartesian;
    value_t m_peak_endvel;
    value_t m_peak_endangvel;
};

} // namespace trajectory
} // namespace rt_control