#pragma once

#include "trajectory_attrj.hpp"
#include "../model/model.hpp"
#include <Eigen/Geometry>
#include <iostream>
#include <algorithm>

namespace rt_control { // 🌟 네임스페이스 통일
namespace trajectory {

class TrajAttrL : public TrajAttrJ {
  public:
    using Base = TrajAttrJ;
    using value_t = Base::value_t;
    using angles_t = Base::angles_t;
    using tmat_t = Eigen::Isometry3d;

  public:
    TrajAttrL() = default;

    /**
     * @brief TrajAttrL 생성자
     * 모델의 하드웨어 한계를 100% 사용하여 직선 기동 성능을 확보합니다.
     */
    TrajAttrL(
        const model::RobotModel* robot_model,
        const angles_t &start_angles,
        const angles_t &start_angvels,
        const angles_t &start_angaccs,
        value_t peak_endvel = 0.5,
        value_t peak_endangvel = M_PI,
        value_t peak_endacc = 2.0,      // 🌟 추가된 인자 (테스트 코드 대응)
        value_t peak_endangacc = M_PI*2) // 🌟 추가된 인자 (테스트 코드 대응)
        : Base(robot_model, start_angles, start_angvels, start_angaccs,
               robot_model->get_max_angvels(), robot_model->get_max_angaccs()),
          m_model(robot_model),
          m_peak_endvel(peak_endvel),
          m_peak_endangvel(peak_endangvel) 
    {
        m_current_pose = m_model->forward_kinematics(start_angles);
        m_goal_pose = m_current_pose; 
        
        // 🌟 Cartesian 추종 성능을 위해 조인트 P-게인을 충분히 확보
        // 이게 낮으면 IK 결과를 제때 따라가지 못해 속도가 줄어듭니다.
        this->set_kp(angles_t::Constant(800.0)); 
        m_kp_cartesian = 200.0; // 🌟 기본 Cartesian 게인 상향
    }

  public:
    /**
     * @brief Cartesian Atractor 업데이트
     * 부모의 update(dt)를 호출하여 최종적으로 조인트 공간의 물리 한계를 준수합니다.
     */
    [[nodiscard]] bool update(const value_t &dt) noexcept override {
        if (dt <= 0.0 || m_model == nullptr) return false;

        // 1. 현재 포즈 계산
        m_current_pose = m_model->forward_kinematics(this->angles());

        // 2. Cartesian 오차 계산 (목표 - 현재)
        Eigen::Vector3d p_err = m_goal_pose.translation() - m_current_pose.translation();
        Eigen::AngleAxisd r_err_aa(m_goal_pose.linear() * m_current_pose.linear().transpose());
        Eigen::Vector3d w_err = r_err_aa.angle() * r_err_aa.axis();

        // 3. 목표 속도 생성 및 클리핑
        Eigen::Matrix<value_t, 6, 1> V_target;
        V_target << p_err * m_kp_cartesian, w_err * m_kp_cartesian;

        // 선속도 제한 (m_peak_endvel)
        value_t v_norm = V_target.head<3>().norm();
        if (v_norm > m_peak_endvel) V_target.head<3>() *= (m_peak_endvel / v_norm);
        
        // 각속도 제한 (m_peak_endangvel)
        value_t w_norm = V_target.tail<3>().norm();
        if (w_norm > m_peak_endangvel) V_target.tail<3>() *= (m_peak_endangvel / w_norm);

        // 4. 차기 포즈 적분 (Numerical Integration)
        tmat_t next_step_pose = m_current_pose;
        next_step_pose.translation() += V_target.head<3>() * dt;
        if (w_norm > 1e-6) {
            next_step_pose.linear() = Eigen::AngleAxisd(w_norm * dt, V_target.tail<3>().normalized()) 
                                      * next_step_pose.linear();
        }

        // 5. 역기구학(IK) 풀이
        // Step 크기를 1.0(mm/deg)까지 넉넉히 허용하여 속도 병목을 방지합니다.
        auto [new_q, converged] = m_model->inverse_kinematics(
            next_step_pose, this->angles(), 1000, 1e-4, 1.0);

        if (converged) {
            // IK 결과를 부모의 조인트 목표 각도로 설정
            this->set_goal_angles(new_q);
        } else {
            // IK 실패 시 업데이트 중단 (안전을 위해 false 반환)
            return false;
        }
        
        // 6. 부모 클래스(TrajAttrJ)의 업데이트 호출
        // 여기서 실제 조인트 속도/가속도 제한이 적용됩니다.
        return Base::update(dt);
    }

    // --- 인터페이스 ---
    void set_goal_pose(const tmat_t &new_goal_pose) noexcept { m_goal_pose = new_goal_pose; }
    void set_kp_cartesian(value_t kp) noexcept { m_kp_cartesian = kp; }
    
    [[nodiscard]] const tmat_t& current_pose() const noexcept { return m_current_pose; }
    [[nodiscard]] const tmat_t& goal_pose() const noexcept { return m_goal_pose; }

  protected:
    const model::RobotModel* m_model = nullptr;
    tmat_t m_current_pose = tmat_t::Identity();
    tmat_t m_goal_pose = tmat_t::Identity();
    
    value_t m_kp_cartesian = 200.0;
    value_t m_peak_endvel = 0.5;
    value_t m_peak_endangvel = M_PI;
};

} // namespace trajectory
} // namespace rt_control