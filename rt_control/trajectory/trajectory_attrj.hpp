#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include "../core/core.hpp"
#include "../model/model.hpp"

namespace rt_control {
namespace trajectory {

class TrajAttrJ {
  public:
    using angles_t = rt_control::angles_t;
    using value_t = rt_control::value_t;

  public:
    TrajAttrJ() = default;

    /**
     * @brief TrajAttrJ 생성자
     * @param robot 로봇 모델 (각도 제한 참조용)
     * @param q 현재 관절 각도
     * @param dq 현재 관절 속도
     * @param ddq 현재 관절 가속도
     * @param max_v 최대 속도 (모델에서 전달됨)
     * @param max_a 최대 가속도 (모델에서 전달됨)
     */
    TrajAttrJ(const model::RobotModel* robot,
              const angles_t &q, const angles_t &dq, const angles_t &ddq,
              const angles_t &max_v, const angles_t &max_a) noexcept
        : m_angles(q), m_angvels(dq), m_angaccs(ddq),
          m_goal_angles(q), m_goal_angvels(angles_t::Zero())
    {
        if (robot) {
            m_min_angles = robot->get_min_angles();
            m_max_angles = robot->get_max_angles();
        } else {
            m_min_angles = angles_t::Constant(-360.0);
            m_max_angles = angles_t::Constant(360.0);
        }

        m_max_angvels = max_v;
        m_max_angaccs = max_a;

        // 초기 게인 설정 (Python 규격: Kp = 10.0, 임계 댐핑 적용 Kd = 2 * sqrt(10))
        value_t default_kp = 10.0;
        m_kp = angles_t::Constant(default_kp);
        m_kd = angles_t::Constant(2.0 * std::sqrt(default_kp));

        update_clip();
    }

    /**
     * @brief PD 제어 기반 궤적 업데이트
     */
    virtual bool update(const value_t &dt) noexcept {
        if (dt <= 0.0) return false;

        // 1. 오차 계산 (e = goal - current)
        angles_t error = m_goal_angles - m_angles;
        angles_t error_dot = m_goal_angvels - m_angvels;

        // 2. 목표 가속도 계산 (PD Control: a = Kp*e + Kd*edot)
        angles_t target_acc = m_kp.cwiseProduct(error) + m_kd.cwiseProduct(error_dot);
        
        // 3. 가속도 제한 (Clipping)
        m_angaccs = target_acc.cwiseMax(-m_max_angaccs).cwiseMin(m_max_angaccs);
        
        // 4. 상태 적분 (Integration)
        // 속도 업데이트 및 제한 적용
        angles_t next_vel = m_angvels + m_angaccs * dt;
        m_angvels = next_vel.cwiseMax(-m_max_angvels).cwiseMin(m_max_angvels);
        
        // 위치 업데이트
        m_angles += m_angvels * dt;

        // 5. 최종 하드웨어 한계 보호
        update_clip();
        return true;
    }

    // --- Getters ---
    [[nodiscard]] const angles_t& angles() const noexcept { return m_angles; }
    [[nodiscard]] const angles_t& angvels() const noexcept { return m_angvels; }
    [[nodiscard]] const angles_t& angaccs() const noexcept { return m_angaccs; }
    [[nodiscard]] const angles_t& goal_angles() const noexcept { return m_goal_angles; }

    // --- Setters ---
    void set_goal_angles(const angles_t &new_goal) noexcept { 
        m_goal_angles = new_goal.cwiseMax(m_min_angles).cwiseMin(m_max_angles); 
    }
    
    void set_goal_angvels(const angles_t &new_goal_vel) noexcept { 
        m_goal_angvels = new_goal_vel.cwiseMax(-m_max_angvels).cwiseMin(m_max_angvels); 
    }
    
    void set_kp(const angles_t &new_kp) noexcept { 
        m_kp = new_kp; 
    }
    
    void set_kd(const angles_t &new_kd) noexcept { 
        m_kd = new_kd; 
    }

    /**
     * @brief 단일 값으로 PD 게인을 설정하며 자동으로 임계 댐핑을 적용합니다.
     */
    void set_pd_gains(value_t kp) noexcept {
        m_kp = angles_t::Constant(kp);
        m_kd = angles_t::Constant(2.0 * std::sqrt(kp));
    }

  protected:
    void update_clip() noexcept {
        m_angles  = m_angles.cwiseMax(m_min_angles).cwiseMin(m_max_angles);
        m_angvels = m_angvels.cwiseMax(-m_max_angvels).cwiseMin(m_max_angvels);
        m_angaccs = m_angaccs.cwiseMax(-m_max_angaccs).cwiseMin(m_max_angaccs);
    }

  protected:
    angles_t m_angles = angles_t::Zero();
    angles_t m_angvels = angles_t::Zero();
    angles_t m_angaccs = angles_t::Zero();
    
    angles_t m_goal_angles = angles_t::Zero();
    angles_t m_goal_angvels = angles_t::Zero();
    
    angles_t m_kp, m_kd; 
    
    angles_t m_min_angles, m_max_angles;
    angles_t m_max_angvels, m_max_angaccs;
};

} // namespace trajectory
} // namespace rt_control