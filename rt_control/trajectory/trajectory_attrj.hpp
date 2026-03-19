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

        // 초기 기본 게인 설정
        set_pd_gains(10.0);

        update_clip();
    }

    virtual bool update(const value_t &dt) noexcept {
        if (dt <= 0.0) return false;

        // PD 제어: a = Kp*(q_target - q) + Kd*(dq_target - dq)
        angles_t error = m_goal_angles - m_angles;
        angles_t error_dot = m_goal_angvels - m_angvels;
        angles_t target_acc = m_kp.cwiseProduct(error) + m_kd.cwiseProduct(error_dot);
        
        // 가속도 제한
        m_angaccs = target_acc.cwiseMax(-m_max_angaccs).cwiseMin(m_max_angaccs);
        
        // 적분 (속도 제한 포함)
        m_angvels = (m_angvels + m_angaccs * dt).cwiseMax(-m_max_angvels).cwiseMin(m_max_angvels);
        m_angles += m_angvels * dt;

        update_clip();
        return true;
    }

    // --- Getters ---
    [[nodiscard]] const angles_t& angles() const noexcept { return m_angles; }
    [[nodiscard]] const angles_t& angvels() const noexcept { return m_angvels; }
    [[nodiscard]] const angles_t& angaccs() const noexcept { return m_angaccs; }
    [[nodiscard]] const angles_t& goal_angles() const noexcept { return m_goal_angles; }

    // --- Setters ---
    /** @brief 목표 각도 설정 (내부에서 조인트 리밋으로 클램핑) */
    void set_goal_angles(const angles_t &new_goal) noexcept { 
        m_goal_angles = new_goal.cwiseMax(m_min_angles).cwiseMin(m_max_angles); 
    }

    /** @brief 개별 Kp 게인 설정 및 자동 임계 댐핑(Kd) 적용 */
    void set_kp(const angles_t &new_kp) noexcept { 
        m_kp = new_kp; 
        for(int i = 0; i < 6; ++i) {
            // Kd = 2 * sqrt(Kp) : 임계 댐핑 조건
            m_kd(i) = 2.0 * std::sqrt(std::max(0.0, m_kp(i)));
        }
    }

    /** @brief 개별 Kd 게인 수동 설정 */
    void set_kd(const angles_t &new_kd) noexcept { 
        m_kd = new_kd; 
    }

    /** @brief 단일 value_t 값으로 전체 관절 PD 게인 설정 */
    void set_pd_gains(value_t kp) noexcept {
        m_kp = angles_t::Constant(kp);
        m_kd = angles_t::Constant(2.0 * std::sqrt(std::max(0.0, kp)));
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