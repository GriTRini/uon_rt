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

    // 🌟 attrj_zeta 기본값 1.2 적용
    TrajAttrJ(const angles_t &q, const angles_t &dq, const angles_t &ddq,
              const angles_t &min_q, const angles_t &max_q,
              const angles_t &min_dq, const angles_t &max_dq,
              const angles_t &min_ddq, const angles_t &max_ddq,
              value_t attrj_kp = 150.0, value_t attrj_zeta = 1.2) noexcept
        : m_angles(q), m_angvels(dq), m_angaccs(ddq),
          m_min_angles(min_q), m_max_angles(max_q),
          m_min_angvels(min_dq), m_max_angvels(max_dq),
          m_min_angaccs(min_ddq), m_max_angaccs(max_ddq)
    {
        m_goal_angles = q;
        m_goal_angvels = angles_t::Zero();
        
        set_kp(attrj_kp, attrj_zeta);
    }

    virtual bool update(const value_t &dt) noexcept {
        if (dt <= 0.0) return false;

        angles_t q_err = m_goal_angles - m_angles;
        angles_t dq_err = m_goal_angvels - m_angvels;
        
        angles_t target_ddq = m_kp.cwiseProduct(q_err) + m_kd.cwiseProduct(dq_err);
        
        m_angaccs = target_ddq.cwiseMax(m_min_angaccs).cwiseMin(m_max_angaccs);
        
        angles_t next_angvels = m_angvels + m_angaccs * dt;
        angles_t clamped_angvels = next_angvels.cwiseMax(m_min_angvels).cwiseMin(m_max_angvels);
        
        m_angaccs = (clamped_angvels - m_angvels) / dt;
        m_angles += (m_angvels * dt) + (0.5 * m_angaccs * dt * dt);
        m_angvels = clamped_angvels;

        update_clip(); 
        return true;
    }

    // --- Getters ---
    [[nodiscard]] const angles_t& angles() const noexcept { return m_angles; }
    [[nodiscard]] const angles_t& angvels() const noexcept { return m_angvels; }
    [[nodiscard]] const angles_t& angaccs() const noexcept { return m_angaccs; }
    [[nodiscard]] const angles_t& goal_angles() const noexcept { return m_goal_angles; }
    [[nodiscard]] const angles_t& goal_angvels() const noexcept { return m_goal_angvels; }

    // --- Setters ---
    void set_goal_angles(const angles_t &new_goal) noexcept { 
        m_goal_angles = new_goal.cwiseMax(m_min_angles).cwiseMin(m_max_angles); 
    }

    void set_goal_angvels(const angles_t &new_goal_dq) noexcept {
        m_goal_angvels = new_goal_dq.cwiseMax(m_min_angvels).cwiseMin(m_max_angvels);
    }

    // =========================================================================
    // 🌟 [추가됨] 1. 조인트 전체에 동일한 Kp 설정
    // =========================================================================
    void set_kp(value_t kp, value_t base_zeta = 1.2) noexcept {
        m_kp = angles_t::Constant(kp);
        
        value_t dynamic_zeta = base_zeta;
        if (kp > 100.0) {
            dynamic_zeta += (kp - 100.0) * 0.002; 
        }

        m_kd = angles_t::Constant(2.0 * dynamic_zeta * std::sqrt(std::max(0.0, kp)));
    }

    // =========================================================================
    // 🌟 [추가됨] 2. 조인트별로 각각 다른 Kp 설정 (배열)
    // =========================================================================
    void set_kp(const angles_t &kp_vec, value_t base_zeta = 1.2) noexcept {
        m_kp = kp_vec;
        
        // 각 조인트마다 개별적으로 Kp 값을 확인하여 댐핑 비율 계산
        for(int i = 0; i < 6; ++i) {
            value_t current_kp = m_kp(i);
            value_t dynamic_zeta = base_zeta;
            
            if (current_kp > 100.0) {
                dynamic_zeta += (current_kp - 100.0) * 0.002;
            }
            
            m_kd(i) = 2.0 * dynamic_zeta * std::sqrt(std::max(0.0, current_kp));
        }
    }

    void set_kd(const angles_t &kd_vec) noexcept { m_kd = kd_vec; }

  protected:
    void update_clip() noexcept {
        m_angles = m_angles.cwiseMax(m_min_angles).cwiseMin(m_max_angles);
    }

  protected:
    angles_t m_angles, m_angvels, m_angaccs;
    angles_t m_goal_angles, m_goal_angvels;
    angles_t m_kp, m_kd; 
    angles_t m_min_angles, m_max_angles;
    angles_t m_min_angvels, m_max_angvels;
    angles_t m_min_angaccs, m_max_angaccs;
};

} // namespace trajectory
} // namespace rt_control