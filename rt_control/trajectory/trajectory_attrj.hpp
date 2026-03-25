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

    // DSR 스타일의 생성자 인자 구성
    TrajAttrJ(const angles_t &q, const angles_t &dq, const angles_t &ddq,
              const angles_t &min_q, const angles_t &max_q,
              const angles_t &min_dq, const angles_t &max_dq,
              const angles_t &min_ddq, const angles_t &max_ddq) noexcept
        : m_angles(q), m_angvels(dq), m_angaccs(ddq),
          m_min_angles(min_q), m_max_angles(max_q),
          m_min_angvels(min_dq), m_max_angvels(max_dq),
          m_min_angaccs(min_ddq), m_max_angaccs(max_ddq)
    {
        m_goal_angles = q;
        m_goal_angvels = angles_t::Zero();
        
        // 기본 게인 설정 (DSR은 보통 10~40 사이 사용)
        set_kp(150.0);
    }

    // TrajGenerator 업데이트 루프에서 호출
    virtual bool update(const value_t &dt) noexcept {
        if (dt <= 0.0) return false;

        // 1. PD Control Law: ddq = Kp * (q_err) + Kd * (dq_err)
        angles_t q_err = m_goal_angles - m_angles;
        angles_t dq_err = m_goal_angvels - m_angvels;
        
        // 조인트별 독립 제어
        angles_t target_ddq = m_kp.cwiseProduct(q_err) + m_kd.cwiseProduct(dq_err);
        
        // 2. 가속도 제한 (Saturation)
        m_angaccs = target_ddq.cwiseMax(m_min_angaccs).cwiseMin(m_max_angaccs);
        
        // 3. 적분 및 속도 제한
        m_angvels += m_angaccs * dt;
        m_angvels = m_angvels.cwiseMax(m_min_angvels).cwiseMin(m_max_angvels);
        
        // 4. 위치 업데이트
        m_angles += m_angvels * dt;

        update_clip();
        return true;
    }

    // --- DSR 호환 Getters ---
    [[nodiscard]] const angles_t& angles() const noexcept { return m_angles; }
    [[nodiscard]] const angles_t& angvels() const noexcept { return m_angvels; }
    [[nodiscard]] const angles_t& angaccs() const noexcept { return m_angaccs; }
    [[nodiscard]] const angles_t& goal_angles() const noexcept { return m_goal_angles; }
    [[nodiscard]] const angles_t& goal_angvels() const noexcept { return m_goal_angvels; }

    // --- DSR 호환 Setters ---
    void set_goal_angles(const angles_t &new_goal) noexcept { 
        m_goal_angles = new_goal.cwiseMax(m_min_angles).cwiseMin(m_max_angles); 
    }

    void set_goal_angvels(const angles_t &new_goal_dq) noexcept {
        m_goal_angvels = new_goal_dq.cwiseMax(m_min_angvels).cwiseMin(m_max_angvels);
    }

    /** @brief DSR 스타일의 단일 값 Kp 설정 (임계 댐핑 Kd 자동 계산) */
    void set_kp(value_t kp) noexcept {
        m_kp = angles_t::Constant(kp);
        // Kd = 2 * sqrt(Kp)
        m_kd = angles_t::Constant(2.0 * std::sqrt(std::max(0.0, kp)));
    }

    /** @brief 조인트별 개별 Kp 설정 */
    void set_kp(const angles_t &kp_vec) noexcept {
        m_kp = kp_vec;
        for(int i = 0; i < 6; ++i) {
            m_kd(i) = 2.0 * std::sqrt(std::max(0.0, m_kp(i)));
        }
    }

    void set_kd(const angles_t &kd_vec) noexcept { m_kd = kd_vec; }

  protected:
    void update_clip() noexcept {
        // 하드웨어 한계를 넘지 않도록 최종 클램핑
        m_angles = m_angles.cwiseMax(m_min_angles).cwiseMin(m_max_angles);
    }

  protected:
    angles_t m_angles, m_angvels, m_angaccs;
    angles_t m_goal_angles, m_goal_angvels;
    
    angles_t m_kp, m_kd; 
    
    // DSR은 상하한을 각각 관리하므로 min/max 구조 유지
    angles_t m_min_angles, m_max_angles;
    angles_t m_min_angvels, m_max_angvels;
    angles_t m_min_angaccs, m_max_angaccs;
};

} // namespace trajectory
} // namespace rt_control