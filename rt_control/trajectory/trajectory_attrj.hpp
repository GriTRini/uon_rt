#pragma once

#include "../core/core.hpp"
#include "../model/model.hpp"
#include <iostream>
#include <algorithm>
#include <optional>
#include <cmath>

namespace rt_control {
namespace trajectory {

class TrajAttrJ {
  public:
    using angles_t = rt_control::angles_t;
    using value_t = rt_control::value_t;

    constexpr static value_t KP_MIN = 0;
    constexpr static value_t KP_MAX = 1000;

  public:
    TrajAttrJ() = default;

    TrajAttrJ(
        const model::RobotModel* robot,
        const angles_t &start_angles, 
        const angles_t &start_angvels,
        const angles_t &start_angaccs,
        const std::optional<angles_t> &max_v_limit = std::nullopt,
        const std::optional<angles_t> &max_a_limit = std::nullopt) noexcept
        : m_angles(start_angles),
          m_angvels(start_angvels),
          m_angaccs(start_angaccs),
          m_goal_angles(start_angles),
          m_goal_angvels(angles_t::Zero()),
          m_kp(angles_t::Constant(100.0)),
          m_zeta(1.0) // 🌟 Critical Damping 설정
    {
        angles_t hw_max_v = robot->get_max_angvels();
        angles_t hw_max_a = robot->get_max_angaccs();

        m_max_angvels = max_v_limit.has_value() ? max_v_limit->cwiseAbs().cwiseMin(hw_max_v) : hw_max_v;
        m_max_angaccs = max_a_limit.has_value() ? max_a_limit->cwiseAbs().cwiseMin(hw_max_a) : hw_max_a;

        m_min_angvels = -m_max_angvels;
        m_min_angaccs = -m_max_angaccs;
        m_min_angles = robot->get_min_angles();
        m_max_angles = robot->get_max_angles();

        update_clip();
    }

    virtual bool update(const value_t &dt) noexcept {
        if (dt <= 0.0) return false;
        
        // 🌟 [DSR 방식] 2차 시스템 가속도 계산
        // Acc = Kp * (Goal_Q - Curr_Q) - Kv * Curr_DQ
        // 여기서 Kv = 2 * sqrt(Kp) * zeta (zeta=1.0이면 진동 없음)
        
        angles_t error = m_goal_angles - m_angles;
        
        // 조인트별로 Kp에 따른 Kv(D항) 계산
        angles_t kv;
        for(int i=0; i<6; ++i) {
            kv(i) = 2.0 * std::sqrt(m_kp(i)) * m_zeta;
        }

        // 가속도 결정
        angles_t target_acc = m_kp.cwiseProduct(error) - kv.cwiseProduct(m_angvels);
        
        // 가속도 제한
        m_angaccs = target_acc.cwiseMax(m_min_angaccs).cwiseMin(m_max_angaccs);
        
        // 수치 적분 (가속도 -> 속도 -> 위치)
        m_angvels += m_angaccs * dt;
        
        // 속도 제한
        m_angvels = m_angvels.cwiseMax(m_min_angvels).cwiseMin(m_max_angvels);
        
        m_angles += m_angvels * dt;
        
        update_clip();
        return true;
    }

    void set_kp(const angles_t &new_kp) noexcept {
        m_kp = new_kp.cwiseMax(KP_MIN).cwiseMin(KP_MAX);
    }

    // --- 원본 코드 참고한 에러 관련 함수들 ---
    [[nodiscard]] angles_t angles_error() const noexcept { return m_goal_angles - m_angles; }
    [[nodiscard]] value_t angles_enorm() const noexcept { return angles_error().norm(); }
    [[nodiscard]] value_t angvels_enorm() const noexcept { return m_angvels.norm(); }

    // --- Getters ---
    [[nodiscard]] const angles_t &angles() const noexcept { return m_angles; }
    [[nodiscard]] const angles_t &angvels() const noexcept { return m_angvels; }
    [[nodiscard]] const angles_t &angaccs() const noexcept { return m_angaccs; }
    [[nodiscard]] const angles_t &goal_angles() const noexcept { return m_goal_angles; }

    void set_goal_angles(const angles_t &q) noexcept { m_goal_angles = q; }
    void set_goal_angvels(const angles_t &dq) noexcept { m_goal_angvels = dq; }

  protected:
    void update_clip() noexcept {
        m_angles = m_angles.cwiseMax(m_min_angles).cwiseMin(m_max_angles);
    }

  protected:
    angles_t m_angles, m_angvels, m_angaccs;
    angles_t m_goal_angles, m_goal_angvels, m_kp;
    value_t m_zeta; // 댐핑 계수

    angles_t m_min_angles, m_max_angles;
    angles_t m_min_angvels, m_max_angvels;
    angles_t m_min_angaccs, m_max_angaccs;
};

} // namespace trajectory
} // namespace rt_control