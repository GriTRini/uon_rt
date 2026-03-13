#pragma once

#include "../core/core.hpp"
#include "../model/model.hpp"
#include <iostream>
#include <algorithm>

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
        const angles_t &user_peak_angvels,
        const angles_t &user_peak_angaccs) noexcept
        : m_angles(start_angles),
          m_angvels(start_angvels),
          m_angaccs(start_angaccs),
          m_goal_angles(start_angles),
          m_goal_angvels(angles_t::Zero()),
          m_kp(angles_t::Constant(10.0))
    {
        // 🌟 1. 하드웨어 한계값 가져오기
        angles_t hw_max_v = robot->get_max_angvels();
        angles_t hw_max_a = robot->get_max_angaccs();

        // 🌟 2. 한계값 클리핑 및 경고 알림
        if ((user_peak_angvels.cwiseAbs().array() > hw_max_v.array()).any()) {
            std::cerr << "[경고] AttrJ: 조인트 최대 속도가 로봇 한계를 초과하여 자동 수정되었습니다.\n";
            std::cerr << "  - 입력값: " << user_peak_angvels.transpose() << "\n";
            std::cerr << "  - 적용값: " << user_peak_angvels.cwiseAbs().cwiseMin(hw_max_v).transpose() << "\n\n";
        }
        if ((user_peak_angaccs.cwiseAbs().array() > hw_max_a.array()).any()) {
            std::cerr << "[경고] AttrJ: 조인트 최대 가속도가 로봇 한계를 초과하여 자동 수정되었습니다.\n";
            std::cerr << "  - 입력값: " << user_peak_angaccs.transpose() << "\n";
            std::cerr << "  - 적용값: " << user_peak_angaccs.cwiseAbs().cwiseMin(hw_max_a).transpose() << "\n\n";
        }

        angles_t safe_peak_v = user_peak_angvels.cwiseAbs().cwiseMin(hw_max_v);
        angles_t safe_peak_a = user_peak_angaccs.cwiseAbs().cwiseMin(hw_max_a);

        m_max_angvels = safe_peak_v;
        m_min_angvels = -safe_peak_v;
        m_max_angaccs = safe_peak_a;
        m_min_angaccs = -safe_peak_a;

        m_min_angles = robot->get_min_angles();
        m_max_angles = robot->get_max_angles();

        update_clip();
    }

    void update(const value_t &dt) noexcept {
        if (dt <= 0.0) return;
        
        // 위치 오차 기반 P제어
        angles_t error = m_goal_angles - m_angles;
        angles_t target_vel = m_kp.cwiseProduct(error) + m_goal_angvels;
        target_vel = target_vel.cwiseMax(m_min_angvels).cwiseMin(m_max_angvels);
        
        angles_t target_acc = (target_vel - m_angvels) / dt;
        m_angaccs = target_acc.cwiseMax(m_min_angaccs).cwiseMin(m_max_angaccs);
        
        m_angvels += m_angaccs * dt;
        m_angles  += m_angvels * dt;
        
        update_clip();
    }

    void set_goal_angles(const angles_t &new_goal_angles) noexcept {
        m_goal_angles = new_goal_angles.cwiseMax(m_min_angles).cwiseMin(m_max_angles);
    }
    void set_goal_angvels(const angles_t &new_goal_angvels) noexcept {
        m_goal_angvels = new_goal_angvels.cwiseMax(m_min_angvels).cwiseMin(m_max_angvels);
    }
    void set_kp(const angles_t &new_kp) noexcept {
        m_kp = new_kp.cwiseMax(KP_MIN).cwiseMin(KP_MAX);
    }

  protected:
    void update_clip() noexcept {
        m_angles  = m_angles.cwiseMax(m_min_angles).cwiseMin(m_max_angles);
        m_angvels = m_angvels.cwiseMax(m_min_angvels).cwiseMin(m_max_angvels);
        m_angaccs = m_angaccs.cwiseMax(m_min_angaccs).cwiseMin(m_max_angaccs);
    }

  public:
    [[nodiscard]] const angles_t &angles() const noexcept { return m_angles; }
    [[nodiscard]] const angles_t &angvels() const noexcept { return m_angvels; }
    [[nodiscard]] const angles_t &angaccs() const noexcept { return m_angaccs; }
    [[nodiscard]] const angles_t &goal_angles() const noexcept { return m_goal_angles; }

  protected:
    angles_t m_angles, m_angvels, m_angaccs;
    angles_t m_goal_angles, m_goal_angvels, m_kp;

    angles_t m_min_angles, m_max_angles;
    angles_t m_min_angvels, m_max_angvels;
    angles_t m_min_angaccs, m_max_angaccs;
};

} // namespace trajectory
} // namespace rt_control