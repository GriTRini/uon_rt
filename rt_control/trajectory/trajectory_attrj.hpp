#pragma once

#include "../core/core.hpp"
#include "../model/model.hpp"
#include <iostream>
#include <algorithm>
#include <optional> // 🌟 필수 추가

namespace rt_control {
namespace trajectory {

class TrajAttrJ {
  public:
    using angles_t = rt_control::angles_t;
    using value_t = rt_control::value_t;

    constexpr static value_t KP_MIN = 0;
    constexpr static value_t KP_MAX = 2000; // 🌟 게인 범위를 좀 더 넓힘

  public:
    TrajAttrJ() = default;

    /**
     * @brief TrajAttrJ 생성자
     * @param robot 모델 포인터
     * @param q, dq, ddq 초기 상태
     * @param max_v_limit (선택) 사용자 정의 속도 제한. 없으면 모델의 100% 사용
     * @param max_a_limit (선택) 사용자 정의 가속도 제한. 없으면 모델의 100% 사용
     */
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
          m_kp(angles_t::Constant(100.0)) // 🌟 기본 P-게인을 100으로 상향 (기존 10은 너무 느림)
    {
        // 🌟 1. 하드웨어 한계값 100% 그대로 가져오기 (0.1 곱하지 않음)
        angles_t hw_max_v = robot->get_max_angvels();
        angles_t hw_max_a = robot->get_max_angaccs();

        // 🌟 2. 속도/가속도 제한 설정
        // 사용자 입력값이 있으면 사용하고, 없으면 하드웨어 최대치를 100% 사용
        m_max_angvels = max_v_limit.has_value() ? max_v_limit->cwiseAbs().cwiseMin(hw_max_v) : hw_max_v;
        m_max_angaccs = max_a_limit.has_value() ? max_a_limit->cwiseAbs().cwiseMin(hw_max_a) : hw_max_a;

        m_min_angvels = -m_max_angvels;
        m_min_angaccs = -m_max_angaccs;

        m_min_angles = robot->get_min_angles();
        m_max_angles = robot->get_max_angles();

        update_clip();
    }

    // 🌟 가상 함수(virtual)로 선언하여 AttrL에서 오버라이딩 가능하게 함
    virtual bool update(const value_t &dt) noexcept {
        if (dt <= 0.0) return false;
        
        // 1. 위치 오차 기반 P제어 속도 계산
        angles_t error = m_goal_angles - m_angles;
        angles_t target_vel = m_kp.cwiseProduct(error) + m_goal_angvels;
        
        // 2. 속도 제한 적용
        target_vel = target_vel.cwiseMax(m_min_angvels).cwiseMin(m_max_angvels);
        
        // 3. 목표 가속도 계산 및 제한 적용
        angles_t target_acc = (target_vel - m_angvels) / dt;
        m_angaccs = target_acc.cwiseMax(m_min_angaccs).cwiseMin(m_max_angaccs);
        
        // 4. 수치 적분 (속도, 위치 업데이트)
        m_angvels += m_angaccs * dt;
        m_angles  += m_angvels * dt;
        
        update_clip();
        return true;
    }

    // --- Setters ---
    void set_goal_angles(const angles_t &new_goal_angles) noexcept {
        m_goal_angles = new_goal_angles.cwiseMax(m_min_angles).cwiseMin(m_max_angles);
    }
    void set_goal_angvels(const angles_t &new_goal_angvels) noexcept {
        m_goal_angvels = new_goal_angvels.cwiseMax(m_min_angvels).cwiseMin(m_max_angvels);
    }
    void set_kp(const angles_t &new_kp) noexcept {
        m_kp = new_kp.cwiseMax(KP_MIN).cwiseMin(KP_MAX);
    }

    // --- Getters (Public으로 노출하여 Generator가 상태 복사 가능하게 함) ---
    [[nodiscard]] const angles_t &angles() const noexcept { return m_angles; }
    [[nodiscard]] const angles_t &angvels() const noexcept { return m_angvels; }
    [[nodiscard]] const angles_t &angaccs() const noexcept { return m_angaccs; }
    [[nodiscard]] const angles_t &goal_angles() const noexcept { return m_goal_angles; }

  protected:
    void update_clip() noexcept {
        m_angles  = m_angles.cwiseMax(m_min_angles).cwiseMin(m_max_angles);
        m_angvels = m_angvels.cwiseMax(m_min_angvels).cwiseMin(m_max_angvels);
        m_angaccs = m_angaccs.cwiseMax(m_min_angaccs).cwiseMin(m_max_angaccs);
    }

  protected:
    angles_t m_angles, m_angvels, m_angaccs;
    angles_t m_goal_angles, m_goal_angvels, m_kp;

    angles_t m_min_angles, m_max_angles;
    angles_t m_min_angvels, m_max_angvels;
    angles_t m_min_angaccs, m_max_angaccs;
};

} // namespace trajectory
} // namespace rt_control