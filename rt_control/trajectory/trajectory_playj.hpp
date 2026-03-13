#pragma once

#include "../core/core.hpp"
#include "../model/model.hpp"
#include <iostream>
#include <optional>

namespace rt_control {
namespace trajectory {

class TrajPlayJ {
  public:
    using angles_t = rt_control::angles_t;
    using value_t = rt_control::value_t;
    // N개의 웨이포인트를 담는 Nx6 동적 행렬 타입
    using angles_set_t = Eigen::Matrix<value_t, Eigen::Dynamic, 6>; 

  public:
    TrajPlayJ() = default;

    TrajPlayJ(const model::RobotModel* robot,
              const angles_t &start_angles, 
              const angles_t &start_angvels,
              const angles_t &start_angaccs, 
              const angles_set_t &goal_angles_set,
              const std::optional<angles_set_t> &goal_angvels_set,
              const std::optional<angles_set_t> &goal_angaccs_set,
              const angles_t &user_peak_angvels,
              const angles_t &user_peak_angaccs) noexcept
        : m_angles(start_angles), 
          m_angvels(start_angvels),
          m_angaccs(start_angaccs), 
          m_goal_index(0),
          m_goal_angles_set(goal_angles_set),
          m_goal_angvels_set(goal_angvels_set),
          m_goal_angaccs_set(goal_angaccs_set)
    {
        // 🌟 1. 하드웨어 한계값 가져오기
        angles_t hw_max_v = robot->get_max_angvels();
        angles_t hw_max_a = robot->get_max_angaccs();

        // 🌟 2. 안전 클리핑 및 경고 알림 로직
        if ((user_peak_angvels.cwiseAbs().array() > hw_max_v.array()).any()) {
            std::cerr << "[경고] PlayJ: 조인트 최대 속도가 로봇 한계를 초과하여 자동 수정되었습니다.\n";
        }
        if ((user_peak_angaccs.cwiseAbs().array() > hw_max_a.array()).any()) {
            std::cerr << "[경고] PlayJ: 조인트 최대 가속도가 로봇 한계를 초과하여 자동 수정되었습니다.\n";
        }

        angles_t safe_peak_v = user_peak_angvels.cwiseAbs().cwiseMin(hw_max_v);
        angles_t safe_peak_a = user_peak_angaccs.cwiseAbs().cwiseMin(hw_max_a);

        m_max_angvels = safe_peak_v;  m_min_angvels = -safe_peak_v;
        m_max_angaccs = safe_peak_a;  m_min_angaccs = -safe_peak_a;
        m_min_angles = robot->get_min_angles();
        m_max_angles = robot->get_max_angles();
    }

  public:
    void update(const value_t &dt) noexcept {
        if (goal_reached() || dt <= 0.0) return;

        // 현재 인덱스의 웨이포인트(행 벡터) 추출
        angles_t new_angles = m_goal_angles_set.row(m_goal_index);

        // 속도가 주어지면 그대로 쓰고, 없으면 이전 스텝과의 미분으로 계산
        angles_t new_angvels = m_goal_angvels_set.has_value() 
                               ? m_goal_angvels_set.value().row(m_goal_index) 
                               : (new_angles - m_angles) / dt;

        // 가속도가 주어지면 그대로 쓰고, 없으면 미분으로 계산
        angles_t new_angaccs = m_goal_angaccs_set.has_value() 
                               ? m_goal_angaccs_set.value().row(m_goal_index) 
                               : (new_angvels - m_angvels) / dt;

        // 한계값 클리핑 (Clipping)
        m_angles  = new_angles.cwiseMax(m_min_angles).cwiseMin(m_max_angles);
        m_angvels = new_angvels.cwiseMax(m_min_angvels).cwiseMin(m_max_angvels);
        m_angaccs = new_angaccs.cwiseMax(m_min_angaccs).cwiseMin(m_max_angaccs);

        // 다음 1ms 스텝을 위해 인덱스 증가
        m_goal_index++;
    }

    [[nodiscard]] bool goal_reached() const noexcept {
        return m_goal_index >= goal_length();
    }

    [[nodiscard]] size_t goal_length() const noexcept {
        return m_goal_angles_set.rows();
    }

    [[nodiscard]] const angles_t &angles() const noexcept { return m_angles; }
    [[nodiscard]] const angles_t &angvels() const noexcept { return m_angvels; }
    [[nodiscard]] const angles_t &angaccs() const noexcept { return m_angaccs; }

  protected:
    angles_t m_angles, m_angvels, m_angaccs;
    size_t m_goal_index;
    
    angles_set_t m_goal_angles_set;
    std::optional<angles_set_t> m_goal_angvels_set;
    std::optional<angles_set_t> m_goal_angaccs_set;

    angles_t m_min_angles, m_max_angles;
    angles_t m_min_angvels, m_max_angvels;
    angles_t m_min_angaccs, m_max_angaccs;
};

} // namespace trajectory
} // namespace rt_control