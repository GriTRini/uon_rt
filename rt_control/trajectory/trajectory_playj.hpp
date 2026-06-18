#pragma once

#include "../core/core.hpp"
#include "../model/model.hpp"
#include <iostream>
#include <vector>
#include <cmath> // std::sqrt 사용

namespace rt_control {
namespace trajectory {

// 1. 웨이포인트 구조체 정의
struct WaypointJ {
    rt_control::angles_t angles; // 6축 조인트 각도 (q)
    rt_control::value_t attrl;   // 블렌딩 반경 (이 오차 내에 들어오면 다음 목표로 전환)
};

class TrajPlayJ {
  public:
    using angles_t = rt_control::angles_t;
    using value_t = rt_control::value_t;

  public:
    TrajPlayJ() = default;

    TrajPlayJ(const model::RobotModel* robot,
              const angles_t &start_angles, 
              const angles_t &start_angvels,
              const angles_t &start_angaccs, 
              const std::vector<WaypointJ> &waypoints,
              const angles_t &user_peak_angvels,
              const angles_t &user_peak_angaccs,
              value_t p_gain = 5.0) noexcept
        : m_angles(start_angles), 
          m_angvels(start_angvels),
          m_angaccs(start_angaccs), 
          m_goal_index(0),
          m_waypoints(waypoints),
          m_p_gain(p_gain)
    {
        angles_t hw_max_v = robot->get_max_angvels();
        angles_t hw_max_a = robot->get_max_angaccs();

        angles_t safe_peak_v = user_peak_angvels.cwiseAbs().cwiseMin(hw_max_v);
        angles_t safe_peak_a = user_peak_angaccs.cwiseAbs().cwiseMin(hw_max_a);

        m_max_angvels = safe_peak_v;  m_min_angvels = -safe_peak_v;
        m_max_angaccs = safe_peak_a;  m_min_angaccs = -safe_peak_a;
        m_min_angles = robot->get_min_angles();
        m_max_angles = robot->get_max_angles();
    }

  public:
    void update(const value_t &dt) noexcept {
        // 이미 모든 웨이포인트를 돌았거나 dt가 비정상일 경우 제로 출력 및 종료
        if (goal_reached() || dt <= 0.0) {
            m_angvels.setZero();
            m_angaccs.setZero();
            return;
        }

        angles_t target_angles = m_waypoints[m_goal_index].angles;
        value_t current_attrl = m_waypoints[m_goal_index].attrl;

        // 오차 계산
        angles_t error = target_angles - m_angles;
        
        // 현재 타겟이 리스트의 마지막 목적지인지 확인
        bool is_last_wp = (m_goal_index == m_waypoints.size() - 1);

        if (!is_last_wp) {
            // [중간 경유지] 반경(attrl) 안에 들어오면 즉시 타겟 갱신 (논스톱 패스)
            if (error.cwiseAbs().maxCoeff() <= current_attrl) {
                m_goal_index++; 
                target_angles = m_waypoints[m_goal_index].angles;
                error = target_angles - m_angles;
                // 🌟 인덱스가 바뀌었으므로 마지막 목적지인지 다시 확인
                is_last_wp = (m_goal_index == m_waypoints.size() - 1); 
            }
        } else {
            // [최종 목적지] 반경 안에 들어오고 + 속도도 완전히 죽었을 때만 도달(종료) 처리
            bool pos_reached = error.cwiseAbs().maxCoeff() <= current_attrl;
            bool vel_stopped = m_angvels.cwiseAbs().maxCoeff() <= 0.1; // 0.1 deg/s 이하 정지 간주

            if (pos_reached && vel_stopped) {
                m_goal_index++; // 타겟 도달 완료
                m_angvels.setZero();
                m_angaccs.setZero();
                return;
            }
        }

        // 🌟 오버슈트 방지를 위한 속도 계산 및 제동 프로파일 적용
        angles_t desired_vel;
        for (int i = 0; i < 6; ++i) {
            value_t e = error(i);
            value_t v_p = e * m_p_gain; // 기본 P-제어 속도
            
            // 마지막 목적지일 때만 역학적 제동 곡선(Kinematic Braking Bound) 적용
            if (is_last_wp) {
                // 1ms 이산 제어 환경의 오버슈트를 막기 위해 가속도 한계의 80%만 브레이크로 사용
                value_t max_a = m_max_angaccs(i) * 0.8; 
                value_t v_bound = std::sqrt(2.0 * max_a * std::abs(e));
                
                // P-제어 속도가 제동 한계를 넘지 못하도록 클리핑
                if (v_p > v_bound) v_p = v_bound;
                if (v_p < -v_bound) v_p = -v_bound;
            }
            
            // 하드웨어 최대/최소 속도 제한 클리핑
            if (v_p > m_max_angvels(i)) v_p = m_max_angvels(i);
            if (v_p < m_min_angvels(i)) v_p = m_min_angvels(i);
            
            desired_vel(i) = v_p;
        }

        // 가속도 산출 및 하드웨어 최대 가속도 제한 클리핑 (Jerk 방지)
        angles_t desired_acc = (desired_vel - m_angvels) / dt;
        m_angaccs = desired_acc.cwiseMax(m_min_angaccs).cwiseMin(m_max_angaccs);

        // 상태 업데이트 (적분)
        m_angvels += m_angaccs * dt;
        m_angvels = m_angvels.cwiseMax(m_min_angvels).cwiseMin(m_max_angvels);

        m_angles += m_angvels * dt;
        m_angles = m_angles.cwiseMax(m_min_angles).cwiseMin(m_max_angles);
    }

    [[nodiscard]] bool goal_reached() const noexcept {
        return m_goal_index >= m_waypoints.size();
    }

    [[nodiscard]] size_t goal_length() const noexcept {
        return m_waypoints.size();
    }

    [[nodiscard]] const angles_t &angles() const noexcept { return m_angles; }
    [[nodiscard]] const angles_t &angvels() const noexcept { return m_angvels; }
    [[nodiscard]] const angles_t &angaccs() const noexcept { return m_angaccs; }

  protected:
    angles_t m_angles, m_angvels, m_angaccs;
    size_t m_goal_index;
    
    std::vector<WaypointJ> m_waypoints;
    value_t m_p_gain;

    angles_t m_min_angles, m_max_angles;
    angles_t m_min_angvels, m_max_angvels;
    angles_t m_min_angaccs, m_max_angaccs;
};

} // namespace trajectory
} // namespace rt_control