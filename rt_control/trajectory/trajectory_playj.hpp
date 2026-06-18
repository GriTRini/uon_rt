#pragma once

#include "../core/core.hpp"
#include "../model/model.hpp"
#include <iostream>
#include <vector>
#include <cmath>

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
              value_t p_gain = 10.0) noexcept // 유선형을 더 타이트하고 예쁘게 그리기 위해 게인 상향
        : m_start_angles(start_angles),       // 지나침(Pass-By) 판별용 시작 위치 저장
          m_angles(start_angles), 
          m_angvels(start_angvels),
          m_angaccs(start_angaccs), 
          m_goal_index(0),
          m_waypoints(waypoints),
          m_p_gain(p_gain)
    {
        angles_t hw_max_v = robot->get_max_angvels();
        angles_t hw_max_a = robot->get_max_angaccs();

        m_max_angvels = user_peak_angvels.cwiseAbs().cwiseMin(hw_max_v);
        m_max_angaccs = user_peak_angaccs.cwiseAbs().cwiseMin(hw_max_a);

        m_min_angles = robot->get_min_angles();
        m_max_angles = robot->get_max_angles();
    }

  public:
    void update(const value_t &dt) noexcept {
        // 이미 모든 궤적을 마쳤거나 dt가 비정상일 경우 안전 종료
        if (goal_reached() || dt <= 0.0) {
            m_angvels.setZero();
            m_angaccs.setZero();
            return;
        }

        angles_t target_angles = m_waypoints[m_goal_index].angles;
        value_t current_attrl = m_waypoints[m_goal_index].attrl;
        angles_t error = target_angles - m_angles;
        bool is_last_wp = (m_goal_index == m_waypoints.size() - 1);

        // =================================================================
        // 🌟 1. Pass-by & Radius Detection (지나가도 인지하고 넘어가기)
        // =================================================================
        if (!is_last_wp) {
            bool transition = false;
            
            // 조건 A: 정상적으로 attrl 반경 내에 진입한 경우
            if (error.cwiseAbs().maxCoeff() <= current_attrl) {
                transition = true;
            } 
            // 조건 B: 반경에 살짝 못 미쳤으나, 타겟 지점을 앞질러서 스쳐 지나간 경우 (투영 내적)
            else {
                angles_t prev_wp = (m_goal_index == 0) ? m_start_angles : m_waypoints[m_goal_index - 1].angles;
                angles_t seg_vec = target_angles - prev_wp;
                value_t seg_len_sq = seg_vec.squaredNorm();
                
                if (seg_len_sq > 1e-6) {
                    angles_t rob_vec = m_angles - prev_wp;
                    // 진행 방향(seg_vec) 위로 로봇 위치를 투영했을 때, 선분 길이를 100% 초과했다면 지나친 것!
                    if (rob_vec.dot(seg_vec) >= seg_len_sq) {
                        transition = true;
                    }
                }
            }

            // 타겟 갱신 (논스톱 패스)
            if (transition) {
                m_goal_index++;
                target_angles = m_waypoints[m_goal_index].angles;
                error = target_angles - m_angles;
                is_last_wp = (m_goal_index == m_waypoints.size() - 1);
            }
        } else {
            // [최종 목적지] 오버슈트 없이 완전히 멈출 때까지 대기
            bool pos_reached = error.cwiseAbs().maxCoeff() <= current_attrl;
            bool vel_stopped = m_angvels.cwiseAbs().maxCoeff() <= 0.1; // 0.1 deg/s 이하 제동 완료

            if (pos_reached && vel_stopped) {
                m_goal_index++; 
                m_angvels.setZero();
                m_angaccs.setZero();
                return;
            }
        }

        // =================================================================
        // 🌟 2. Streamlined Vector Scaling (유선형의 완벽한 궤적 생성)
        // =================================================================
        angles_t desired_vel = error * m_p_gain;

        // [Step 2-1] 방향을 100% 유지한 채 속도 스케일링
        value_t max_v_ratio = 0.0;
        for(int i = 0; i < 6; ++i) {
            value_t ratio = std::abs(desired_vel(i)) / m_max_angvels(i);
            if (ratio > max_v_ratio) max_v_ratio = ratio;
        }
        if (max_v_ratio > 1.0) {
            desired_vel /= max_v_ratio; // 모든 관절의 속도를 동일 비율로 줄임
        }

        // [Step 2-2] 최종 목적지 도착 시, 부드러운 제동 곡선 프로파일(Braking) 강제 적용
        if (is_last_wp) {
            value_t max_brake_ratio = 0.0;
            for (int i = 0; i < 6; ++i) {
                value_t e = std::abs(error(i));
                value_t max_a = m_max_angaccs(i) * 0.85; // 85% 가속도로 여유있는 제동
                value_t v_bound = std::sqrt(2.0 * max_a * e);
                value_t ratio = std::abs(desired_vel(i)) / (v_bound + 1e-6);
                if (ratio > max_brake_ratio) max_brake_ratio = ratio;
            }
            if (max_brake_ratio > 1.0) {
                desired_vel /= max_brake_ratio;
            }
        }

        // [Step 2-3] 방향을 100% 유지한 채 가속도 스케일링 (코너가 아름답게 둥글어지는 핵심)
        angles_t desired_acc = (desired_vel - m_angvels) / dt;
        value_t max_a_ratio = 0.0;
        for(int i = 0; i < 6; ++i) {
            value_t ratio = std::abs(desired_acc(i)) / m_max_angaccs(i);
            if (ratio > max_a_ratio) max_a_ratio = ratio;
        }
        if (max_a_ratio > 1.0) {
            desired_acc /= max_a_ratio; // 모든 관절의 가속도를 동일 비율로 줄임
        }

        // 3. 상태 업데이트 및 적분
        m_angaccs = desired_acc;
        m_angvels += m_angaccs * dt;
        m_angles += m_angvels * dt;

        // 최종 하드웨어 한계 보호
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
    angles_t m_start_angles; // 궤적 시작 위치 기록용
    angles_t m_angles, m_angvels, m_angaccs;
    size_t m_goal_index;
    
    std::vector<WaypointJ> m_waypoints;
    value_t m_p_gain;

    angles_t m_min_angles, m_max_angles;
    angles_t m_max_angvels, m_max_angaccs;
};

} // namespace trajectory
} // namespace rt_control