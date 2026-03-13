#pragma once

#include "../core/core.hpp"
#include "../model/model.hpp"
#include <iostream>
#include <cmath>

namespace rt_control {
namespace trajectory {

class TrajTrapJ {
  public:
    using angles_t = rt_control::angles_t;
    using value_t = rt_control::value_t;

  public:
    TrajTrapJ(
        const model::RobotModel* robot,
        const angles_t &start_q, 
        const angles_t &start_dq, 
        const angles_t &goal_q,
        const angles_t &goal_dq,
        const angles_t &user_peak_angvels,
        const angles_t &user_peak_angaccs)
    {
        m_start_q = start_q;
        m_goal_q = goal_q;
        
        // 🌟 1. 하드웨어 한계값 가져오기
        angles_t hw_max_v = robot->get_max_angvels();
        angles_t hw_max_a = robot->get_max_angaccs();

        // 🌟 2. 사용자 입력값과 한계값 비교 및 방어(Clipping)
        m_max_v = user_peak_angvels.cwiseAbs().cwiseMin(hw_max_v);
        m_max_a = user_peak_angaccs.cwiseAbs().cwiseMin(hw_max_a);

        // 🌟 3. 에러/경고 알림 로직
        if ((user_peak_angvels.cwiseAbs().array() > hw_max_v.array()).any()) {
            std::cerr << "[경고] TrapJ: 입력된 최대 속도가 로봇 하드웨어 한계를 초과했습니다!\n";
            std::cerr << "  - 입력된 속도: " << user_peak_angvels.transpose() << "\n";
            std::cerr << "  - 실제 적용값: " << m_max_v.transpose() << "\n\n";
        }
        if ((user_peak_angaccs.cwiseAbs().array() > hw_max_a.array()).any()) {
            std::cerr << "[경고] TrapJ: 입력된 최대 가속도가 로봇 하드웨어 한계를 초과했습니다!\n";
            std::cerr << "  - 입력된 가속도: " << user_peak_angaccs.transpose() << "\n";
            std::cerr << "  - 실제 적용값: " << m_max_a.transpose() << "\n\n";
        }

        // 0으로 나누기 방지
        m_max_v = m_max_v.cwiseMax(1e-6);
        m_max_a = m_max_a.cwiseMax(1e-6);

        // 🌟 4. 도달 시간(Duration) 계산
        m_duration = angles_t::Zero();
        for (int i = 0; i < 6; ++i) {
            value_t distance = std::abs(m_goal_q(i) - m_start_q(i));
            if (distance > 0.0001) {
                value_t t_acc = m_max_v(i) / m_max_a(i);
                value_t d_acc = 0.5 * m_max_a(i) * t_acc * t_acc;

                if (distance < 2.0 * d_acc) {
                    m_duration(i) = 2.0 * std::sqrt(distance / m_max_a(i));
                } else {
                    m_duration(i) = 2.0 * t_acc + (distance - 2.0 * d_acc) / m_max_v(i);
                }
            }
        }
        
        // 동기화를 위해 가장 오래 걸리는 축의 시간을 기준으로 삼음
        m_max_duration = m_duration.maxCoeff(); 
        m_current_q = start_q;
        m_current_time = 0.0;
    }

    void update(value_t dt) {
        m_current_time += dt;
        if (m_current_time >= m_max_duration) {
            m_current_q = m_goal_q;
            return;
        }

        value_t s = 0.0;
        value_t t = m_current_time;
        value_t T = m_max_duration;
        
        value_t t_acc = 0.2 * T; 
        if (T > 0) {
             if (t <= t_acc) {
                 s = 0.5 * (1.0 / (t_acc * (T - t_acc))) * t * t;
             } else if (t <= T - t_acc) {
                 s = (0.5 * t_acc + (t - t_acc)) / (T - t_acc);
             } else {
                 value_t dt_dec = t - (T - t_acc);
                 s = 1.0 - 0.5 * (1.0 / (t_acc * (T - t_acc))) * (t_acc - dt_dec) * (t_acc - dt_dec);
             }
        }
        m_current_q = m_start_q + (m_goal_q - m_start_q) * s;
    }

    bool goal_reached() const { return m_current_time >= m_max_duration; }
    bool valid() const { return m_max_duration > 0; }
    
    [[nodiscard]] const angles_t& angles() const { return m_current_q; }
    [[nodiscard]] const angles_t& angvels() const { return m_max_v; }
    [[nodiscard]] const angles_t& angaccs() const { return m_max_a; }

  private:
    angles_t m_start_q, m_goal_q;
    angles_t m_max_v, m_max_a;
    angles_t m_current_q;
    angles_t m_duration;
    value_t m_max_duration;
    value_t m_current_time;
};

} // namespace trajectory
} // namespace rt_control