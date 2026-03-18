#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>
#include "../model/model.hpp"

namespace rt_control {
namespace trajectory {

class TrajTrapJ {
public:
    using angles_t = rt_control::angles_t;
    using value_t = rt_control::value_t;

    struct Profile {
        value_t t_acc = 0.0, t_const = 0.0, t_dec = 0.0, duration = 0.0;
        value_t v_const = 0.0, a_acc = 0.0, a_dec = 0.0;
        bool valid = false;
    };

public:
    TrajTrapJ() = default;

    TrajTrapJ(const model::RobotModel* model,
              const angles_t& q_start, const angles_t& dq_start,
              const angles_t& q_goal, const angles_t& dq_goal,
              const angles_t& peak_v, const angles_t& peak_a,
              std::optional<value_t> target_duration = std::nullopt)
        : m_q_start(q_start), m_dq_start(dq_start), m_q_goal(q_goal), m_dq_goal(dq_goal),
          m_angles(q_start), m_angvels(dq_start), m_angaccs(angles_t::Zero()),
          m_time(0.0) {
        
        m_profiles.resize(6);
        angles_t individual_durations;

        for (int i = 0; i < 6; ++i) {
            m_profiles[i] = plan_profile(q_start(i), dq_start(i), q_goal(i), dq_goal(i), 
                                         peak_v(i), peak_a(i), std::nullopt);
            individual_durations(i) = m_profiles[i].duration;
        }

        m_max_duration = target_duration.has_value() ? *target_duration : individual_durations.maxCoeff();
        
        if (m_max_duration < 1e-6) m_max_duration = 0.0;

        if (m_max_duration > 0) {
            for (int i = 0; i < 6; ++i) {
                m_profiles[i] = plan_profile(q_start(i), dq_start(i), q_goal(i), dq_goal(i), 
                                             peak_v(i), peak_a(i), m_max_duration);
            }
        }
    }

    void update(value_t dt) {
        if (m_max_duration <= 0) {
            m_angles = m_q_goal;
            m_angvels = m_dq_goal;
            m_angaccs.setZero();
            return;
        }

        m_time += dt;
        // 시간 클램핑 강화
        if (m_time > m_max_duration) m_time = m_max_duration;

        for (int i = 0; i < 6; ++i) {
            calculate_state(i, m_time, m_angles(i), m_angvels(i), m_angaccs(i));
        }
    }

    const angles_t& angles() const { return m_angles; }
    const angles_t& angvels() const { return m_angvels; }
    const angles_t& angaccs() const { return m_angaccs; }
    bool goal_reached() const { return m_time >= (m_max_duration - 1e-9); }
    const angles_t& goal_angles() const { return m_q_goal; }
    value_t duration() const { return m_max_duration; }

    Profile plan_profile(value_t x0, value_t v0, value_t xf, value_t vf, 
                        value_t v_max, value_t a_max, std::optional<value_t> T) {
        Profile p;
        value_t dist = xf - x0;
        value_t dir = (dist >= 0) ? 1.0 : -1.0;

        if (std::abs(dist) < 1e-8 && std::abs(v0) < 1e-8 && std::abs(vf) < 1e-8) {
            p.duration = 0.0; p.valid = true; return p;
        }

        // 기본 가속도 방향 설정
        p.a_acc = a_max * dir;
        p.a_dec = -a_max * dir;

        if (T.has_value() && *T > 1e-6) {
            value_t duration = *T;
            value_t a = p.a_acc - p.a_dec;
            value_t b = (p.a_acc * p.a_dec * duration + p.a_dec * v0 - p.a_acc * vf) * 2.0;
            value_t c = p.a_acc * vf * vf - p.a_dec * v0 * v0 + p.a_acc * p.a_dec * (x0 - xf) * 2.0;
            
            value_t det = std::max(0.0, b * b - 4.0 * a * c);
            value_t v_sol[2];
            v_sol[0] = (-b + std::sqrt(det)) / (2.0 * a);
            v_sol[1] = (-b - std::sqrt(det)) / (2.0 * a);
            
            bool found = false;
            for (int i = 0; i < 2; ++i) {
                value_t v = v_sol[i];
                value_t t1 = (v - v0) / p.a_acc;
                value_t t3 = (vf - v) / p.a_dec;
                value_t t2 = duration - t1 - t3;

                // 핵심: 모든 구간 시간이 양수여야 하며, 방향성(dir)이 일치해야 함
                // J1처럼 짧은 거리는 v의 절대값이 매우 작아야 함
                if (t1 >= -1e-7 && t2 >= -1e-7 && t3 >= -1e-7) {
                    p.v_const = v;
                    p.t_acc = std::max(0.0, t1);
                    p.t_const = std::max(0.0, t2);
                    p.t_dec = std::max(0.0, t3);
                    p.duration = duration;
                    found = true;
                    break; 
                }
            }

            // 만약 해를 못 찾았다면 (매우 짧은 거리), 등속 구간을 0으로 잡고 다시 계산
            if (!found) {
                p.t_const = 0.0;
                // T = t1 + t3 = (v-v0)/a + (vf-v)/d 수식을 만족하는 v를 다시 찾거나
                // 단순히 선형 보간에 가까운 아주 낮은 속도 적용
                p.v_const = (2.0 * dist / duration) - (v0 + vf) / 2.0;
                p.t_acc = duration * 0.5; // 단순 분할
                p.t_dec = duration * 0.5;
                p.duration = duration;
            }
            p.valid = true;
        } else{
            // 최단 시간 모드
            p.v_const = v_max * dir;
            p.t_acc = std::max(0.0, (p.v_const - v0) / p.a_acc);
            p.t_dec = std::max(0.0, (vf - p.v_const) / p.a_dec);
            
            value_t s_acc = (v0 + p.v_const) * p.t_acc * 0.5;
            value_t s_dec = (vf + p.v_const) * p.t_dec * 0.5;
            p.t_const = (dist - s_acc - s_dec) / p.v_const;
            
            if (p.t_const < 0) { // 삼각형 프로파일
                p.t_const = 0.0;
                value_t v_pow = (2.0 * p.a_acc * p.a_dec * dist + p.a_dec * v0 * v0 - p.a_acc * vf * vf) / (p.a_dec - p.a_acc);
                p.v_const = std::sqrt(std::max(0.0, v_pow)) * dir;
                p.t_acc = std::max(0.0, (p.v_const - v0) / p.a_acc);
                p.t_dec = std::max(0.0, (vf - p.v_const) / p.a_dec);
            }
            p.duration = p.t_acc + p.t_const + p.t_dec;
            p.valid = true;
        }
        return p;
    }

    void calculate_state(int i, value_t t, value_t& x, value_t& v, value_t& a) {
        const auto& p = m_profiles[i];
        
        // 종료 조건 도달 시 상태 고정
        if (p.duration <= 1e-9 || t >= (p.duration - 1e-9)) {
            x = m_q_goal(i); v = m_dq_goal(i); a = 0.0;
            return;
        }

        if (t < p.t_acc) {
            x = m_q_start(i) + (m_dq_start(i) + 0.5 * p.a_acc * t) * t;
            v = m_dq_start(i) + p.a_acc * t;
            a = p.a_acc;
        } else if (t < p.t_acc + p.t_const) {
            value_t dt_const = t - p.t_acc;
            value_t x_acc_end = m_q_start(i) + (m_dq_start(i) + p.v_const) * p.t_acc * 0.5;
            x = x_acc_end + p.v_const * dt_const;
            v = p.v_const;
            a = 0.0;
        } else if (t < p.duration) {
            value_t t_dec_start = t - (p.t_acc + p.t_const);
            value_t x_const_end = m_q_start(i) + (m_dq_start(i) + p.v_const) * p.t_acc * 0.5 + p.v_const * p.t_const;
            x = x_const_end + (p.v_const + 0.5 * p.a_dec * t_dec_start) * t_dec_start;
            v = p.v_const + p.a_dec * t_dec_start;
            a = p.a_dec;
        } else {
            x = m_q_goal(i); v = m_dq_goal(i); a = 0.0;
        }
    }

private:
    angles_t m_q_start, m_dq_start, m_q_goal, m_dq_goal;
    angles_t m_angles, m_angvels, m_angaccs;
    std::vector<Profile> m_profiles;
    value_t m_time = 0.0;
    value_t m_max_duration = 0.0;
};

} // namespace trajectory
} // namespace rt_control