#pragma once

#include "../core/core.hpp"
#include "../model/model.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <optional>
#include <vector>

namespace rt_control {
namespace trajectory {

class TrajTrapJ {
public:
    using angles_t = rt_control::angles_t;
    using value_t = rt_control::value_t;

public:
    TrajTrapJ() = default;

    TrajTrapJ(
        const model::RobotModel* robot,
        const angles_t &start_angles,
        const angles_t &start_angvels,
        const angles_t &goal_angles,
        const angles_t &goal_angvels,
        const std::optional<angles_t> &user_peak_angvels = std::nullopt,
        const std::optional<angles_t> &user_peak_angaccs = std::nullopt,
        const std::optional<value_t> &duration = std::nullopt)
    {
        angles_t hw_max_v = robot->get_max_angvels();
        angles_t hw_max_a = robot->get_max_angaccs();

        // 🌟 [수정] Eigen 타입 불일치 해결을 위해 if-else 사용
        angles_t final_v, final_a;

        if (user_peak_angvels.has_value()) {
            final_v = user_peak_angvels->cwiseAbs().cwiseMin(hw_max_v);
        } else {
            final_v = hw_max_v * 0.1;
        }

        if (user_peak_angaccs.has_value()) {
            final_a = user_peak_angaccs->cwiseAbs().cwiseMin(hw_max_a);
        } else {
            final_a = hw_max_a * 0.1;
        }

        m_start_angles = start_angles;
        m_angles = start_angles;
        m_angvels = start_angvels;
        m_angaccs = angles_t::Zero(6);
        m_current_time = 0.0;
        m_goal_angles = goal_angles.cwiseMax(robot->get_min_angles()).cwiseMin(robot->get_max_angles());
        
        value_t max_dur = 0.0;
        std::vector<value_t> h_list(6);

        for (int i = 0; i < 6; ++i) {
            h_list[i] = m_goal_angles(i) - m_start_angles(i);
            value_t dist = std::abs(h_list[i]);
            if (dist < 1e-6) continue;

            value_t v_max = final_v(i);
            value_t a_max = final_a(i);

            value_t t_unsync;
            if (dist >= (v_max * v_max) / a_max) {
                t_unsync = (dist / v_max) + (v_max / a_max);
            } else {
                t_unsync = 2.0 * std::sqrt(dist / a_max);
            }
            max_dur = std::max(max_dur, t_unsync);
        }

        max_dur += 0.001; 
        max_dur = std::ceil(max_dur * 1000.0) / 1000.0;
        m_max_duration = duration.has_value() ? std::max(max_dur, duration.value()) : max_dur;

        m_sync_v = angles_t::Zero(6);
        m_sync_a = angles_t::Zero(6);
        m_t_acc = angles_t::Zero(6);
        m_dir = angles_t::Zero(6);

        for (int i = 0; i < 6; ++i) {
            value_t dist = std::abs(h_list[i]);
            if (dist < 1e-6) continue;

            m_dir(i) = (h_list[i] > 0) ? 1.0 : -1.0;
            value_t T = m_max_duration;
            m_t_acc(i) = T * 0.25; 
            m_sync_v(i) = dist / (T - m_t_acc(i));
            m_sync_a(i) = m_sync_v(i) / m_t_acc(i);
        }
    }

    void update(value_t dt) { set_time(m_current_time + dt); }
    
    void set_time(value_t new_time) {
        m_current_time = std::clamp(new_time, 0.0, m_max_duration);
        for (int i = 0; i < 6; ++i) {
            value_t dist = std::abs(m_goal_angles(i) - m_start_angles(i));
            if (dist < 1e-6) {
                m_angles(i) = m_goal_angles(i); m_angvels(i) = 0.0; m_angaccs(i) = 0.0;
                continue;
            }
            value_t t = m_current_time, T = m_max_duration, ta = m_t_acc(i), td = T - ta;
            value_t v = m_sync_v(i), a = m_sync_a(i);

            if (t < ta) {
                m_angles(i) = m_start_angles(i) + m_dir(i) * (0.5 * a * t * t);
                m_angvels(i) = m_dir(i) * (a * t); m_angaccs(i) = m_dir(i) * a;
            } else if (t < td) {
                value_t d_acc = 0.5 * a * ta * ta;
                m_angles(i) = m_start_angles(i) + m_dir(i) * (d_acc + v * (t - ta));
                m_angvels(i) = m_dir(i) * v; m_angaccs(i) = 0.0;
            } else if (t < T) {
                value_t dt_end = T - t;
                m_angles(i) = m_goal_angles(i) - m_dir(i) * (0.5 * a * dt_end * dt_end);
                m_angvels(i) = m_dir(i) * (a * dt_end); m_angaccs(i) = -m_dir(i) * a;
            } else {
                m_angles(i) = m_goal_angles(i); m_angvels(i) = 0.0; m_angaccs(i) = 0.0;
            }
        }
    }

    [[nodiscard]] bool goal_reached() const noexcept { return m_current_time >= m_max_duration; }
    [[nodiscard]] const angles_t& angles() const noexcept { return m_angles; }
    [[nodiscard]] const angles_t& angvels() const noexcept { return m_angvels; }
    [[nodiscard]] const angles_t& angaccs() const noexcept { return m_angaccs; }
    
    // 🌟 [추가] TrajGenerator가 사용할 Getter
    [[nodiscard]] const angles_t& goal_angles() const noexcept { return m_goal_angles; }

protected:
    angles_t m_start_angles, m_angles, m_angvels, m_angaccs, m_goal_angles;
    angles_t m_sync_v, m_sync_a, m_t_acc, m_dir;
    value_t m_max_duration = 0.0, m_current_time = 0.0;
};

} // namespace trajectory
} // namespace rt_control