#pragma once

#include "../core/core.hpp"
#include "../model/model.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <optional>

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
        const angles_t &user_peak_angvels,
        const angles_t &user_peak_angaccs,
        const std::optional<value_t> &duration = std::nullopt)
    {
        // 1. 하드웨어 한계값 로드 (로봇 모델 기반)
        m_min_angles = robot->get_min_angles();
        m_max_angles = robot->get_max_angles();
        m_min_angvels = robot->get_min_angvels();
        m_max_angvels = robot->get_max_angvels();
        m_min_angaccs = robot->get_min_angaccs();
        m_max_angaccs = robot->get_max_angaccs();

        // 2. 초기 상태 저장
        m_start_angles = start_angles;
        m_angles = start_angles;
        m_angvels = start_angvels;
        m_angaccs = angles_t::Zero(6); // 초기 가속도는 0으로 가정

        m_current_time = 0.0;

        // 3. 목표값 및 한계값 클리핑 (방어 로직)
        m_goal_angles = goal_angles.cwiseMax(m_min_angles).cwiseMin(m_max_angles);
        
        angles_t peak_vel_clipped = user_peak_angvels.cwiseAbs()
                                    .cwiseMin(m_max_angvels.cwiseAbs())
                                    .cwiseMax(1e-6); // 0 나누기 방지
                                    
        angles_t peak_acc_clipped = user_peak_angaccs.cwiseAbs()
                                    .cwiseMin(m_max_angaccs.cwiseAbs())
                                    .cwiseMax(1e-6);

        // 4. 비동기화 기준 각 축별 최소 도달 시간 계산
        value_t max_dur = 0.0;
        angles_t dur_unsync = angles_t::Zero(6);

        for (int i = 0; i < 6; ++i) {
            value_t distance = std::abs(m_goal_angles(i) - start_angles(i));
            if (distance < 1e-6) continue;

            value_t v = peak_vel_clipped(i);
            value_t a = peak_acc_clipped(i);

            if (distance > (v * v) / a) {
                dur_unsync(i) = (distance / v) + (v / a); // 사다리꼴 형태
            } else {
                dur_unsync(i) = 2.0 * std::sqrt(distance / a); // 삼각형 형태
            }
            max_dur = std::max(max_dur, dur_unsync(i));
        }

        // 5. 최종 동기화 시간(Duration) 결정
        if (duration.has_value() && duration.value() > max_dur) {
            max_dur = duration.value();
        } else {
            max_dur += 0.001; // 수치 오류 방지를 위한 epsilon 추가
            max_dur = std::ceil(max_dur * 1000.0) / 1000.0; // 소수점 3자리 반올림
        }
        m_max_duration = max_dur;

        // 6. 다축 동기화를 위한 프로파일 재계산 (Time Scaling)
        m_sync_v = angles_t::Zero(6);
        m_sync_a = angles_t::Zero(6);
        m_t_acc = angles_t::Zero(6);
        m_dir = angles_t::Zero(6);

        for (int i = 0; i < 6; ++i) {
            value_t distance = m_goal_angles(i) - start_angles(i);
            if (std::abs(distance) < 1e-6) continue;

            m_dir(i) = (distance > 0) ? 1.0 : -1.0;
            
            // 핵심: 가장 오래 걸리는 시간에 맞춰 속도/가속도를 스케일링
            value_t S = dur_unsync(i) / m_max_duration; 
            
            m_sync_v(i) = peak_vel_clipped(i) * S;
            m_sync_a(i) = peak_acc_clipped(i) * S * S;
            
            if (m_sync_a(i) > 1e-6) {
                m_t_acc(i) = m_sync_v(i) / m_sync_a(i);
            }
        }
    }

    // --- 시간 기반 업데이트 로직 ---
    void update(value_t dt) {
        set_time(m_current_time + dt);
    }

    void set_time(value_t new_time) {
        m_current_time = std::max(0.0, std::min(new_time, m_max_duration));

        for (int i = 0; i < 6; ++i) {
            if (m_sync_v(i) < 1e-6) {
                m_angles(i) = m_start_angles(i);
                m_angvels(i) = 0.0;
                m_angaccs(i) = 0.0;
                continue;
            }

            value_t t = m_current_time;
            value_t t_acc = m_t_acc(i);
            value_t t_dec = m_max_duration - t_acc;
            value_t v = m_sync_v(i);
            value_t a = m_sync_a(i);

            if (t <= t_acc) { // 가속 구간
                m_angles(i) = m_start_angles(i) + m_dir(i) * 0.5 * a * t * t;
                m_angvels(i) = m_dir(i) * a * t;
                m_angaccs(i) = m_dir(i) * a;
            } else if (t <= t_dec) { // 등속 구간
                value_t dist_acc = 0.5 * a * t_acc * t_acc;
                m_angles(i) = m_start_angles(i) + m_dir(i) * (dist_acc + v * (t - t_acc));
                m_angvels(i) = m_dir(i) * v;
                m_angaccs(i) = 0.0;
            } else if (t < m_max_duration) { // 감속 구간
                value_t dt_dec = t - t_dec;
                value_t dist_acc = 0.5 * a * t_acc * t_acc;
                value_t dist_const = v * (t_dec - t_acc);
                m_angles(i) = m_start_angles(i) + m_dir(i) * (dist_acc + dist_const + v * dt_dec - 0.5 * a * dt_dec * dt_dec);
                m_angvels(i) = m_dir(i) * (v - a * dt_dec);
                m_angaccs(i) = -m_dir(i) * a;
            } else { // 완료
                m_angles(i) = m_goal_angles(i);
                m_angvels(i) = 0.0;
                m_angaccs(i) = 0.0;
            }
        }

        update_clip();
    }

protected:
    void update_clip() {
        m_angles = m_angles.cwiseMax(m_min_angles).cwiseMin(m_max_angles);
        m_angvels = m_angvels.cwiseMax(m_min_angvels).cwiseMin(m_max_angvels);
        m_angaccs = m_angaccs.cwiseMax(m_min_angaccs).cwiseMin(m_max_angaccs);
    }

public:
    [[nodiscard]] bool valid() const noexcept { return m_max_duration > 0; }
    [[nodiscard]] bool goal_reached() const noexcept { return m_current_time >= m_max_duration; }

    // --- Getters ---
    [[nodiscard]] const angles_t& angles() const noexcept { return m_angles; }
    [[nodiscard]] const angles_t& angvels() const noexcept { return m_angvels; }
    [[nodiscard]] const angles_t& angaccs() const noexcept { return m_angaccs; }
    [[nodiscard]] const angles_t& goal_angles() const noexcept { return m_goal_angles; }
    [[nodiscard]] value_t duration() const noexcept { return m_max_duration; }
    [[nodiscard]] value_t current_time() const noexcept { return m_current_time; }

    // --- Error Metrics ---
    [[nodiscard]] angles_t angles_error() const noexcept { return m_angles - m_goal_angles; }
    [[nodiscard]] value_t angles_enorm() const noexcept { return angles_error().norm(); }

protected:
    // 상태 변수
    angles_t m_start_angles;
    angles_t m_angles;
    angles_t m_angvels;
    angles_t m_angaccs;
    angles_t m_goal_angles;

    // 동기화된 프로파일 변수
    angles_t m_sync_v;
    angles_t m_sync_a;
    angles_t m_t_acc;
    angles_t m_dir;

    // 한계 변수
    angles_t m_min_angles, m_max_angles;
    angles_t m_min_angvels, m_max_angvels;
    angles_t m_min_angaccs, m_max_angaccs;

    value_t m_max_duration = 0.0;
    value_t m_current_time = 0.0;
};

} // namespace trajectory
} // namespace rt_control