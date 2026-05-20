#pragma once

#include <mutex>
#include <thread>
#include <iostream>
#include <vector>
#include <queue>
#include <atomic>
#include <string>
#include <functional>
#include <optional>
#include <chrono>
#include <array>
#include <cmath>

#include "../core/robot_base.hpp"
#include "../../hcr/include/clink_api_rpc.h"
#include "../../hcr/include/clink_api_rpc_system.h"

namespace rt_control {

/**
 * @brief 한화로봇 실시간 제어 클래스 
 * RobotBase의 규격을 준수하며 HCR SDK(clink_api)와 통신을 담당합니다.
 */
class HanwhaRobot : public RobotBase {
public:
    HanwhaRobot(const std::string& model_name, 
                const std::chrono::milliseconds update_dt = std::chrono::milliseconds(1))
        : RobotBase(model_name, update_dt),
          cbox_id(0), 
          robot_id(0),
          m_is_rt_control_ready(false)
    {
        std::fill(std::begin(cmd_joint), std::end(cmd_joint), 0.0);
        std::fill(std::begin(last_vel), std::end(last_vel), 0.0);
    }

    ~HanwhaRobot() override {
        disconnect_rt();
        close_connection();
    }

    // ==============================================================
    // 연결 및 초기화 로직 (System)
    // ==============================================================

    bool open_connection(const std::string &strIpAddr = "", const uint32_t usPort = 0) override {
        std::lock_guard<std::mutex> lock(m_control_mutex);
        
        std::string ip = strIpAddr.empty() ? "192.168.1.132" : strIpAddr; // 한화 기본 IP 예시
        CLINK_API_RESULT err;

        // 1. 컨트롤 박스 연결
        err = clink_rpc_system_cbox_connect(CLINK_CONFIG_FILE.c_str(), ip.c_str(), &cbox_id);
        if (err != CLINK_API_RESULT_OK) {
            std::cerr << "[Error] [Hanwha] 컨트롤 박스 연결 실패: " << err << std::endl;
            return false;
        }

        // 2. 제어SW 초기화
        err = clink_rpc_gen_system_create(cbox_id, "", CBOX_MODEL_NAME);
        if (err != CLINK_API_RESULT_OK && err < CLINK_API_RESULT_WARNING_BEGIN) return false;

        // 3. 제어권 획득
        err = clink_rpc_system_control_take(cbox_id);
        if (err != CLINK_API_RESULT_OK) return false;

        // 4. 로봇 생성
        err = clink_rpc_robot_create(cbox_id, ROBOT_MODEL_NAME, "", 0U, &robot_id);
        if (err != CLINK_API_RESULT_OK) return false;

        // 5. EtherCAT 상태 확인 및 대기
        CLINK_ECAT_CONN_STATE ecat_stat = CLINK_ECAT_CONN_STATE_DISCONNECTED;
        clink_rpc_cbox_ecat_connection_state_get(cbox_id, &ecat_stat);
        if (CLINK_ECAT_CONN_STATE_CONNECTED != ecat_stat) {
            char_t valid_event = -1;
            clink_rpc_system_wait_event_group_subgroup(
                cbox_id, CLINK_EVENT_GRP_NOTIFICATION, CLINK_EVENT_SUBGRP_NOTIFICATION_ECAT_CONNECTED,
                1000000, 1, &valid_event);
        }

        // 6. 자동 속도 조절 기능 ON
        clink_rpc_robot_motion_auto_adjust_swith_set(cbox_id, robot_id, CLINK_SWITCH_ON);

        std::cout << "[Info] [Hanwha] 로봇 연결 및 초기화 완료: " << ip << std::endl;
        return true;
    }

    bool close_connection() override {
        std::lock_guard<std::mutex> lock(m_control_mutex);
        clink_rpc_robot_stop(cbox_id, robot_id, 0.5);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        clink_rpc_robot_servo_switch_set(cbox_id, robot_id, CLINK_SWITCH_OFF);
        clink_rpc_system_control_release(cbox_id);
        return true;
    }

    // ==============================================================
    // 실시간 제어(RT) 모드 전환 (Joint Chaser)
    // ==============================================================

    bool connect_rt(const std::string &strIpAddr = "", const uint32_t usPort = 0) override {
        std::lock_guard<std::mutex> lock(m_control_rt_mutex);
        
        // 현재 각도를 제어 변수 초기값으로 설정
        auto current_q = get_current_angles();
        if (current_q) {
            for(int i=0; i<6; i++) cmd_joint[i] = current_q.value()(i);
        }

        clink_rpc_robot_joint_chaser_switch_set(cbox_id, robot_id, CLINK_SWITCH_ON);
        m_is_rt_control_ready.store(true);
        std::cout << "[Info] [Hanwha] 조인트 체이서(RT) 모드 ON" << std::endl;
        return true;
    }

    void disconnect_rt() override {
        std::lock_guard<std::mutex> lock(m_control_rt_mutex);
        clink_rpc_robot_joint_chaser_switch_set(cbox_id, robot_id, CLINK_SWITCH_OFF);
        m_is_rt_control_ready.store(false);
        std::cout << "[Info] [Hanwha] 조인트 체이서(RT) 모드 OFF" << std::endl;
    }

    // ==============================================================
    // 구동 제어 및 데이터 획득 (Robot/Motion)
    // ==============================================================

    bool servo_on() override {
        std::lock_guard<std::mutex> lock(m_control_mutex);
        
        CLINK_SWITCH result;
        clink_rpc_robot_servo_switch_get(cbox_id, robot_id, &result);
        if (result == CLINK_SWITCH_ON) return true;

        if (clink_rpc_robot_servo_switch_set(cbox_id, robot_id, CLINK_SWITCH_ON) != CLINK_API_RESULT_OK) {
            return false;
        }

        // 로봇의 현재 상태로 Trajectory Generator 초기화 (RobotBase 규격)
        const auto start_angles = this->get_current_angles();
        const auto start_angvels = this->get_current_angvels();
        if (start_angles.has_value() && start_angvels.has_value()) {
            m_traj_gen.initialize(m_model, start_angles.value(), start_angvels.value(), angles_t::Zero());
        }

        m_update_timer.start(); // 실시간 제어 루프(update) 구동 시작
        return true;
    }

    bool servo_off() override {
        m_update_timer.stop(); // 실시간 루프 정지
        std::lock_guard<std::mutex> lock(m_control_mutex);

        if (clink_rpc_robot_servo_switch_set(cbox_id, robot_id, CLINK_SWITCH_OFF) == CLINK_API_RESULT_OK) {
            return true;
        }
        return false;
    }

    std::optional<angles_t> get_current_angles() const noexcept override {
        angles_t q;
        clink_float_t val;
        for(int i=0; i<6; i++) {
            clink_rpc_robot_joint_angle_actual_get(cbox_id, robot_id, i, &val);
            q(i) = static_cast<double>(val);
        }
        return q;
    }

    std::optional<angles_t> get_current_angvels() const noexcept override {
        angles_t dq;
        clink_float_t val;
        for(int i=0; i<6; i++) {
            clink_rpc_robot_joint_speed_actual_get(cbox_id, robot_id, i, &val);
            dq(i) = static_cast<double>(val);
        }
        return dq;
    }

    // 목표 도달 여부 확인 (물리적 안착)
    [[nodiscard]] bool goal_reached(
        const std::optional<value_t> &q_th = 2.0, 
        const std::optional<value_t> &p_th = 0.002, 
        const std::optional<value_t> &r_th = 3.0,
        const std::optional<value_t> &v_th = 1.0) const override {
        
        if (!m_traj_gen.goal_reached(q_th, p_th, r_th, std::nullopt, std::nullopt, std::nullopt)) {
            return false;
        }

        auto actual_q_opt = get_current_angles();
        auto actual_dq_opt = get_current_angvels();
        if (!actual_q_opt || !actual_dq_opt) return false;

        if (q_th.has_value() && (actual_q_opt.value() - m_traj_gen.angles()).norm() > q_th.value()) return false;
        if (v_th.has_value() && actual_dq_opt.value().norm() > v_th.value()) return false;

        return true;
    }

    // 미사용 가상 함수 더미 처리
    bool add_tool(const std::string &, float, std::array<float, 3>, std::array<float, 6>) noexcept override { return false; }
    bool set_tool(const std::string &) noexcept override { return false; }
    bool del_tool(const std::string &) noexcept override { return false; }
    bool change_collision_sensitivity(const float) noexcept override { return false; }
    void set_digital_output(int, bool) override {}
    bool get_digital_input(int) override { return false; }
    std::optional<RobotAlarm> pop_alarm() noexcept override { return std::nullopt; }

protected:
    // ==============================================================
    // 🌟 실시간 루프 (Update) - m_update_timer 에 의해 주기적으로 호출됨
    // ==============================================================
    void update() override {
        if (!m_is_rt_control_ready.load()) return;

        // 1. dt 계산 및 궤적 갱신
        const double dt = std::chrono::duration_cast<std::chrono::microseconds>(m_update_timer.interval).count() / 1000000.0;
        if (dt <= 1e-6) return;
        m_traj_gen.update(dt);

        // 2. Trajectory Generator에서 타겟 조인트 값 가져오기
        const angles_t& target_q = m_traj_gen.angles();
        clink_float_t next_target[6];

        // 3. SMC 제어기 로직 적용
        for (int i = 0; i < 6; i++) {
            double error = target_q(i) - cmd_joint[i];

            // (s = de/dt + lambda * e)
            double s = LAMBDA * error - last_vel[i];

            // 제어 입력 계산
            double reach_vel = K_GAIN * std::tanh(s / PHI);
            double current_vel = reach_vel + (LAMBDA * error);

            // 상태 업데이트
            cmd_joint[i] += current_vel * dt;
            last_vel[i] = current_vel;

            next_target[i] = static_cast<clink_float_t>(cmd_joint[i]);
        }

        // 4. 로봇에 실시간 각도 전송
        clink_rpc_robot_joint_chaser_move(cbox_id, robot_id, 6, next_target);
    }

private:
    // 상수 및 설정
    const CLINK_CBOX_MODEL  CBOX_MODEL_NAME = CLINK_CBOX_MODEL_3GEN;
    const CLINK_ROBOT_MODEL ROBOT_MODEL_NAME = CLINK_ROBOT_MODEL_HCR14;
    const std::string       CLINK_CONFIG_FILE = "/config/config_rpc.ini"; // 실제 경로로 수정 필요

    // SMC 제어 파라미터
    const double LAMBDA = 10.0;
    const double K_GAIN = 4.0;
    const double PHI = 0.2;

    // 제어 상태 변수
    double cmd_joint[6];
    double last_vel[6];

    uint32_t cbox_id;
    uint32_t robot_id;

    std::mutex m_control_mutex;
    std::mutex m_control_rt_mutex;
    std::atomic<bool> m_is_rt_control_ready;
};

} // namespace rt_control