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

// 한화 로봇의 CLINK API 헤더
#include "../../hcr/include/clink_api_rpc.h"
#include "../../hcr/include/clink_api_rpc_system.h"

// CMake 등에서 HANWHA_ROOT_PATH가 정의되지 않은 경우를 대비한 폴백(Fallback)
#ifndef HANWHA_ROOT_PATH
#define HANWHA_ROOT_PATH "."
#endif

namespace rt_control {

/**
 * @brief 한화로봇 실시간 제어 클래스 
 * RobotBase의 규격을 준수하며 Hanwha CLINK API와 통신을 담당합니다.
 */
class HcrRobot : public RobotBase {
public:
    HcrRobot(const std::string& model_name, 
             const std::chrono::milliseconds update_dt = std::chrono::milliseconds(1))
        : RobotBase(model_name, update_dt),
          CLINK_CONFIG_FILE(std::string(HANWHA_ROOT_PATH) + "/config/config_rpc.ini"),
          m_is_connected(false),
          m_is_rt_control_ready(false),
          cbox_id(0),
          robot_id(0) 
    {
        cmd_joint.fill(0.0);
        last_vel.fill(0.0);
    }

    ~HcrRobot() override {
        disconnect_rt();
        close_connection();
    }

    // ==============================================================
    // 🔌 연결 및 초기화 로직
    // ==============================================================

    bool open_connection(const std::string &strIpAddr = "", const uint32_t usPort = 0) override {
        std::lock_guard<std::mutex> lock(m_control_mutex);
        
        std::string ip = strIpAddr.empty() ? "192.168.100.200" : strIpAddr; // 기본 IP
        CLINK_API_RESULT err_ret_val = CLINK_API_RESULT_OK;
        
        // 1. 컨트롤 박스 연결 (설정 파일 사용)
        err_ret_val = clink_rpc_system_cbox_connect(CLINK_CONFIG_FILE.c_str(), ip.c_str(), &cbox_id);
        if (err_ret_val != CLINK_API_RESULT_OK) {
            std::cerr << "[Error] [HcrRobot] 컨트롤 박스 연결 실패: " << err_ret_val << std::endl;
            return false;
        }

        // 2. 제어SW 초기화
        err_ret_val = clink_rpc_gen_system_create(cbox_id, "", CBOX_MODEL_NAME);
        if (err_ret_val != CLINK_API_RESULT_OK && err_ret_val < CLINK_API_RESULT_WARNING_BEGIN) {
            std::cerr << "[Error] [HcrRobot] 제어SW 초기화 실패: " << err_ret_val << std::endl;
            return false;
        }

        // 3. 제어권 획득
        err_ret_val = clink_rpc_system_control_take(cbox_id);
        if (err_ret_val != CLINK_API_RESULT_OK) return false;

        // 4. 로봇 생성
        err_ret_val = clink_rpc_robot_create(cbox_id, ROBOT_MODEL_NAME, "", 0U, &robot_id);
        if (err_ret_val != CLINK_API_RESULT_OK) return false;

        // 5. EtherCAT 상태 확인 및 대기
        CLINK_ECAT_CONN_STATE ecat_stat = CLINK_ECAT_CONN_STATE_DISCONNECTED;
        clink_rpc_cbox_ecat_connection_state_get(cbox_id, &ecat_stat);
        if (CLINK_ECAT_CONN_STATE_CONNECTED != ecat_stat) {
            char_t valid_event = -1;
            clink_rpc_system_wait_event_group_subgroup(
                cbox_id, CLINK_EVENT_GRP_NOTIFICATION,
                CLINK_EVENT_SUBGRP_NOTIFICATION_ECAT_CONNECTED,
                1000000, 1, &valid_event);
        }

        // 6. 자동 속도 조절 기능 ON
        clink_rpc_robot_motion_auto_adjust_swith_set(cbox_id, robot_id, CLINK_SWITCH_ON);

        m_is_connected.store(true);
        std::cout << "[Info] [HcrRobot] 로봇 연결 및 초기화 완료: " << ip << std::endl;
        
        return true;
    }

    bool close_connection() override {
        std::lock_guard<std::mutex> lock(m_control_mutex);
        if (!m_is_connected.load()) return false;

        // 안전 상 로봇 stop 및 servo off 실행 후 종료
        clink_rpc_robot_stop(cbox_id, robot_id, 0.5);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        clink_rpc_robot_servo_switch_set(cbox_id, robot_id, CLINK_SWITCH_OFF);
        clink_rpc_system_control_release(cbox_id);

        m_is_connected.store(false);
        std::cout << "[Info] [HcrRobot] 로봇 연결 해제" << std::endl;
        return true;
    }

    bool connect_rt(const std::string &strIpAddr = "", const uint32_t usPort = 0) override {
        std::lock_guard<std::mutex> lock(m_control_rt_mutex);
        if (!m_is_connected.load()) return false;

        // Joint Chaser는 서보 온 이후에 켜기 위해 플래그만 활성화
        m_is_rt_control_ready.store(true);
        std::cout << "[Info] [HcrRobot] 실시간 제어 준비 완료 (서보 온 시 켜짐)" << std::endl;
        return true;
    }

    void disconnect_rt() override {
        std::lock_guard<std::mutex> lock(m_control_rt_mutex);
        if (m_is_rt_control_ready.load()) {
            clink_rpc_robot_joint_chaser_switch_set(cbox_id, robot_id, CLINK_SWITCH_OFF);
            m_is_rt_control_ready.store(false);
            std::cout << "[Info] [HcrRobot] 실시간 제어(Joint Chaser) 비활성화" << std::endl;
        }
    }

    // ==============================================================
    // ⚙️ 도구(Tool) 및 하드웨어 설정 (미지원 기능 예외처리)
    // ==============================================================

    bool add_tool(const std::string &name, float weight, 
                  std::array<float, 3> cog, 
                  std::array<float, 6> inertia) noexcept override { return false; }
    bool set_tool(const std::string &name) noexcept override { return false; }
    bool del_tool(const std::string &name) noexcept override { return false; }
    bool change_collision_sensitivity(const float fSensitivity) noexcept override { return false; }

    // ==============================================================
    // ⚙️ 구동 제어 및 데이터 획득
    // ==============================================================

    bool servo_on() override {
        std::lock_guard<std::mutex> lock(m_control_mutex);
        if (!m_is_connected.load()) return false;

        CLINK_SWITCH result;
        clink_rpc_robot_servo_switch_get(cbox_id, robot_id, &result);
        
        // 1. 서보 온 (브레이크 해제)
        if (result != CLINK_SWITCH_ON) {
            if (clink_rpc_robot_servo_switch_set(cbox_id, robot_id, CLINK_SWITCH_ON) != CLINK_API_RESULT_OK) {
                std::cerr << "[Error] [HcrRobot] 서보 온 실패" << std::endl;
                return false;
            }
            std::cout << "[Info] [HcrRobot] 서보 온 (물리적 브레이크 해제 대기 1초...)" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }

        // 2. 현재 위치 획득
        auto start_angles_opt = this->get_current_angles();
        auto start_angvels_opt = this->get_current_angvels();
        
        if (!start_angles_opt || !start_angvels_opt) {
            std::cerr << "[Error] [HcrRobot] 현재 위치 획득 실패" << std::endl;
            return false;
        }

        // 3. 컨트롤러 내부 상태 동기화 (제자리 movej 및 대기)
        std::cout << "[Info] [HcrRobot] 제어기 상태 동기화 진행 중..." << std::endl;
        sync_controller_state(start_angles_opt.value());

        // 4. 동기화 완료 후 Joint Chaser(실시간 모드) 활성화
        if (m_is_rt_control_ready.load()) {
            clink_rpc_robot_joint_chaser_switch_set(cbox_id, robot_id, CLINK_SWITCH_ON);
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 모드 전환 대기
            std::cout << "[Info] [HcrRobot] Joint Chaser 모드 ON" << std::endl;
        }

        // 5. Trajectory Generator 및 제어 변수 초기화
        m_traj_gen.initialize(m_model, start_angles_opt.value(), start_angvels_opt.value(), angles_t::Zero());
        for(int i = 0; i < 6; i++) {
            cmd_joint[i] = start_angles_opt.value()(i);
            last_vel[i] = start_angvels_opt.value()(i);
        }

        // 6. 실시간 업데이트 타이머 시작
        m_update_timer.start();
        std::cout << "[Info] [HcrRobot] 구동 초기화 완료 및 실시간 루프 시작" << std::endl;
        return true;
    }

    bool servo_off() override {
        m_update_timer.stop();
        std::lock_guard<std::mutex> lock(m_control_mutex);
        if (!m_is_connected.load()) return false;

        // 서보 오프 전 Joint Chaser 안전 종료
        if (m_is_rt_control_ready.load()) {
            clink_rpc_robot_joint_chaser_switch_set(cbox_id, robot_id, CLINK_SWITCH_OFF);
        }

        if (clink_rpc_robot_servo_switch_set(cbox_id, robot_id, CLINK_SWITCH_OFF) == CLINK_API_RESULT_OK) {
            std::cout << "[Info] [HcrRobot] 서보 오프" << std::endl;
            return true;
        }
        return false;
    }

    std::optional<angles_t> get_current_angles() const noexcept override {
        if (!m_is_connected.load()) return std::nullopt;
        
        clink_float_t angle[6];
        for(int i = 0; i < 6; i++) {
            clink_rpc_robot_joint_angle_actual_get(cbox_id, robot_id, i, &angle[i]);
        }
        return Eigen::Map<const Eigen::Matrix<double, 6, 1>>(angle);
    }

    std::optional<angles_t> get_current_angvels() const noexcept override {
        if (!m_is_connected.load()) return std::nullopt;

        clink_float_t vel[6];
        for(int i = 0; i < 6; i++) {
            clink_rpc_robot_joint_speed_actual_get(cbox_id, robot_id, i, &vel[i]);
        }
        return Eigen::Map<const Eigen::Matrix<double, 6, 1>>(vel);
    }

    std::optional<angles_t> get_current_torque() const noexcept { return std::nullopt; }
    void set_digital_output(int index, bool value) override { }
    bool get_digital_input(int index) override { return false; }

    // ==============================================================
    // 🎯 물리적 로봇 안착(Settling) 확인 함수
    // ==============================================================
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

        const auto& actual_q = actual_q_opt.value();
        const auto& actual_dq = actual_dq_opt.value();
        const auto& target_q = m_traj_gen.angles();

        if (q_th.has_value() && (actual_q - target_q).norm() > q_th.value()) return false; 
        if (v_th.has_value() && actual_dq.norm() > v_th.value()) return false; 

        return true;
    }

    // ==============================================================
    // 🚨 알람(Error) 데이터 획득
    // ==============================================================
    std::optional<RobotAlarm> pop_alarm() noexcept override {
        std::lock_guard<std::mutex> lock(m_alarm_mutex);
        if (m_alarm_queue.empty()) return std::nullopt;
        
        RobotAlarm alarm = m_alarm_queue.front();
        m_alarm_queue.pop();
        return alarm;
    }

protected:
    // ==============================================================
    // 🌟 실시간 루프 (Update)
    // ==============================================================
    void update() override {
        if (!m_is_rt_control_ready.load()) return;

        const double dt = std::chrono::duration_cast<std::chrono::milliseconds>(m_update_timer.interval).count() / 1000.0;
        if (dt <= 1e-6) return;
        
        m_traj_gen.update(dt);
        const angles_t& target_q = m_traj_gen.angles();
        
        clink_float_t next_target[6];

        // SMC (Sliding Mode Control)
        for (int i = 0; i < 6; i++) {
            double error = target_q(i) - cmd_joint[i];

            // s = de/dt + lambda * e
            double s = LAMBDA * error - last_vel[i];

            // 제어 입력
            double reach_vel = K_GAIN * std::tanh(s / PHI);
            double current_vel = reach_vel + (LAMBDA * error);

            // 업데이트
            cmd_joint[i] += current_vel * dt;
            last_vel[i] = current_vel;

            next_target[i] = static_cast<clink_float_t>(cmd_joint[i]);
        }
        // 조인트 체이서 명령 전송
        clink_rpc_robot_joint_chaser_move(cbox_id, robot_id, 6, next_target);
    }

private:
    std::mutex m_control_mutex;
    std::mutex m_control_rt_mutex;
    
    std::atomic<bool> m_is_connected;
    std::atomic<bool> m_is_rt_control_ready;

    uint32_t cbox_id;
    uint32_t robot_id;

    std::queue<RobotAlarm> m_alarm_queue;
    std::mutex m_alarm_mutex;

    // ==============================================================
    // 사용자 정의 설정값 및 파라미터 
    // ==============================================================
    const CLINK_CBOX_MODEL CBOX_MODEL_NAME = CLINK_CBOX_MODEL_3GEN;       // 펜던트 버전
    const CLINK_ROBOT_MODEL ROBOT_MODEL_NAME = CLINK_ROBOT_MODEL_HCR14;   // 로봇 종류
    const std::string CLINK_CONFIG_FILE;                                  // 한화 로봇 설정 파일

    // SMC 파라미터
    const double LAMBDA = 10.0; // 슬라이딩 평면 기울기 (오차 수렴 속도)
    const double K_GAIN = 4.0;  // 제어 강도 (외란 억제력)
    const double PHI = 0.2;     // 진동 방지

    std::array<double, 6> cmd_joint;
    std::array<double, 6> last_vel;

    // ==============================================================
    // 🌟 제어기 동기화 유틸리티 함수 (기존 movej 기반)
    // ==============================================================
    bool sync_controller_state(const angles_t& q) {
        clink_rpc_system_event_queue_clear(cbox_id);

        uint32_t cmd_id = 0;
        clink_rpc_motion_command_robot_joint_create(cbox_id, &cmd_id);
        clink_rpc_motion_command_robot_robot_id_set(cbox_id, cmd_id, robot_id);

        for (uint32_t j_idx = 0; j_idx < 6; j_idx++) {
            clink_rpc_motion_command_robot_joint_angle_end_set(
                cbox_id, cmd_id, j_idx, static_cast<clink_float_t>(q(j_idx)));
        }

        // 제자리 이동용이므로 속도/가속도 파라미터는 기본 최소/안전값 적용
        clink_rpc_motion_command_velocity_profiler_set(cbox_id, cmd_id, CLINK_VELOCITY_PROFILE_ASYMMETRIC_S_CURVE);
        clink_rpc_motion_command_acc_max_set(cbox_id, cmd_id, 10.0f);
        clink_rpc_motion_command_dec_max_set(cbox_id, cmd_id, 10.0f);
        clink_rpc_motion_command_speed_max_set(cbox_id, cmd_id, 10.0f);
        clink_rpc_motion_command_jerk_percentage_set(cbox_id, cmd_id, 0.5f);
        clink_rpc_motion_command_blending_motion_mode_set(cbox_id, cmd_id, CLINK_SWITCH_OFF);

        clink_rpc_motion_command_queue_add(cbox_id, cmd_id);
        clink_rpc_motion_command_queue_flush(cbox_id);
        clink_rpc_motion_command_destroy(cbox_id, cmd_id);

        // 안착 대기 (기존 wait_event_group_subgroup 재사용)
        char_t valid_event = -1;
        clink_rpc_system_wait_event_group_subgroup(
            cbox_id,
            CLINK_EVENT_GRP_MOTION_COMMAND,
            CLINK_EVENT_SUBGRP_MOTION_COMMAND_SETTLED_DOWN,
            5000, 
            1, 
            &valid_event);

        std::cout << "[Info] [HcrRobot] 모션 동기화 완료 (SETTLED_DOWN)" << std::endl;
        return true;
    }
};

} // namespace rt_control