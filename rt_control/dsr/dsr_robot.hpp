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

#include "../core/robot_base.hpp"
#include "dsr_data.hpp"
#include "dsr_enum.hpp"
#include "../../drfl/include/DRFLEx.h" 

namespace rt_control {

namespace draf = DRAFramework;

/**
 * @brief 두산로봇 실시간 제어 클래스 
 * RobotBase의 규격을 준수하며 DRFL SDK와 통신을 담당합니다.
 */
class DsrRobot : public RobotBase {
public:
    DsrRobot(const std::string& model_name, 
             const std::chrono::milliseconds update_dt = std::chrono::milliseconds(1))
        : RobotBase(model_name, update_dt),
          m_is_tp_initialized(false), 
          m_has_control_right(false),
          m_is_rt_control_ready(false), 
          m_servoj_target_time(0.033f) 
    {
        m_dsr_instance = this; 
    }

    ~DsrRobot() override {
        disconnect_rt();
        close_connection();
        if (m_dsr_instance == this) m_dsr_instance = nullptr;
    }

    // ==============================================================
    // 🔌 연결 및 초기화 로직
    // ==============================================================

    bool open_connection(const std::string &strIpAddr = "", const uint32_t usPort = 0) override {
        std::lock_guard<std::mutex> lock(m_control_mutex);
        
        // IP와 포트가 비어있으면 두산 로봇의 기본값 사용
        std::string ip = strIpAddr.empty() ? "192.168.1.30" : strIpAddr;
        uint32_t port = (usPort == 0) ? 12345 : usPort;

        if (m_control) { 
            close_and_destroy_ctrl(m_control); 
            m_control = nullptr;
        }

        m_control = draf::_CreateRobotControl();
        if (!m_control) return false;

        m_is_tp_initialized.store(false);
        m_has_control_right.store(false);

        // 콜백 등록
        draf::_set_on_monitoring_speed_mode(m_control, TOnMonitoringSpeedModeCB);
        draf::_set_on_monitoring_access_control(m_control, TOnMonitoringAccessControlCB);
        draf::_set_on_tp_initializing_completed(m_control, TOnTpInitializingCompletedCB);
        draf::_set_on_disconnected(m_control, TOnDisconnectedCB);
        draf::_set_on_log_alarm(m_control, TOnLogAlarmCB);

        if (!draf::_open_connection(m_control, ip.c_str(), port)) {
            close_and_destroy_ctrl(m_control);
            m_control = nullptr;
            return false;
        }

        if (!wait_for([this]() { return m_is_tp_initialized.load(); }, std::chrono::milliseconds(2000))) return false;
        if (!draf::_manage_access_control(m_control, MANAGE_ACCESS_CONTROL_FORCE_REQUEST)) return false;
        if (!wait_for([this]() { return m_has_control_right.load(); }, std::chrono::milliseconds(1000))) return false;

        return true;
    }

    bool close_connection() override {
        std::lock_guard<std::mutex> lock(m_control_mutex);
        if (m_control) {
            close_and_destroy_ctrl(m_control);
            m_control = nullptr;
            return true;
        }
        return false;
    }

    bool connect_rt(const std::string &strIpAddr = "", const uint32_t usPort = 0) override {
        std::lock_guard<std::mutex> lock(m_control_rt_mutex);
        
        std::string ip = strIpAddr.empty() ? "192.168.1.30" : strIpAddr;
        uint32_t port = (usPort == 0) ? 12347 : usPort;

        if (m_control_rt) { 
            close_and_destroy_ctrl_udp(m_control_rt); 
            m_control_rt = nullptr;
        }

        m_control_rt = draf::_create_robot_control_udp();
        if (!m_control_rt) return false;

        if (!draf::_connect_rt_control(m_control_rt, ip.c_str(), port)) return false;
        if (!draf::_set_rt_control_output(m_control_rt, "v1.0", 0.001, 4)) return false;
        if (!draf::_start_rt_control(m_control_rt)) return false;

        m_is_rt_control_ready.store(true);
        return true;
    }

    void disconnect_rt() override {
        std::lock_guard<std::mutex> lock(m_control_rt_mutex);
        if (m_control_rt) {
            draf::_stop_rt_control(m_control_rt);
            close_and_destroy_ctrl_udp(m_control_rt);
            m_control_rt = nullptr;
            m_is_rt_control_ready.store(false);
        }
    }

    // ==============================================================
    // ⚙️ 도구(Tool) 및 하드웨어 설정
    // ==============================================================

    bool add_tool(const std::string &name, float weight, 
                  std::array<float, 3> cog, 
                  std::array<float, 6> inertia) noexcept override {
        if (!m_control) return false;
        return draf::_add_tool(m_control, name.c_str(), weight, cog.data(), inertia.data());
    }

    bool set_tool(const std::string &name) noexcept override {
        if (!m_control) return false;
        return draf::_set_tool(m_control, name.c_str());
    }

    bool del_tool(const std::string &name) noexcept override {
        if (!m_control) return false;
        return draf::_del_tool(m_control, name.c_str());
    }

    bool change_collision_sensitivity(const float fSensitivity) noexcept override {
        if (!m_control) return false;
        return draf::_change_collision_sensitivity(m_control, fSensitivity);
    }

    // ==============================================================
    // ⚙️ 구동 제어 및 데이터 획득
    // ==============================================================

    bool servo_on() override {
        std::lock_guard<std::mutex> lock(m_control_mutex);
        if (!m_control) return false;

        draf::_set_robot_mode(m_control, ROBOT_MODE_AUTONOMOUS);
        draf::_set_robot_control(m_control, CONTROL_RESET_SAFET_OFF);

        if (!wait_for([this]() { return draf::_get_robot_state(m_control) == STATE_STANDBY; }, std::chrono::milliseconds(3000))) return false;

        if (m_is_rt_control_ready.load()) {
            draf::_set_safety_mode(m_control, SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);
        }

        const auto start_angles = this->get_current_angles();
        const auto start_angvels = this->get_current_angvels();
        if (start_angles.has_value() && start_angvels.has_value()) {
            m_traj_gen.initialize(m_model, start_angles.value(), start_angvels.value(), angles_t::Zero());
        }

        m_update_timer.start();
        return true;
    }

    bool servo_off() override {
        m_update_timer.stop();
        std::lock_guard<std::mutex> lock(m_control_mutex);
        if (m_control && draf::_servo_off(m_control, STOP_TYPE_QUICK)) return true;
        return false;
    }

    std::optional<angles_t> get_current_angles() const noexcept override {
        if (!m_control_rt) return std::nullopt;
        const float* pData = draf::_read_data_rt(m_control_rt)->actual_joint_position;
        return Eigen::Map<const Eigen::Matrix<float, 6, 1>>(pData).cast<double>();
    }

    std::optional<angles_t> get_current_angvels() const noexcept override {
        if (!m_control_rt) return std::nullopt;
        const float* pData = draf::_read_data_rt(m_control_rt)->actual_joint_velocity;
        return Eigen::Map<const Eigen::Matrix<float, 6, 1>>(pData).cast<double>();
    }

    std::optional<angles_t> get_current_torque() const noexcept {
        if (!m_control_rt) return std::nullopt;
        const float* pData = draf::_read_data_rt(m_control_rt)->actual_joint_torque;
        return Eigen::Map<const Eigen::Matrix<float, 6, 1>>(pData).cast<double>();
    }

    void set_digital_output(int index, bool value) override {
        if (!m_control) return;
        draf::_set_digital_output(m_control, static_cast<GPIO_CTRLBOX_DIGITAL_INDEX>(index), value);
    }

    // ==============================================================
    // 🚨 알람(Error) 데이터 획득
    // ==============================================================

    std::optional<RobotAlarm> pop_alarm() noexcept override {
        std::lock_guard<std::mutex> lock(m_alarm_mutex);
        if (m_alarm_queue.empty()) {
            return std::nullopt;
        }
        
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
        m_traj_gen.update(dt);

        float q[6], q_d1[6], q_d2[6];
        const angles_t& des_q = m_traj_gen.angles();
        const angles_t& des_dq = m_traj_gen.angvels();
        const angles_t& des_ddq = m_traj_gen.angaccs();

        for (int i = 0; i < 6; i++) {
            q[i] = static_cast<float>(des_q(i));
            q_d1[i] = static_cast<float>(des_dq(i));
            q_d2[i] = static_cast<float>(des_ddq(i));
        }
        
        draf::_servoj_rt(m_control_rt, q, q_d1, q_d2, m_servoj_target_time);
    }

private:
    // ==============================================================
    // 📩 콜백 함수들
    // ==============================================================

    static void TOnMonitoringSpeedModeCB(const MONITORING_SPEED eSpdMode) {
        if (eSpdMode == SPEED_REDUCED_MODE) {
            std::cerr << "[DSR] SPEED MODE: REDUCED!!!" << std::endl;
        }
    }

    static void TOnMonitoringAccessControlCB(const MONITORING_ACCESS_CONTROL eAccCtrl) {
        if (!m_dsr_instance) return;
        switch (eAccCtrl) {
            case MONITORING_ACCESS_CONTROL_REQUEST:
                draf::_manage_access_control(m_dsr_instance->m_control, MANAGE_ACCESS_CONTROL_RESPONSE_NO);
                break;
            case MONITORING_ACCESS_CONTROL_GRANT:
                m_dsr_instance->m_has_control_right.store(true);
                break;
            case MONITORING_ACCESS_CONTROL_LOSS:
                m_dsr_instance->m_has_control_right.store(false);
                break;
            default: break;
        }
    }

    static void TOnLogAlarmCB(const LPLOG_ALARM pLogAlarm) {
        if (!m_dsr_instance || !pLogAlarm) return;

        RobotAlarm alarm;
        alarm.time  = std::chrono::system_clock::now();
        alarm.level = pLogAlarm->_iLevel;
        alarm.group = pLogAlarm->_iGroup;
        alarm.index = pLogAlarm->_iIndex;
        
        for (int i = 0; i < 3; ++i) {
            if (pLogAlarm->_szParam[i] != nullptr) {
                alarm.param[i] = pLogAlarm->_szParam[i];
            } else {
                alarm.param[i] = ""; 
            }
        }

        {
            std::lock_guard<std::mutex> lock(m_dsr_instance->m_alarm_mutex);
            constexpr size_t MAX_ALARM_SIZE = 50; 
            while (m_dsr_instance->m_alarm_queue.size() >= MAX_ALARM_SIZE) {
                m_dsr_instance->m_alarm_queue.pop();
            }
            m_dsr_instance->m_alarm_queue.push(alarm);
        }

        std::cerr << "[DSR ALARM] Index: " << alarm.index 
                  << " | Level: " << alarm.level 
                  << " | Msg: " << alarm.param[0] << std::endl;
    }

    static void TOnTpInitializingCompletedCB() { 
        if (m_dsr_instance) m_dsr_instance->m_is_tp_initialized.store(true); 
    }

    static void TOnDisconnectedCB() { 
        if (m_dsr_instance) {
            m_dsr_instance->m_has_control_right.store(false); 
            m_dsr_instance->m_is_rt_control_ready.store(false);
        }
    }

    // ==============================================================
    // 🛠 내부 유틸리티
    // ==============================================================

    static void busy_wait(std::chrono::milliseconds ms) {
        const auto end_time = std::chrono::steady_clock::now() + ms;
        while (std::chrono::steady_clock::now() < end_time) {}
    }

    static void close_and_destroy_ctrl(draf::LPROBOTCONTROL pCtrl) {
        if (!pCtrl) return;
        busy_wait(std::chrono::milliseconds(10));
        draf::_close_connection(pCtrl); 
        busy_wait(std::chrono::milliseconds(10));
        draf::_DestroyRobotControl(pCtrl); 
    }

    static void close_and_destroy_ctrl_udp(draf::LPROBOTCONTROL pCtrlUDP) {
        if (!pCtrlUDP) return;
        busy_wait(std::chrono::milliseconds(10));
        draf::_disconnect_rt_control(pCtrlUDP); 
        busy_wait(std::chrono::milliseconds(10));
        draf::_destroy_robot_control_udp(pCtrlUDP); 
    }

    static bool wait_for(std::function<bool()> func, const std::chrono::milliseconds timeout) {
        const auto end_time = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < end_time) {
            if (func()) return true;
        }
        return false;
    }

private:
    static DsrRobot* m_dsr_instance;

    draf::LPROBOTCONTROL m_control = nullptr;
    std::mutex m_control_mutex;
    draf::LPROBOTCONTROL m_control_rt = nullptr;
    std::mutex m_control_rt_mutex;

    std::atomic<bool> m_is_tp_initialized{};
    std::atomic<bool> m_has_control_right{};
    std::atomic<bool> m_is_rt_control_ready{};

    std::queue<RobotAlarm> m_alarm_queue;
    std::mutex m_alarm_mutex;

    float m_servoj_target_time = 0.033f; 
};

DsrRobot* DsrRobot::m_dsr_instance = nullptr;

} // namespace rt_control