#pragma once

#include <mutex>
#include <thread>
#include <iostream>

#include "../core/robot_base.hpp"
#include "dsr_data.hpp"
#include "dsr_enum.hpp"
#include "../../drfl/include/DRFLEx.h" // 경로 확인 요망

namespace rt_control {

namespace draf = DRAFramework;

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
        m_dsr_instance = this; // 콜백 함수용 인스턴스 등록
    }

    ~DsrRobot() override {
        disconnect_rt();
        (void)close_connection();
        if (m_dsr_instance == this) m_dsr_instance = nullptr;
    }

    // ==============================================================
    // 🔌 DSR Override 로직
    // ==============================================================
    bool open_connection(const std::string &strIpAddr = "192.168.1.30", const uint32_t usPort = 12345) override {
        std::lock_guard<std::mutex> lock(m_control_mutex);
        
        if (m_control) { 
            close_and_destroy_ctrl(m_control); 
            m_control = nullptr;
        }

        m_control = draf::_CreateRobotControl();
        if (!m_control) return false;

        m_is_tp_initialized.store(false);
        m_has_control_right.store(false);

        draf::_set_on_monitoring_access_control(m_control, TOnMonitoringAccessControlCB);
        draf::_set_on_tp_initializing_completed(m_control, TOnTpInitializingCompletedCB);
        draf::_set_on_disconnected(m_control, TOnDisconnectedCB);

        if (!draf::_open_connection(m_control, strIpAddr.c_str(), usPort)) {
            close_and_destroy_ctrl(m_control);
            return false;
        }

        if (!wait_for([this]() { return m_is_tp_initialized.load(); }, std::chrono::milliseconds(500))) return false;
        if (!draf::_manage_access_control(m_control, MANAGE_ACCESS_CONTROL_FORCE_REQUEST)) return false;
        if (!wait_for([this]() { return m_has_control_right.load(); }, std::chrono::milliseconds(500))) return false;

        return true;
    }

    bool connect_rt(const std::string &strIpAddr = "192.168.1.30", const uint32_t usPort = 12347) override {
        std::lock_guard<std::mutex> lock(m_control_rt_mutex);
        if (m_control_rt) { 
            close_and_destroy_ctrl_udp(m_control_rt); 
            m_control_rt = nullptr;
        }

        m_control_rt = draf::_create_robot_control_udp();
        if (!m_control_rt) return false;

        if (!draf::_connect_rt_control(m_control_rt, strIpAddr.c_str(), usPort)) return false;
        if (!draf::_set_rt_control_output(m_control_rt, "v1.0", 0.001, 4)) return false;
        if (!draf::_start_rt_control(m_control_rt)) return false;

        m_is_rt_control_ready.store(true);
        return true;
    }

    bool servo_on() override {
        std::lock_guard<std::mutex> lock(m_control_mutex);
        if (!m_control) return false;

        draf::_set_robot_mode(m_control, ROBOT_MODE_AUTONOMOUS);
        draf::_set_robot_control(m_control, CONTROL_RESET_SAFET_OFF);

        if (!wait_for([this]() { return draf::_get_robot_state(m_control) == STATE_STANDBY; }, std::chrono::milliseconds(3000))) {
            return false;
        }

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

    void disconnect_rt() override {
        std::lock_guard<std::mutex> lock(m_control_rt_mutex);
        if (m_control_rt) {
            draf::_stop_rt_control(m_control_rt);
            close_and_destroy_ctrl_udp(m_control_rt);
            m_control_rt = nullptr;
            m_is_rt_control_ready.store(false);
        }
    }

    bool close_connection() override {
        std::lock_guard<std::mutex> lock(m_control_mutex);
        if (m_control) {
            close_and_destroy_ctrl(m_control);
            m_control = nullptr;
        }
        return true;
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

    void set_digital_output(int index, bool value) override {
        if (m_control) {
            draf::_set_digital_output(m_control, static_cast<GPIO_CTRLBOX_DIGITAL_INDEX>(index-1), value);
        }
    }

protected:
    // ==============================================================
    // 🌟 DSR 실시간 제어 루프
    // ==============================================================
    void update() override {
        if (!m_is_rt_control_ready.load()) return;

        const double dt = std::chrono::duration_cast<std::chrono::milliseconds>(m_update_timer.interval).count() / 1000.0;

        // 부모의 기구학 및 궤적 업데이트
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
    static void TOnMonitoringAccessControlCB(const MONITORING_ACCESS_CONTROL eAccCtrl) {
        if (!m_dsr_instance) return;
        if (eAccCtrl == MONITORING_ACCESS_CONTROL_GRANT) m_dsr_instance->m_has_control_right.store(true);
        else if (eAccCtrl == MONITORING_ACCESS_CONTROL_LOSS) m_dsr_instance->m_has_control_right.store(false);
    }
    static void TOnTpInitializingCompletedCB() { 
        if (m_dsr_instance) m_dsr_instance->m_is_tp_initialized.store(true); 
    }
    static void TOnDisconnectedCB() { 
        if (m_dsr_instance) m_dsr_instance->m_has_control_right.store(false); 
    }

    static void close_and_destroy_ctrl(draf::LPROBOTCONTROL pCtrl) {
        if (!pCtrl) return;
        draf::_close_connection(pCtrl); 
        draf::_DestroyRobotControl(pCtrl); 
    }

    static void close_and_destroy_ctrl_udp(draf::LPROBOTCONTROL pCtrlUDP) {
        if (!pCtrlUDP) return;
        draf::_disconnect_rt_control(pCtrlUDP); 
        draf::_destroy_robot_control_udp(pCtrlUDP); 
    }

    static bool wait_for(std::function<bool()> func, const std::chrono::milliseconds timeout) {
        const auto end_time = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < end_time) {
            if (func()) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return false;
    }

private:
    static DsrRobot* m_dsr_instance; // 정적 콜백을 위한 포인터

    draf::LPROBOTCONTROL m_control = nullptr;
    std::mutex m_control_mutex;
    draf::LPROBOTCONTROL m_control_rt = nullptr;
    std::mutex m_control_rt_mutex;

    std::atomic<bool> m_is_tp_initialized{};
    std::atomic<bool> m_has_control_right{};
    std::atomic<bool> m_is_rt_control_ready{};
    float m_servoj_target_time = 0.033f; 
};

// 정적 멤버 초기화
DsrRobot* DsrRobot::m_dsr_instance = nullptr;

} // namespace rt_control