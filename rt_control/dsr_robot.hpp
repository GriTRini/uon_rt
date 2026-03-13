#pragma once

#include <array>
#include <iostream>
#include <mutex>
#include <string>
#include <atomic>
#include <memory>
#include <chrono>
#include <optional>
#include <Eigen/Dense>

#include "uon/uon_timer.hpp" 
#include "../drfl/BINARY_GL013201_20250213/include/DRFLEx.h"
#include "model/model.hpp"
#include "rt_data.hpp"
#include "sim_robot.hpp" // 수학 엔진 부모 클래스 포함

namespace rt_control {

namespace draf = DRAFramework;

template <size_t ID = 0> 
class DsrRobot : public SimRobot {
  public:
    using angles_t = rt_control::angles_t;
    using value_t = rt_control::value_t;
    constexpr static size_t id = ID;

  public:
    // 모델을 주입받아 생성
    DsrRobot(std::shared_ptr<model::RobotModel> model, const std::chrono::milliseconds update_dt = std::chrono::milliseconds(1))
        : SimRobot(model),
          m_update_timer(update_dt, [this]() { rt_loop(); }),
          m_is_tp_initialized(false), 
          m_has_control_right(false),
          m_is_rt_control_ready(false), 
          m_servoj_target_time(0.001) 
    {
        m_robot_instance = this;
    }

    // ==========================================================
    // 1. 공통 인터페이스: 연결 및 제어권
    // ==========================================================
    [[nodiscard]] OpenConnError open_connection(
        const std::string &strIpAddr = "192.168.137.100",
        const uint32_t usPort = 12345,
        const std::chrono::milliseconds timeout_init_tp = std::chrono::milliseconds(100),
        const std::chrono::milliseconds timeout_get_ctrl = std::chrono::milliseconds(500)) 
    {
        std::lock_guard<std::mutex> lock(m_control_mutex);
        if (m_control) m_control.reset();

        m_control = std::unique_ptr<draf::LPROBOTCONTROL, decltype(&close_and_destroy_ctrl)>(
            static_cast<draf::LPROBOTCONTROL *>(draf::_CreateRobotControl()), close_and_destroy_ctrl);

        if (!m_control) return OpenConnError::CREATE_ROBOT_CONTROL_ERROR;

        draf::_set_on_monitoring_access_control(m_control.get(), TOnMonitoringAccessControlCB);
        draf::_set_on_tp_initializing_completed(m_control.get(), TOnTpInitializingCompletedCB);
        draf::_set_on_disconnected(m_control.get(), TOnDisconnectedCB);

        if (!draf::_open_connection(m_control.get(), strIpAddr.c_str(), usPort)) return OpenConnError::OPEN_CONNECTION_ERROR;
        if (!wait_for([this]() { return m_is_tp_initialized.load(); }, timeout_init_tp)) return OpenConnError::INITIALIZE_TP_ERROR;
        
        draf::_manage_access_control(m_control.get(), MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
        if (!wait_for([this]() { return m_has_control_right.load(); }, timeout_get_ctrl)) return OpenConnError::GET_CONTROL_RIGHT_ERROR;

        return OpenConnError::NO_ERROR;
    }

    [[nodiscard]] bool connect_rt(const std::string &strIpAddr = "192.168.137.100", const uint32_t usPort = 12347) {
        std::lock_guard<std::mutex> lock(m_control_rt_mutex);
        if (m_control_rt) m_control_rt.reset();

        m_control_rt = std::unique_ptr<draf::LPROBOTCONTROL, decltype(&close_and_destroy_ctrl_udp)>(
            static_cast<draf::LPROBOTCONTROL *>(draf::_create_robot_control_udp()), close_and_destroy_ctrl_udp);

        if (!draf::_connect_rt_control(m_control_rt.get(), strIpAddr.c_str(), usPort)) return false;
        draf::_set_rt_control_output(m_control_rt.get(), "v1.0", 0.001, 4);
        if (!draf::_start_rt_control(m_control_rt.get())) return false;

        m_is_rt_control_ready.store(true);
        return true;
    }

    [[nodiscard]] ServoOnError servo_on(const std::chrono::milliseconds timeout = std::chrono::milliseconds(3000)) {
        if (!m_control) return ServoOnError::NO_ROBOT_CONTROL_ERROR;
        draf::_set_robot_mode(m_control.get(), ROBOT_MODE_AUTONOMOUS);
        draf::_set_robot_control(m_control.get(), CONTROL_RESET_SAFET_OFF);

        if (!wait_for([this]() { return draf::_get_robot_state(m_control.get()) == STATE_STANDBY; }, timeout)) 
            return ServoOnError::GET_STATE_STANDBY_ERROR;

        sync_state(); // 🌟 전원 인가 시 하드웨어 위치로 엔진 동기화
        m_update_timer.start(); 
        return ServoOnError::NO_ERROR;
    }

    // ==========================================================
    // 2. 공통 인터페이스: 툴(TCP) 설정
    // ==========================================================
    [[nodiscard]] bool set_tool(const std::string &symbol, const Eigen::Matrix4d &tcp_offset = Eigen::Matrix4d::Identity()) noexcept {
        if (!m_control) return false;
        bool res = draf::_set_tool(m_control.get(), symbol.c_str());
        if (res) {
            this->set_tcp_tmat(tcp_offset); // 🌟 수학 엔진(SimRobot) 좌표계 자동 업데이트
        }
        return res;
    }

    // ==========================================================
    // 3. 공통 인터페이스: 궤적 제어 (Override)
    // ==========================================================
    bool trapj(const angles_t &goal, const angles_t &goal_v = angles_t::Zero(), 
               const angles_t &pk_v = angles_t::Constant(120), const angles_t &pk_a = angles_t::Constant(1200)) override {
        if (!m_update_timer.is_running()) return false;
        sync_state(); // 🌟 실행 직전 실제 각도 동기화
        return SimRobot::trapj(goal, goal_v, pk_v, pk_a);
    }

    bool attrj(const angles_t &goal, const angles_t &kp) override {
        if (!m_update_timer.is_running()) return false;
        sync_state(); 
        return SimRobot::attrj(goal, kp);
    }

    bool attrl(const Eigen::Matrix4d &goal_tmat, value_t kp = 50.0) override {
        if (!m_update_timer.is_running()) return false;
        sync_state(); // 🌟 TCP 위치를 파악한 뒤 시작
        return SimRobot::attrl(goal_tmat, kp);
    }

    // 🌟 추가된 playj 인터페이스 (Eigen::MatrixXd 사용)
    bool playj(const Eigen::MatrixXd &q_set, 
               const std::optional<Eigen::MatrixXd> &v_set = std::nullopt,
               const std::optional<Eigen::MatrixXd> &a_set = std::nullopt) override {
        if (!m_update_timer.is_running()) return false;
        sync_state(); // 🌟 연속 궤적 시작 전에도 현재 위치와 속도 동기화!
        return SimRobot::playj(q_set, v_set, a_set);
    }

    // ==========================================================
    // 4. 공통 인터페이스: 텔레메트리 (정보 읽기)
    // ==========================================================
    [[nodiscard]] std::optional<angles_t> get_current_angles() const noexcept {
        if (!m_is_rt_control_ready.load()) return std::nullopt;
        auto data = draf::_read_data_rt(m_control_rt.get());
        angles_t res; for(int i=0; i<6; i++) res(i) = data->actual_joint_position[i];
        return res;
    }

    [[nodiscard]] std::optional<angles_t> get_current_angvels() const noexcept {
        if (!m_is_rt_control_ready.load()) return std::nullopt;
        auto data = draf::_read_data_rt(m_control_rt.get());
        angles_t res; for(int i=0; i<6; i++) res(i) = data->actual_joint_velocity[i];
        return res;
    }

    [[nodiscard]] std::optional<Eigen::Matrix4d> get_current_tmat() const noexcept {
        auto q = get_current_angles();
        if (!q) return std::nullopt;
        return (m_model->forward_kinematics(q.value()) * m_tcp_tmat).matrix(); // 🌟 TCP가 반영된 진짜 위치
    }

    // ==========================================================
    // 5. 내부 실시간 제어 루프
    // ==========================================================
  protected:
    void rt_loop() {
        if (!m_is_rt_control_ready.load()) return;

        // 1. 수학 계산 (SimRobot 로직 1ms 전진)
        this->update_sim(0.001);

        // 2. 하드웨어 전송
        angles_t q = this->get_angles();
        float fq[6], fdq[6]={0,}, fddq[6]={0,};
        for (int i = 0; i < 6; i++) fq[i] = (float)q(i);
        draf::_servoj_rt(m_control_rt.get(), fq, fdq, fddq, m_servoj_target_time);
    }

    void sync_state() {
        if (!m_is_rt_control_ready.load()) return;
        auto data = draf::_read_data_rt(m_control_rt.get());
        angles_t q_hw;
        for(int i=0; i<6; i++) q_hw(i) = data->actual_joint_position[i];
        this->set_current_state(q_hw, angles_t::Zero()); // 🌟 엔진 시작점을 하드웨어로 강제 보정
    }

  private:
    static void TOnMonitoringAccessControlCB(const MONITORING_ACCESS_CONTROL e) {
        if(e == MONITORING_ACCESS_CONTROL_GRANT) m_robot_instance->m_has_control_right.store(true);
        else if(e == MONITORING_ACCESS_CONTROL_LOSS) m_robot_instance->m_has_control_right.store(false);
    }
    static void TOnTpInitializingCompletedCB() { m_robot_instance->m_is_tp_initialized.store(true); }
    static void TOnDisconnectedCB() { m_robot_instance->m_has_control_right.store(false); }
    static void close_and_destroy_ctrl(draf::LPROBOTCONTROL p) { if(p){ draf::_close_connection(p); draf::_DestroyRobotControl(p); } }
    static void close_and_destroy_ctrl_udp(draf::LPROBOTCONTROL p) { if(p){ draf::_disconnect_rt_control(p); draf::_destroy_robot_control_udp(p); } }

    static bool wait_for(std::function<bool()> f, std::chrono::milliseconds t) {
        auto start = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start < t) { if(f()) return true; std::this_thread::sleep_for(std::chrono::milliseconds(10)); }
        return false;
    }

  private:
    static DsrRobot<ID> *m_robot_instance;
    std::unique_ptr<draf::LPROBOTCONTROL, decltype(&close_and_destroy_ctrl)> m_control{nullptr, close_and_destroy_ctrl};
    std::unique_ptr<draf::LPROBOTCONTROL, decltype(&close_and_destroy_ctrl_udp)> m_control_rt{nullptr, close_and_destroy_ctrl_udp};
    std::mutex m_control_mutex, m_control_rt_mutex;
    std::atomic<bool> m_is_tp_initialized, m_has_control_right, m_is_rt_control_ready;
    uon::timer::Timer<int64_t, std::milli> m_update_timer;
    float m_servoj_target_time;
};

template <size_t ID> DsrRobot<ID> *DsrRobot<ID>::m_robot_instance = nullptr;

} // namespace rt_control