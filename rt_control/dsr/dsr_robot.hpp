#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <condition_variable>
#include <format>
#include <functional>
#include <iostream>
#include <cmath>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <optional>

#include "../timer.hpp"
#include "../../drfl/include/DRFLEx.h"

#include "dsr_data.hpp"
#include "dsr_enum.hpp"
#include "../model/model.hpp"
#include "../trajectory/trajectory_generator.hpp"

namespace dsr_control {

namespace draf = DRAFramework;

template <size_t ID = 0> class Robot {
  public:
    using traj_gen_t = rt_control::trajectory::TrajGenerator;
    using angles_t = rt_control::angles_t;
    using value_t = rt_control::value_t;
    using tmat_t = Eigen::Isometry3d;
    
    using jmat_t = Eigen::Matrix<double, 6, 6>;
    using a_t = Eigen::Matrix<value_t, 6, 1>;

    enum class ActiveMotion { NONE, TRAPJ, ATTRL };

  public:
    constexpr static size_t id = ID;

  public:
    Robot(const std::string& model_name = "m1013",
          const std::chrono::milliseconds update_dt = std::chrono::milliseconds(1))
        : m_model(model_name),
          m_update_timer(std::chrono::milliseconds(update_dt), [this]() { update(); }),
          m_is_tp_initialized(false), 
          m_has_control_right(false),
          m_is_rt_control_ready(false), 
          m_servoj_target_time(0.01f),
          m_is_paused(false),
          m_active_motion(ActiveMotion::NONE)
    {
        m_traj_gen.initialize(m_model, angles_t::Zero(), angles_t::Zero(), angles_t::Zero());
        m_robot_instance = this;
    }

    ~Robot() {
        disconnect_rt();
        (void)close_connection();
    }

    // ==============================================================
    // 🌟 데이터 수신 및 상태 확인 유틸리티
    // ==============================================================
    [[nodiscard]] std::optional<angles_t> get_current_angles() const noexcept {
        if (!m_control_rt) return std::nullopt;
        const float* pData = draf::_read_data_rt(m_control_rt)->actual_joint_position;
        return Eigen::Map<const Eigen::Matrix<float, 6, 1>>(pData).cast<double>();
    }

    [[nodiscard]] std::optional<angles_t> get_current_angvels() const noexcept {
        if (!m_control_rt) return std::nullopt;
        const float* pData = draf::_read_data_rt(m_control_rt)->actual_joint_velocity;
        return Eigen::Map<const Eigen::Matrix<float, 6, 1>>(pData).cast<double>();
    }

    [[nodiscard]] bool is_paused() const noexcept {
        return m_is_paused.load();
    }

    void set_digital_output(GPIO_CTRLBOX_DIGITAL_INDEX index, bool value) {
        if (m_control) draf::_set_digital_output(m_control, index, value);
    }

  public:
    // ==============================================================
    // 🔌 Connection & Control Setup
    // ==============================================================

    [[nodiscard]] OpenConnError open_connection(const std::string &strIpAddr = "192.168.1.30", uint32_t usPort = 12345, std::chrono::milliseconds timeout_init_tp = std::chrono::milliseconds(100), std::chrono::milliseconds timeout_get_ctrl = std::chrono::milliseconds(500)) {
        std::lock_guard<std::mutex> lock(m_control_mutex);
        if (m_control) { close_and_destroy_ctrl(m_control); m_control = nullptr; }
        m_control = draf::_CreateRobotControl();
        if (!m_control) return OpenConnError::CREATE_ROBOT_CONTROL_ERROR;
        m_is_tp_initialized.store(false); m_has_control_right.store(false);
        draf::_set_on_monitoring_access_control(m_control, TOnMonitoringAccessControlCB);
        draf::_set_on_tp_initializing_completed(m_control, TOnTpInitializingCompletedCB);
        draf::_set_on_disconnected(m_control, TOnDisconnectedCB);
        if (!draf::_open_connection(m_control, strIpAddr.c_str(), usPort)) { close_and_destroy_ctrl(m_control); return OpenConnError::OPEN_CONNECTION_ERROR; }
        if (!wait_for([this]() { return m_is_tp_initialized.load(); }, timeout_init_tp)) return OpenConnError::INITIALIZE_TP_ERROR;
        if (!draf::_manage_access_control(m_control, MANAGE_ACCESS_CONTROL_FORCE_REQUEST)) return OpenConnError::MANAGE_ACCESS_CONTROL_ERROR;
        if (!wait_for([this]() { return m_has_control_right.load(); }, timeout_get_ctrl)) return OpenConnError::GET_CONTROL_RIGHT_ERROR;
        return OpenConnError::NO_ERROR;
    }

    [[nodiscard]] bool connect_rt(const std::string &strIpAddr = "192.168.1.30", uint32_t usPort = 12347) {
        std::lock_guard<std::mutex> lock(m_control_rt_mutex);
        if (m_control_rt) { close_and_destroy_ctrl_udp(m_control_rt); m_control_rt = nullptr; }
        m_control_rt = draf::_create_robot_control_udp();
        if (!m_control_rt) return false;
        if (!draf::_connect_rt_control(m_control_rt, strIpAddr.c_str(), usPort)) return false;
        if (!draf::_set_rt_control_output(m_control_rt, "v1.0", 0.001, 4)) return false;
        if (!draf::_start_rt_control(m_control_rt)) return false;
        m_is_rt_control_ready.store(true);
        return true;
    }

    [[nodiscard]] ServoOnError servo_on(const std::chrono::milliseconds timeout = std::chrono::milliseconds(3000)) {
        std::lock_guard<std::mutex> lock(m_control_mutex);
        if (!m_control) return ServoOnError::NO_ROBOT_CONTROL_ERROR;
        draf::_set_robot_mode(m_control, ROBOT_MODE_AUTONOMOUS);
        draf::_set_robot_control(m_control, CONTROL_RESET_SAFET_OFF);
        if (!wait_for([this]() { return draf::_get_robot_state(m_control) == STATE_STANDBY; }, timeout)) return ServoOnError::GET_STATE_STANDBY_ERROR;
        if (m_is_rt_control_ready.load()) draf::_set_safety_mode(m_control, SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);
        const auto start_angles = get_current_angles();
        const auto start_angvels = get_current_angvels();
        if (start_angles.has_value() && start_angvels.has_value()) m_traj_gen.initialize(m_model, start_angles.value(), start_angvels.value(), angles_t::Zero());
        m_is_paused.store(false); m_update_timer.start();
        return ServoOnError::NO_ERROR;
    }

    [[nodiscard]] ServoOffError servo_off() {
        m_update_timer.stop(); std::lock_guard<std::mutex> lock(m_control_mutex);
        if (m_control && draf::_servo_off(m_control, STOP_TYPE_QUICK)) return ServoOffError::NO_ERROR;
        return ServoOffError::SET_SERVO_OFF_ERROR;
    }

    void disconnect_rt() {
        std::lock_guard<std::mutex> lock(m_control_rt_mutex);
        if (m_control_rt) { draf::_stop_rt_control(m_control_rt); close_and_destroy_ctrl_udp(m_control_rt); m_control_rt = nullptr; m_is_rt_control_ready.store(false); }
    }

    [[nodiscard]] CloseConnError close_connection() {
        std::lock_guard<std::mutex> lock(m_control_mutex);
        if (m_control) { close_and_destroy_ctrl(m_control); m_control = nullptr; }
        return CloseConnError::NO_ERROR;
    }

    // ==============================================================
    // 🌟 Motion Commands & TCP Info (이름 변경됨!)
    // ==============================================================
    void set_tcp(value_t x, value_t y, value_t z, value_t r_deg, value_t p_deg, value_t yaw_deg) noexcept { m_traj_gen.set_tcp(x, y, z, r_deg, p_deg, yaw_deg); }
    
    // 1. 제어기(Generator)가 계산한 현재 명령 위치(Commanded Pose)
    [[nodiscard]] const tmat_t& get_goal_endpoint() const noexcept { return m_traj_gen.tmat(); }
    
    // 2. 실제 로봇의 센서 값(Encoder)을 기반으로 Forward Kinematics를 거친 실제 위치(Actual Pose)
    [[nodiscard]] std::optional<tmat_t> get_current_endpoint() const noexcept {
        auto cur_q = get_current_angles();
        if (cur_q) {
            return m_traj_gen.solve_forward(*cur_q);
        }
        return std::nullopt;
    }

    [[nodiscard]] const jmat_t& get_jacobian() const noexcept { return m_traj_gen.jmat(); }
    [[nodiscard]] const a_t& get_task_vel() const noexcept { return m_traj_gen.a(); }
    [[nodiscard]] tmat_t solve_forward(const angles_t& q) const noexcept { return m_traj_gen.solve_forward(q); }
    
    void stop() noexcept { m_traj_gen.stop(); }

    [[nodiscard]] bool get_goal_reached(const std::optional<value_t> &q_th = std::nullopt, const std::optional<value_t> &p_th = std::nullopt, const std::optional<value_t> &r_th = std::nullopt) const noexcept { 
        return m_traj_gen.goal_reached(q_th, p_th, r_th, std::nullopt, std::nullopt, std::nullopt); 
    }

    [[nodiscard]] bool trapj(const angles_t &goal_angles, const std::optional<angles_t> &goal_angvels = std::nullopt) noexcept {
        if (!m_update_timer.is_running()) return false;
        m_active_motion.store(ActiveMotion::TRAPJ); m_saved_goal_angles = goal_angles; m_is_paused.store(false);
        return m_traj_gen.trapj(goal_angles, goal_angvels.value_or(angles_t::Zero()));
    }

    [[nodiscard]] bool attrl(const tmat_t &goal_tmat, value_t kp = 50.0) noexcept {
        if (!m_update_timer.is_running()) return false;
        m_active_motion.store(ActiveMotion::ATTRL); m_saved_goal_tmat = goal_tmat; m_saved_kp = kp; m_is_paused.store(false);
        return m_traj_gen.attrl(goal_tmat, kp);
    }

    // 이름 변경된 get_goal_endpoint()를 반영
    [[nodiscard]] bool movel(double x, double y, double z, value_t kp = 50.0) noexcept {
        if (!m_update_timer.is_running()) return false;
        tmat_t target = get_goal_endpoint();
        target.translation() << x, y, z;
        return attrl(target, kp);
    }

    [[nodiscard]] bool align_tcp_to_floor(double yaw_deg = 0.0, value_t kp = 100.0) noexcept { 
        if (!m_update_timer.is_running()) return false; 
        return m_traj_gen.align_tcp_to_floor(yaw_deg, kp); 
    }
    
    [[nodiscard]] bool align_tcp_to_front(value_t kp = 100.0) noexcept {
        if (!m_update_timer.is_running()) return false;
        return m_traj_gen.align_tcp_to_front(kp);
    }

    // ==============================================================
    // 🌟 에러 복구 로직 (사용자가 1번/2번 선택 시 자동 Servo-On)
    // ==============================================================
    
    [[nodiscard]] bool resume_trajectory() {
        if (!m_is_paused.load()) return true;

        int state = draf::_get_robot_state(m_control);
        
        if (state == 6) {
            std::cerr << "\n[Resume 실패] 🔴 비상 정지 버튼이 눌려있습니다. 펜던트의 버튼을 돌려서 빼주세요!" << std::endl;
            return false;
        }
        
        if (state != 1) {
            std::cout << "   ▶ 모터 오프(충돌/안전정지) 감지. 자동으로 Servo-On을 실행하여 복구합니다..." << std::endl;
            {
                std::lock_guard<std::mutex> lock(m_control_mutex);
                draf::_set_robot_mode(m_control, ROBOT_MODE_AUTONOMOUS);
                draf::_set_robot_control(m_control, CONTROL_RESET_SAFET_OFF);
            }
            
            if (!wait_for([this]() { return draf::_get_robot_state(m_control) == 1; }, std::chrono::milliseconds(4000))) {
                std::cerr << "   ❌ Servo-On 복구 실패. 펜던트 화면에서 다른 알람이 있는지 확인하세요." << std::endl;
                return false;
            }
            std::cout << "   ✅ Servo-On 완료! 상태 복구 성공." << std::endl;
        }

        draf::_set_safety_mode(m_control, SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);
        
        bool res = false;
        ActiveMotion mode = m_active_motion.load();
        if (mode == ActiveMotion::TRAPJ) {
            res = m_traj_gen.trapj(m_saved_goal_angles, angles_t::Zero());
        } else if (mode == ActiveMotion::ATTRL) {
            res = m_traj_gen.attrl(m_saved_goal_tmat, m_saved_kp);
        }
        
        m_is_paused.store(false); 
        return res;
    }

    [[nodiscard]] bool cancel_trajectory() {
        if (!m_is_paused.load()) return true;

        int state = draf::_get_robot_state(m_control);
        
        if (state == 6) {
            std::cerr << "\n[Cancel 실패] 🔴 비상 정지 버튼이 눌려있습니다. 펜던트의 버튼을 돌려서 빼주세요!" << std::endl;
            return false;
        }
        
        if (state != 1) {
            std::cout << "   ▶ 모터 오프(충돌/안전정지) 감지. 자동으로 Servo-On을 실행하여 복구합니다..." << std::endl;
            {
                std::lock_guard<std::mutex> lock(m_control_mutex);
                draf::_set_robot_mode(m_control, ROBOT_MODE_AUTONOMOUS);
                draf::_set_robot_control(m_control, CONTROL_RESET_SAFET_OFF);
            }
            
            if (!wait_for([this]() { return draf::_get_robot_state(m_control) == 1; }, std::chrono::milliseconds(4000))) {
                std::cerr << "   ❌ Servo-On 복구 실패. 펜던트 화면에서 다른 알람이 있는지 확인하세요." << std::endl;
                return false;
            }
            std::cout << "   ✅ Servo-On 완료! 상태 복구 성공." << std::endl;
        }

        draf::_set_safety_mode(m_control, SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);
        m_active_motion.store(ActiveMotion::NONE); 
        m_is_paused.store(false); 
        return true;
    }

    // ==============================================================
    // 🌟 실시간 업데이트 루프 (Internal Timer)
    // ==============================================================
  protected:
    void update() {
        if (!m_is_rt_control_ready.load()) return;
        
        const auto* rt_data = draf::_read_data_rt(m_control_rt);
        if (!rt_data) return;

        int state = draf::_get_robot_state(m_control);

        bool is_physical_stop = (state != 0 && state != 1 && state != 2);

        if (is_physical_stop && !m_is_paused.load()) {
            m_is_paused.store(true);
            
            const float* act_q = rt_data->actual_joint_position;
            const float* act_dq = rt_data->actual_joint_velocity;
            
            angles_t sync_q = Eigen::Map<const Eigen::Matrix<float, 6, 1>>(act_q).cast<double>();
            angles_t sync_dq = Eigen::Map<const Eigen::Matrix<float, 6, 1>>(act_dq).cast<double>();
            
            m_traj_gen.initialize(m_model, sync_q, sync_dq, angles_t::Zero());
            std::cout << "\n🚨 [안전 회로 가동] 로봇 서보 오프 감지(상태코드: " << state << "). 충돌 위치로 동기화 완료." << std::endl;
        }

        if (m_is_paused.load()) {
            float q[6], q_d1[6] = {0}, q_d2[6] = {0};
            for (int i = 0; i < 6; i++) q[i] = static_cast<float>(m_traj_gen.angles()(i));
            draf::_servoj_rt(m_control_rt, q, q_d1, q_d2, m_servoj_target_time);
            return; 
        }

        const double dt = m_update_timer.interval.count() / 1000.0;
        m_traj_gen.update(dt);
        
        float q[6], q_d1[6], q_d2[6];
        for (int i = 0; i < 6; i++) {
            q[i] = static_cast<float>(m_traj_gen.angles()(i));
            q_d1[i] = static_cast<float>(m_traj_gen.angvels()(i));
            q_d2[i] = static_cast<float>(m_traj_gen.angaccs()(i));
        }
        draf::_servoj_rt(m_control_rt, q, q_d1, q_d2, m_servoj_target_time);
    }

  private:
    static void TOnMonitoringAccessControlCB(const MONITORING_ACCESS_CONTROL eAccCtrl) { if (eAccCtrl == MONITORING_ACCESS_CONTROL_GRANT) m_robot_instance->m_has_control_right.store(true); else if (eAccCtrl == MONITORING_ACCESS_CONTROL_LOSS) m_robot_instance->m_has_control_right.store(false); }
    static void TOnTpInitializingCompletedCB() { m_robot_instance->m_is_tp_initialized.store(true); }
    static void TOnDisconnectedCB() { m_robot_instance->m_has_control_right.store(false); }
    static void close_and_destroy_ctrl(draf::LPROBOTCONTROL pCtrl) { if (pCtrl) { draf::_close_connection(pCtrl); draf::_DestroyRobotControl(pCtrl); } }
    static void close_and_destroy_ctrl_udp(draf::LPROBOTCONTROL pCtrlUDP) { if (pCtrlUDP) { draf::_disconnect_rt_control(pCtrlUDP); draf::_destroy_robot_control_udp(pCtrlUDP); } }
    [[nodiscard]] static bool wait_for(std::function<bool()> func, const std::chrono::milliseconds timeout) { const auto end_time = std::chrono::steady_clock::now() + timeout; while (std::chrono::steady_clock::now() < end_time) { if (func()) return true; std::this_thread::sleep_for(std::chrono::milliseconds(10)); } return false; }

  private:
    static Robot<ID> *m_robot_instance;
    rt_control::model::RobotModel m_model; 
    traj_gen_t m_traj_gen;
    draf::LPROBOTCONTROL m_control = nullptr;
    std::mutex m_control_mutex;
    draf::LPROBOTCONTROL m_control_rt = nullptr;
    std::mutex m_control_rt_mutex;
    std::atomic<bool> m_is_tp_initialized{};
    std::atomic<bool> m_has_control_right{};
    std::atomic<bool> m_is_rt_control_ready{};
    uon::timer::Timer<int64_t, std::milli> m_update_timer;
    float m_servoj_target_time = 0.01f; 
    std::atomic<bool> m_is_paused{};
    std::atomic<ActiveMotion> m_active_motion{};
    angles_t m_saved_goal_angles;
    tmat_t m_saved_goal_tmat;
    value_t m_saved_kp = 50.0;
};

template <size_t ID> Robot<ID> *Robot<ID>::m_robot_instance = nullptr;

} // namespace dsr_control