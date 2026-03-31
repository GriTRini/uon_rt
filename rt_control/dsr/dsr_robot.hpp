#pragma once

#include <array>
#include <condition_variable>
#include <format>
#include <functional>
#include <iostream>
#include <math.h>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <atomic>
#include <optional>

#include "uon/uon_timer.hpp"

#include "../drfl/BINARY_GL013201_20250213/include/DRFLEx.h"

#include "dsr_data.hpp"
#include "dsr_enum.hpp"
#include "trajectory/dsr_trajectory.hpp"

namespace dsr {

constexpr size_t MAX_LOG_QUEUE_SIZE = 10;

namespace draf = DRAFramework;

template <size_t ID = 0> class Robot {
  public:
    using traj_gen_t = trajectory::TrajGenerator;
    using angles_t = dsr::angles_t;
    using value_t = dsr::value_t;
    using tmat_t = dsr::tmat_t;
    using jmat_t = dsr::jmat_t;
    using a_t = dsr::a_t;

    enum class ActiveMotion { NONE, TRAPJ, ATTRL };

  public:
    constexpr static size_t id = ID;

  public:
    Robot(const std::chrono::milliseconds update_dt = std::chrono::milliseconds(1))
        : m_traj_gen(traj_gen_t{}),
          m_update_timer(std::chrono::milliseconds(update_dt), [this]() { update(); }),
          m_is_tp_initialized(false), 
          m_has_control_right(false),
          m_is_rt_control_ready(false), 
          m_servoj_target_time(0.01), // 실시간 주기에 맞춰 0.01로 조정
          m_is_paused(false),
          m_active_motion(ActiveMotion::NONE) 
    {
        m_robot_instance = this;
    }

    ~Robot() {
        disconnect_rt();
        (void)close_connection();
    }

  public:
    // ==============================================================
    // 🔌 Connection & Control Setup (스마트 포인터 적용됨)
    // ==============================================================
    [[nodiscard]] OpenConnError open_connection(const std::string &strIpAddr = "192.168.137.100", const uint32_t usPort = 12345, const std::chrono::milliseconds timeout_init_tp = std::chrono::milliseconds(100), const std::chrono::milliseconds timeout_get_ctrl = std::chrono::milliseconds(500)) {
        std::lock_guard<std::mutex> lock(m_control_mutex);
        if (m_control) m_control.reset();

        clear_queue(m_log_queue, m_log_queue_mutex);

        m_control = std::unique_ptr<draf::LPROBOTCONTROL, decltype(&close_and_destroy_ctrl)>(
            static_cast<draf::LPROBOTCONTROL *>(draf::_CreateRobotControl()), close_and_destroy_ctrl);

        if (!m_control) return OpenConnError::CREATE_ROBOT_CONTROL_ERROR;

        m_is_tp_initialized.store(false);
        m_has_control_right.store(false);

        draf::_set_on_monitoring_speed_mode(m_control.get(), TOnMonitoringSpeedModeCB);
        draf::_set_on_monitoring_access_control(m_control.get(), TOnMonitoringAccessControlCB);
        draf::_set_on_log_alarm(m_control.get(), TOnLogAlarmCB);
        draf::_set_on_tp_initializing_completed(m_control.get(), TOnTpInitializingCompletedCB);
        draf::_set_on_disconnected(m_control.get(), TOnDisconnectedCB);

        if (!draf::_open_connection(m_control.get(), strIpAddr.c_str(), usPort)) { m_control.reset(); return OpenConnError::OPEN_CONNECTION_ERROR; }
        if (!wait_for([this]() { return m_is_tp_initialized.load(); }, timeout_init_tp)) { m_control.reset(); return OpenConnError::INITIALIZE_TP_ERROR; }
        if (!draf::_manage_access_control(m_control.get(), MANAGE_ACCESS_CONTROL_FORCE_REQUEST)) { m_control.reset(); return OpenConnError::MANAGE_ACCESS_CONTROL_ERROR; }
        if (!wait_for([this]() { return m_has_control_right.load(); }, timeout_get_ctrl)) { m_control.reset(); return OpenConnError::GET_CONTROL_RIGHT_ERROR; }
        return OpenConnError::NO_ERROR;
    }

    [[nodiscard]] bool connect_rt(const std::string &strIpAddr = "192.168.137.100", const uint32_t usPort = 12347) {
        std::lock_guard<std::mutex> lock(m_control_rt_mutex);
        if (m_control_rt) m_control_rt.reset();

        m_control_rt = std::unique_ptr<draf::LPROBOTCONTROL, decltype(&close_and_destroy_ctrl_udp)>(
            static_cast<draf::LPROBOTCONTROL *>(draf::_create_robot_control_udp()), close_and_destroy_ctrl_udp);

        if (!m_control_rt) return false;

        m_is_rt_control_ready.store(false);
        if (!draf::_connect_rt_control(m_control_rt.get(), strIpAddr.c_str(), usPort)) { m_control_rt.reset(); return false; }
        if (!draf::_set_rt_control_output(m_control_rt.get(), "v1.0", 0.001, 4)) { m_control_rt.reset(); return false; }
        if (!draf::_start_rt_control(m_control_rt.get())) { m_control_rt.reset(); return false; }
        
        m_is_rt_control_ready.store(true);
        return true;
    }

    [[nodiscard]] ServoOnError servo_on(const std::chrono::milliseconds timeout = std::chrono::milliseconds(3000)) {
        std::lock_guard<std::mutex> lock(m_control_mutex);
        if (!m_control) return ServoOnError::NO_ROBOT_CONTROL_ERROR;

        if (!draf::_set_robot_mode(m_control.get(), ROBOT_MODE_AUTONOMOUS)) return ServoOnError::SET_ROBOT_MODE_ERROR;
        if (!draf::_set_robot_control(m_control.get(), CONTROL_RESET_SAFET_OFF)) return ServoOnError::SET_ROBOT_CONTROL_ERROR;

        if (!wait_for([this]() { return draf::_get_robot_state(m_control.get()) == STATE_STANDBY; }, timeout)) return ServoOnError::GET_STATE_STANDBY_ERROR;

        if (m_is_rt_control_ready.load()) {
            if (!draf::_set_safety_mode(m_control.get(), SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE)) return ServoOnError::SET_SAFETY_MODE_ERROR;
        }

        const auto start_angles = get_current_angles();
        const auto start_angvels = get_current_angvels();
        if (!start_angles.has_value() || !start_angvels.has_value()) return ServoOnError::GET_ROBOT_STATE_ERROR;

        // 🌟 수정됨: 초기화 시 0으로 세팅 (의존성 모델 구조 반영)
        m_traj_gen.initialize(start_angles.value(), start_angvels.value(), md::zeros_like(angles_t{}));

        m_is_paused.store(false);
        m_update_timer.start();
        return ServoOnError::NO_ERROR;
    }

    [[nodiscard]] ServoOffError servo_off(const std::chrono::milliseconds timeout = std::chrono::milliseconds(100)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        std::lock_guard<std::mutex> lock(m_control_mutex);
        if (!m_control) return ServoOffError::NO_ROBOT_CONTROL_ERROR;

        m_update_timer.stop();
        if (!draf::_servo_off(m_control.get(), STOP_TYPE_QUICK)) return ServoOffError::SET_SERVO_OFF_ERROR;
        if (!wait_for([this]() { return draf::_get_robot_state(m_control.get()) == STATE_SAFE_OFF; }, timeout)) return ServoOffError::GET_ROBOT_STATE_ERROR;
        
        return ServoOffError::NO_ERROR;
    }

    void disconnect_rt() {
        std::lock_guard<std::mutex> lock(m_control_rt_mutex);
        if (!m_control_rt) return;
        draf::_stop_rt_control(m_control_rt.get());
        m_control_rt.reset();
    }

    [[nodiscard]] CloseConnError close_connection(const std::chrono::milliseconds timeout = std::chrono::milliseconds(100)) {
        std::lock_guard<std::mutex> lock(m_control_mutex);
        if (!m_control) return CloseConnError::NO_ROBOT_CONTROL_ERROR;
        m_control.reset();
        if (!wait_for([this]() { return !m_has_control_right.load(); }, timeout)) return CloseConnError::LOSE_CONTROL_RIGHT_ERROR;
        return CloseConnError::NO_ERROR;
    }

    // ==============================================================
    // 🌟 TCP & Tool Settings
    // ==============================================================
    void set_tcp_tmat(const tmat_t &new_shift_tmat) noexcept { m_traj_gen.set_tcp_tmat(new_shift_tmat); }
    [[nodiscard]] tmat_t get_tcp_tmat() const noexcept { return traj_gen().get_tcp_tmat(); }

    void set_tcp(value_t x, value_t y, value_t z, value_t r_deg, value_t p_deg, value_t yaw_deg) noexcept {
        m_traj_gen.set_tcp(x, y, z, r_deg, p_deg, yaw_deg);
    }

    // ==============================================================
    // 🌟 Data Getters (이름 동기화 됨!)
    // ==============================================================
    [[nodiscard]] std::optional<ROBOT_STATE> get_robot_state() const noexcept {
        if (!m_control) return std::nullopt;
        return draf::_get_robot_state(m_control.get());
    }

    [[nodiscard]] std::optional<angles_t> get_current_angles() const noexcept {
        if (!m_control_rt) return std::nullopt;
        const auto pData = draf::_read_data_rt(m_robot_instance->m_control_rt.get())->actual_joint_position;
        return md::copy_like(angles_t{}, md::mdspan<const float, dsr::angles_extents_t>{pData});
    }

    [[nodiscard]] std::optional<angles_t> get_current_angvels() const noexcept {
        if (!m_control_rt) return std::nullopt;
        const auto pData = draf::_read_data_rt(m_robot_instance->m_control_rt.get())->actual_joint_velocity;
        return md::copy_like(angles_t{}, md::mdspan<const float, dsr::angles_extents_t>{pData});
    }

    // 🌟 실제(Actual) TMAT
    [[nodiscard]] std::optional<tmat_t> get_current_tmat() const noexcept {
        const auto angles = get_current_angles();
        if (!angles.has_value()) return std::nullopt;
        return solve_forward(angles.value());
    }

    // 🌟 명령(Commanded) TMAT
    [[nodiscard]] const auto &get_desired_tmat() const noexcept { return traj_gen().tmat(); }
    [[nodiscard]] const auto &get_desired_angles() const noexcept { return traj_gen().angles(); }
    [[nodiscard]] const auto &get_desired_angvels() const noexcept { return traj_gen().angvels(); }
    [[nodiscard]] const auto &get_desired_angaccs() const noexcept { return traj_gen().angaccs(); }

    [[nodiscard]] std::optional<tmat_t> get_goal_tmat() const noexcept { return traj_gen().goal_tmat(); }

    [[nodiscard]] tmat_t solve_forward(auto &&angles) const noexcept {
        return traj_gen().solve_forward(std::forward<decltype(angles)>(angles));
    }

    [[nodiscard]] bool is_paused() const noexcept { return m_is_paused.load(); }

    [[nodiscard]] bool get_goal_reached(
        const std::optional<value_t> &angles_enorm_thold = 2,
        const std::optional<value_t> &pos_enorm_thold = 0.002,
        const std::optional<value_t> &rot_enorm_thold = 3,
        const std::optional<value_t> &angvels_enorm_thold = 4,
        const std::optional<value_t> &vel_enorm_thold = 0.004,
        const std::optional<value_t> &w_enorm_thold = 6) const noexcept {
        return traj_gen().goal_reached(angles_enorm_thold, pos_enorm_thold, rot_enorm_thold, angvels_enorm_thold, vel_enorm_thold, w_enorm_thold);
    }

    // ==============================================================
    // 🌟 Motion Commands
    // ==============================================================
    void stop() noexcept { m_traj_gen.stop(); }

    [[nodiscard]] bool trapj(const angles_t &goal_angles, const angles_t &goal_angvels = md::zeros_like(angles_t{})) noexcept {
        if (!m_update_timer.is_running()) return false;
        m_active_motion.store(ActiveMotion::TRAPJ); 
        m_saved_goal_angles = goal_angles; 
        m_is_paused.store(false);
        // Note: 피크 속도/가속도 파라미터는 generator 기본값을 따르도록 함
        return m_traj_gen.trapj(goal_angles, goal_angvels);
    }

    [[nodiscard]] bool attrl(const tmat_t &goal_tmat, const value_t kp = 50.0) noexcept {
        if (!m_update_timer.is_running()) return false;
        m_active_motion.store(ActiveMotion::ATTRL); 
        m_saved_goal_tmat = goal_tmat; 
        m_saved_kp = kp; 
        m_is_paused.store(false);
        return m_traj_gen.attrl(goal_tmat, kp);
    }

    [[nodiscard]] bool align_tcp_to_floor(double yaw_deg = 0.0, value_t kp = 100.0) noexcept { 
        if (!m_update_timer.is_running()) return false; 
        return m_traj_gen.align_tcp_to_floor(yaw_deg, kp); 
    }

    // ==============================================================
    // 🌟 에러 복구 로직 (자동 Servo-On 지원)
    // ==============================================================
    [[nodiscard]] bool resume_trajectory() {
        if (!m_is_paused.load()) return true;

        auto state_opt = get_robot_state();
        if (!state_opt) return false;
        int state = static_cast<int>(state_opt.value());
        
        if (state == 6) {
            std::cerr << "\n[Resume 실패] 🔴 비상 정지 버튼이 눌려있습니다. 펜던트의 버튼을 돌려서 빼주세요!" << std::endl;
            return false;
        }
        
        if (state != 1) {
            std::cout << "   ▶ 모터 오프(충돌/안전정지) 감지. 자동으로 Servo-On을 실행하여 복구합니다..." << std::endl;
            {
                std::lock_guard<std::mutex> lock(m_control_mutex);
                draf::_set_robot_mode(m_control.get(), ROBOT_MODE_AUTONOMOUS);
                draf::_set_robot_control(m_control.get(), CONTROL_RESET_SAFET_OFF);
            }
            if (!wait_for([this]() { return draf::_get_robot_state(m_control.get()) == STATE_STANDBY; }, std::chrono::milliseconds(4000))) {
                std::cerr << "   ❌ Servo-On 복구 실패. 펜던트 화면을 확인하세요." << std::endl;
                return false;
            }
            std::cout << "   ✅ Servo-On 완료! 상태 복구 성공." << std::endl;
        }

        draf::_set_safety_mode(m_control.get(), SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);
        
        bool res = false;
        ActiveMotion mode = m_active_motion.load();
        if (mode == ActiveMotion::TRAPJ) res = m_traj_gen.trapj(m_saved_goal_angles, md::zeros_like(angles_t{}));
        else if (mode == ActiveMotion::ATTRL) res = m_traj_gen.attrl(m_saved_goal_tmat, m_saved_kp);
        
        m_is_paused.store(false); 
        return res;
    }

    [[nodiscard]] bool cancel_trajectory() {
        if (!m_is_paused.load()) return true;

        auto state_opt = get_robot_state();
        if (!state_opt) return false;
        int state = static_cast<int>(state_opt.value());
        
        if (state == 6) {
            std::cerr << "\n[Cancel 실패] 🔴 비상 정지 버튼이 눌려있습니다!" << std::endl;
            return false;
        }
        
        if (state != 1) {
            std::cout << "   ▶ 자동으로 Servo-On을 실행하여 복구합니다..." << std::endl;
            {
                std::lock_guard<std::mutex> lock(m_control_mutex);
                draf::_set_robot_mode(m_control.get(), ROBOT_MODE_AUTONOMOUS);
                draf::_set_robot_control(m_control.get(), CONTROL_RESET_SAFET_OFF);
            }
            if (!wait_for([this]() { return draf::_get_robot_state(m_control.get()) == STATE_STANDBY; }, std::chrono::milliseconds(4000))) {
                return false;
            }
            std::cout << "   ✅ Servo-On 완료! 상태 복구 성공." << std::endl;
        }

        draf::_set_safety_mode(m_control.get(), SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);
        m_active_motion.store(ActiveMotion::NONE); 
        m_is_paused.store(false); 
        return true;
    }

    // ==============================================================
    // 🌟 실시간 업데이트 루프
    // ==============================================================
  protected:
    void update() {
        if (!m_is_rt_control_ready.load()) return;

        const auto* rt_data = draf::_read_data_rt(m_control_rt.get());
        if (!rt_data) return;

        int state = draf::_get_robot_state(m_control.get());

        // 🌟 비상 정지 해제 감지 시 자동 서보 온
        static int prev_state = state;
        if (prev_state == 6 && state == 3) {
            std::thread([this]() {
                std::cout << "\n🟢 [자동 복구] 비상 정지 해제(6->3) 감지! 자동으로 Servo-On을 실행합니다..." << std::endl;
                std::lock_guard<std::mutex> lock(m_control_mutex);
                if (m_control) {
                    draf::_set_robot_mode(m_control.get(), ROBOT_MODE_AUTONOMOUS);
                    draf::_set_robot_control(m_control.get(), CONTROL_RESET_SAFET_OFF);
                }
            }).detach();
        }
        prev_state = state;

        bool is_physical_stop = (state != 0 && state != 1 && state != 2);

        // 🌟 멈춤 감지 시 가상 제어기를 실제 위치로 덮어쓰기
        if (is_physical_stop && !m_is_paused.load()) {
            m_is_paused.store(true);
            
            auto act_q_opt = get_current_angles();
            auto act_dq_opt = get_current_angvels();
            if (act_q_opt && act_dq_opt) {
                m_traj_gen.initialize(*act_q_opt, *act_dq_opt, md::zeros_like(angles_t{}));
            }
            std::cout << "\n🚨 [안전 회로 가동] 로봇 정지 감지(상태코드: " << state << "). 충돌 위치로 동기화 완료." << std::endl;
        }

        if (m_is_paused.load()) {
            // 정지 중일 땐 제어기의 시간을 멈추고 현재 위치만 전송
            float q[6], q_d1[6] = {0}, q_d2[6] = {0};
            for (int i = 0; i < 6; i++) q[i] = static_cast<float>(get_desired_angles()(i));
            draf::_servoj_rt(m_control_rt.get(), q, q_d1, q_d2, m_servoj_target_time);
            return; 
        }

        // --- 정상 동작 구간 ---
        const double dt = std::chrono::duration_cast<std::chrono::milliseconds>(m_update_timer.interval).count() / 1000.0;
        m_traj_gen.update(dt);

        float q[6], q_d1[6], q_d2[6];
        for (int i = 0; i < 6; i++) {
            q[i] = static_cast<float>(get_desired_angles()(i));
            q_d1[i] = static_cast<float>(get_desired_angvels()(i));
            q_d2[i] = static_cast<float>(get_desired_angaccs()(i));
        }
        draf::_servoj_rt(m_control_rt.get(), q, q_d1, q_d2, m_servoj_target_time);
    }

  private:
    static void TOnMonitoringSpeedModeCB(const MONITORING_SPEED eSpdMode) { if (eSpdMode == SPEED_REDUCED_MODE) std::cerr << "[DSR] SPEED MODE: REDUCED!!!" << std::endl; }
    static void TOnMonitoringAccessControlCB(const MONITORING_ACCESS_CONTROL eAccCtrl) { if (eAccCtrl == MONITORING_ACCESS_CONTROL_GRANT) m_robot_instance->m_has_control_right.store(true); else if (eAccCtrl == MONITORING_ACCESS_CONTROL_LOSS) m_robot_instance->m_has_control_right.store(false); }
    static void TOnLogAlarmCB(const LPLOG_ALARM pLogAlarm) { /* 큐 생략 */ }
    static void TOnTpInitializingCompletedCB() { m_robot_instance->m_is_tp_initialized.store(true); }
    static void TOnDisconnectedCB() { m_robot_instance->m_has_control_right.store(false); }

    static void close_and_destroy_ctrl(draf::LPROBOTCONTROL pCtrl) { std::this_thread::sleep_for(std::chrono::milliseconds(10)); draf::_close_connection(pCtrl); std::this_thread::sleep_for(std::chrono::milliseconds(10)); draf::_DestroyRobotControl(pCtrl); std::this_thread::sleep_for(std::chrono::milliseconds(10)); }
    static void close_and_destroy_ctrl_udp(draf::LPROBOTCONTROL pCtrlUDP) { std::this_thread::sleep_for(std::chrono::milliseconds(10)); draf::_disconnect_rt_control(pCtrlUDP); std::this_thread::sleep_for(std::chrono::milliseconds(10)); draf::_destroy_robot_control_udp(pCtrlUDP); std::this_thread::sleep_for(std::chrono::milliseconds(10)); }

    template <typename T> static void clear_queue(std::queue<T> &queue, std::mutex &mutex) { std::lock_guard<std::mutex> lock(mutex); while (!queue.empty()) queue.pop(); }

    [[nodiscard]] static bool wait_for(std::function<bool()> func, const std::chrono::milliseconds timeout, const std::chrono::milliseconds delay = std::chrono::milliseconds(10)) { const auto end_time = std::chrono::steady_clock::now() + timeout; while (std::chrono::steady_clock::now() < end_time) { if (func()) return true; std::this_thread::sleep_for(delay); } return false; }

  public:
    [[nodiscard]] const traj_gen_t &traj_gen() const noexcept { return m_traj_gen; }

  private:
    static Robot<ID> *m_robot_instance;
    traj_gen_t m_traj_gen;

    std::unique_ptr<draf::LPROBOTCONTROL, decltype(&close_and_destroy_ctrl)> m_control{nullptr, close_and_destroy_ctrl};
    std::mutex m_control_mutex;
    std::unique_ptr<draf::LPROBOTCONTROL, decltype(&close_and_destroy_ctrl_udp)> m_control_rt{nullptr, close_and_destroy_ctrl_udp};
    std::mutex m_control_rt_mutex;

    std::atomic<bool> m_is_tp_initialized{};
    std::atomic<bool> m_has_control_right{};
    std::atomic<bool> m_is_rt_control_ready{};

    uon::timer::Timer<int64_t, std::milli> m_update_timer;
    std::queue<LogAlarm> m_log_queue{};
    std::mutex m_log_queue_mutex;

    float m_servoj_target_time = 0.01; 

    // 복구 데이터 백업용
    std::atomic<bool> m_is_paused{};
    std::atomic<ActiveMotion> m_active_motion{};
    angles_t m_saved_goal_angles;
    tmat_t m_saved_goal_tmat;
    value_t m_saved_kp = 50.0;
};

template <size_t ID> Robot<ID> *Robot<ID>::m_robot_instance = nullptr;

} // namespace dsr