#pragma once

#include <algorithm>
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

#include "timer.hpp"
#include "../drfl/include/DRFLEx.h"

#include "dsr_data.hpp"
#include "dsr_enum.hpp"
#include "../model/model.hpp"
#include "../trajectory/trajectory_generator.hpp"

namespace dsr_control {

namespace draf = DRAFramework;

template <size_t ID = 0> class Robot {
  public:
    using traj_gen_t = trajectory::TrajGenerator;
    using angles_t = dsr_control::angles_t;
    using value_t = dsr_control::value_t;
    using tmat_t = Eigen::Isometry3d;
    
    using jmat_t = Eigen::Matrix<double, 6, 6>;
    using a_t = Eigen::Matrix<value_t, 6, 1>;
    // using angles_set_t = trajectory::TrajGenerator::angles_set_t;

  public:
    constexpr static size_t id = ID;

  public:
    Robot(const std::string& model_name = "m1013",
          const std::chrono::milliseconds update_dt = std::chrono::milliseconds(1))
        : m_model(model_name),
          m_traj_gen(),
          m_update_timer(std::chrono::milliseconds(update_dt), [this]() { update(); }),
          m_is_tp_initialized(false), 
          m_has_control_right(false),
          m_is_rt_control_ready(false), 
          m_servoj_target_time(0.01f)
    {
        // TrajGenerator 초기화 (Zero 상태로 시작)
        m_traj_gen.initialize(m_model, angles_t::Zero(), angles_t::Zero(), angles_t::Zero());
        m_robot_instance = this;
    }

    ~Robot() {
        disconnect_rt();
        (void)close_connection();
    }

  public:
    // ==============================================================
    // 🔌 Connection & Control Setup
    // ==============================================================

    [[nodiscard]] OpenConnError open_connection(
        const std::string &strIpAddr = "192.168.1.30",
        const uint32_t usPort = 12345,
        const std::chrono::milliseconds timeout_init_tp = std::chrono::milliseconds(100),
        const std::chrono::milliseconds timeout_get_ctrl = std::chrono::milliseconds(500)) 
    {
        std::lock_guard<std::mutex> lock(m_control_mutex);
        
        if (m_control) { 
            close_and_destroy_ctrl(m_control); 
            m_control = nullptr;
        }

        m_control = draf::_CreateRobotControl();
        if (!m_control) { return OpenConnError::CREATE_ROBOT_CONTROL_ERROR; }

        m_is_tp_initialized.store(false);
        m_has_control_right.store(false);

        // 콜백 등록
        draf::_set_on_monitoring_access_control(m_control, TOnMonitoringAccessControlCB);
        draf::_set_on_tp_initializing_completed(m_control, TOnTpInitializingCompletedCB);
        draf::_set_on_disconnected(m_control, TOnDisconnectedCB);

        if (!draf::_open_connection(m_control, strIpAddr.c_str(), usPort)) {
            close_and_destroy_ctrl(m_control);
            return OpenConnError::OPEN_CONNECTION_ERROR;
        }

        if (!wait_for([this]() { return m_is_tp_initialized.load(); }, timeout_init_tp)) {
            return OpenConnError::INITIALIZE_TP_ERROR;
        }

        if (!draf::_manage_access_control(m_control, MANAGE_ACCESS_CONTROL_FORCE_REQUEST)) {
            return OpenConnError::MANAGE_ACCESS_CONTROL_ERROR;
        }

        if (!wait_for([this]() { return m_has_control_right.load(); }, timeout_get_ctrl)) {
            return OpenConnError::GET_CONTROL_RIGHT_ERROR;
        }

        return OpenConnError::NO_ERROR;
    }

    [[nodiscard]] bool connect_rt(
        const std::string &strIpAddr = "192.168.1.30",
        const uint32_t usPort = 12347) 
    {
        std::lock_guard<std::mutex> lock(m_control_rt_mutex);
        if (m_control_rt) { 
            close_and_destroy_ctrl_udp(m_control_rt); 
            m_control_rt = nullptr;
        }

        m_control_rt = draf::_create_robot_control_udp();
        if (!m_control_rt) { return false; }

        if (!draf::_connect_rt_control(m_control_rt, strIpAddr.c_str(), usPort)) {
            return false;
        }

        // 출력 데이터 설정 (1ms 주기)
        if (!draf::_set_rt_control_output(m_control_rt, "v1.0", 0.001, 4)) {
            return false;
        }

        if (!draf::_start_rt_control(m_control_rt)) {
            return false;
        }

        m_is_rt_control_ready.store(true);
        return true;
    }

    [[nodiscard]] ServoOnError servo_on(
        const std::chrono::milliseconds timeout = std::chrono::milliseconds(3000)) 
    {
        std::lock_guard<std::mutex> lock(m_control_mutex);
        if (!m_control) { return ServoOnError::NO_ROBOT_CONTROL_ERROR; }

        draf::_set_robot_mode(m_control, ROBOT_MODE_AUTONOMOUS);
        draf::_set_robot_control(m_control, CONTROL_RESET_SAFET_OFF);

        if (!wait_for([this]() { return draf::_get_robot_state(m_control) == STATE_STANDBY; }, timeout)) {
            return ServoOnError::GET_STATE_STANDBY_ERROR;
        }

        // 실시간 제어 준비 완료 시 세이프티 모드 전환
        if (m_is_rt_control_ready.load()) {
            draf::_set_safety_mode(m_control, SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);
        }

        // 현재 로봇 각도로 TrajGenerator 동기화 후 시작
        const auto start_angles = this->get_current_angles();
        const auto start_angvels = this->get_current_angvels();
        if (start_angles.has_value() && start_angvels.has_value()) {
            m_traj_gen.initialize(m_model, start_angles.value(), start_angvels.value(), angles_t::Zero());
        }

        m_update_timer.start();
        return ServoOnError::NO_ERROR;
    }

    [[nodiscard]] ServoOffError servo_off() {
        m_update_timer.stop();
        std::lock_guard<std::mutex> lock(m_control_mutex);
        if (m_control && draf::_servo_off(m_control, STOP_TYPE_QUICK)) {
            return ServoOffError::NO_ERROR;
        }
        return ServoOffError::SET_SERVO_OFF_ERROR;
    }

    void disconnect_rt() {
        std::lock_guard<std::mutex> lock(m_control_rt_mutex);
        if (m_control_rt) {
            draf::_stop_rt_control(m_control_rt);
            close_and_destroy_ctrl_udp(m_control_rt);
            m_control_rt = nullptr;
            m_is_rt_control_ready.store(false);
        }
    }

    [[nodiscard]] CloseConnError close_connection() {
        std::lock_guard<std::mutex> lock(m_control_mutex);
        if (m_control) {
            close_and_destroy_ctrl(m_control);
            m_control = nullptr;
        }
        return CloseConnError::NO_ERROR;
    }

    [[nodiscard]] std::optional<ROBOT_STATE> get_robot_state() const noexcept {
        if (!m_control) {
            return std::nullopt;
        }
        return draf::_get_robot_state(m_control);
    }

    // ==============================================================
    // 🌟 [핵심] 실시간 기구학 및 TCP 정보 인터페이스
    // ==============================================================

    /** @brief TCP 오프셋 설정 (XYZ-RPY Degree) */
    void set_tcp(value_t x, value_t y, value_t z, value_t r_deg, value_t p_deg, value_t yaw_deg) noexcept {
        m_traj_gen.set_tcp(x, y, z, r_deg, p_deg, yaw_deg);
    }

    /** @brief 현재 도구 끝단(TCP)의 전역 포즈 반환 */
    [[nodiscard]] const tmat_t& get_current_pos() const noexcept { return m_traj_gen.tmat(); }

    /** @brief 현재 Jacobian 행렬 반환 */
    [[nodiscard]] const jmat_t& get_jacobian() const noexcept { return m_traj_gen.jmat(); }

    /** @brief 현재 작업 공간(Task) 속도 (v, w) 반환 */
    [[nodiscard]] const a_t& get_task_vel() const noexcept { return m_traj_gen.a(); }

    /** @brief 순기구학 계산 (TCP 반영) */
    [[nodiscard]] tmat_t solve_forward(const angles_t& q) const noexcept {
        return m_traj_gen.solve_forward(q);
    }

    /** @brief 역기구학 계산 (현재 위치 기준) - 구현되어 있다면 활성화 */
    // [[nodiscard]] auto solve_inverse(const tmat_t& target_tmat) const noexcept {
    //     return m_traj_gen.solve_inverse(m_traj_gen.angles(), target_tmat);
    // }

    // ==============================================================
    // 🌟 [핵심] Motion Commands
    // ==============================================================

    void stop() noexcept { m_traj_gen.stop(); }

    [[nodiscard]] bool trapj(
        const angles_t &goal_angles,
        const std::optional<angles_t> &goal_angvels = std::nullopt,
        const std::optional<angles_t> &peak_v = std::nullopt,
        const std::optional<angles_t> &peak_a = std::nullopt,
        const std::optional<value_t> &duration = std::nullopt) noexcept 
    {
        if (!m_update_timer.is_running()) return false;
        // 🌟 제너레이터의 trapj는 2개의 인자만 받으므로 아래처럼 수정합니다.
        return m_traj_gen.trapj(goal_angles, goal_angvels.value_or(angles_t::Zero()));
    }

    /** @brief attrl (절대 좌표 Pose 이동) */
    [[nodiscard]] bool attrl(const tmat_t &goal_tmat, value_t kp = 50.0) noexcept {
        if (!m_update_timer.is_running()) return false;
        return m_traj_gen.attrl(goal_tmat, kp);
    }

    // -----------------------------------------------------------
    // 🌟 추가된 TCP 정렬 유틸리티 함수 (Generator 연동)
    // -----------------------------------------------------------
    /**
     * @brief 현재 툴의 팁(Z축)이 월드 좌표계의 바닥(-Z)을 수직으로 바라보도록 자세를 정렬합니다.
     */
    [[nodiscard]] bool align_tcp_to_floor(double yaw_deg = 0.0, value_t kp = 100.0) noexcept {
        if (!m_update_timer.is_running()) return false;
        return m_traj_gen.align_tcp_to_floor(yaw_deg, kp);
    }
    
    /**
     * @brief 현재 툴의 팁(Z축)이 월드 좌표계의 정면(X)을 수평으로 바라보도록 자세를 정렬합니다.
     */
    [[nodiscard]] bool align_tcp_to_front(value_t kp = 100.0) noexcept {
        if (!m_update_timer.is_running()) return false;
        return m_traj_gen.align_tcp_to_front(kp);
    }
    // -----------------------------------------------------------

    /** @brief 목표 도달 여부 확인 */
    [[nodiscard]] bool get_goal_reached(
        const std::optional<value_t> &q_th = std::nullopt, // 값을 안 주면 Generator 기본값(2.0) 사용
        const std::optional<value_t> &p_th = std::nullopt, // 값을 안 주면 Generator 기본값(0.002) 사용
        const std::optional<value_t> &r_th = std::nullopt  // 값을 안 주면 Generator 기본값(3.0) 사용
    ) const noexcept { 
        // 입력받은 값을 그대로 넘깁니다. 속도 3개는 항상 nullopt로 검사 생략.
        return m_traj_gen.goal_reached(q_th, p_th, r_th, std::nullopt, std::nullopt, std::nullopt); 
    }
    
    // ==============================================================
    // 🌟 실시간 업데이트 루프 (Internal Timer)
    // ==============================================================

  protected:
    void update() {
        if (!m_is_rt_control_ready.load()) return;

        const double dt = std::chrono::duration_cast<std::chrono::milliseconds>(
                              m_update_timer.interval).count() / 1000.0;

        // 🌟 TrajGenerator 업데이트 (기구학, Jacobian, 궤적 계산 수행)
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
        
        // 두산 로봇 실시간 서보 제어 명령 송신
        draf::_servoj_rt(m_control_rt, q, q_d1, q_d2, m_servoj_target_time);
    }

    // ==============================================================
    // 🌟 데이터 수신 및 유틸리티
    // ==============================================================

  public:
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

    void set_digital_output(GPIO_CTRLBOX_DIGITAL_INDEX index, bool value) {
        if (m_control) draf::_set_digital_output(m_control, index, value);
    }

  private:
    static void TOnMonitoringAccessControlCB(const MONITORING_ACCESS_CONTROL eAccCtrl) {
        if (eAccCtrl == MONITORING_ACCESS_CONTROL_GRANT) m_robot_instance->m_has_control_right.store(true);
        else if (eAccCtrl == MONITORING_ACCESS_CONTROL_LOSS) m_robot_instance->m_has_control_right.store(false);
    }
    static void TOnTpInitializingCompletedCB() { m_robot_instance->m_is_tp_initialized.store(true); }
    static void TOnDisconnectedCB() { m_robot_instance->m_has_control_right.store(false); }

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

    [[nodiscard]] static bool wait_for(std::function<bool()> func, const std::chrono::milliseconds timeout) {
        const auto end_time = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < end_time) {
            if (func()) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return false;
    }

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
};

template <size_t ID> Robot<ID> *Robot<ID>::m_robot_instance = nullptr;

} // namespace rt_control