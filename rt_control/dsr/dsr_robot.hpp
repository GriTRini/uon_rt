#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <condition_variable>
#include <format>
#include <functional>
#include <iostream>
#include <math.h>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

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

    enum class ActiveMotion { NONE, TRAPJ, ATTRL };

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

        if (m_is_rt_control_ready.load()) {
            draf::_set_safety_mode(m_control, SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);
        }

        const auto start_angles = this->get_current_angles();
        const auto start_angvels = this->get_current_angvels();
        if (start_angles.has_value() && start_angvels.has_value()) {
            m_traj_gen.initialize(m_model, start_angles.value(), start_angvels.value(), angles_t::Zero());
        }

        m_is_paused.store(false);
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

    // ==============================================================
    // 🌟 실시간 기구학 및 TCP 정보 인터페이스
    // ==============================================================

    void set_tcp(value_t x, value_t y, value_t z, value_t r_deg, value_t p_deg, value_t yaw_deg) noexcept {
        m_traj_gen.set_tcp(x, y, z, r_deg, p_deg, yaw_deg);
    }

    [[nodiscard]] const tmat_t& get_task_pos() const noexcept { return m_traj_gen.tmat(); }
    [[nodiscard]] const jmat_t& get_jacobian() const noexcept { return m_traj_gen.jmat(); }
    [[nodiscard]] const a_t& get_task_vel() const noexcept { return m_traj_gen.a(); }

    [[nodiscard]] tmat_t solve_forward(const angles_t& q) const noexcept {
        return m_traj_gen.solve_forward(q);
    }

    // ==============================================================
    // 🌟 Motion Commands
    // ==============================================================

    void stop() noexcept { m_traj_gen.stop(); }

    [[nodiscard]] bool trapj(
        const angles_t &goal_angles,
        const std::optional<angles_t> &goal_angvels = std::nullopt) noexcept 
    {
        if (!m_update_timer.is_running()) return false;
        
        m_active_motion.store(ActiveMotion::TRAPJ);
        m_saved_goal_angles = goal_angles;
        m_is_paused.store(false);
        
        return m_traj_gen.trapj(goal_angles, goal_angvels.value_or(angles_t::Zero()));
    }

    [[nodiscard]] bool attrl(const tmat_t &goal_tmat, value_t kp = 50.0) noexcept {
        if (!m_update_timer.is_running()) return false;

        m_active_motion.store(ActiveMotion::ATTRL);
        m_saved_goal_tmat = goal_tmat;
        m_saved_kp = kp;
        m_is_paused.store(false);

        return m_traj_gen.attrl(goal_tmat, kp);
    }

    [[nodiscard]] bool movel(double x, double y, double z, value_t kp = 50.0) noexcept {
        if (!m_update_timer.is_running()) return false;
        tmat_t target = get_task_pos();
        target.translation() << x, y, z;
        return attrl(target, kp); // attrl을 호출하여 상태 백업 재사용
    }

    [[nodiscard]] bool align_tcp_to_floor(double yaw_deg = 0.0, value_t kp = 100.0) noexcept {
        if (!m_update_timer.is_running()) return false;
        return m_traj_gen.align_tcp_to_floor(yaw_deg, kp);
    }
    
    [[nodiscard]] bool align_tcp_to_front(value_t kp = 100.0) noexcept {
        if (!m_update_timer.is_running()) return false;
        return m_traj_gen.align_tcp_to_front(kp);
    }

    [[nodiscard]] bool get_goal_reached(
        const std::optional<value_t> &q_th = std::nullopt,
        const std::optional<value_t> &p_th = std::nullopt,
        const std::optional<value_t> &r_th = std::nullopt
    ) const noexcept { 
        return m_traj_gen.goal_reached(q_th, p_th, r_th, std::nullopt, std::nullopt, std::nullopt); 
    }

    // ==============================================================
    // 🌟 안전 해제 및 궤적 이어서 가기 (Resume)
    // ==============================================================

    [[nodiscard]] bool resume_trajectory() {
        if (!m_is_paused.load()) return true;

        if (draf::_get_robot_state(m_control) != STATE_STANDBY) {
            std::cerr << "아직 로봇이 Standby 상태가 아닙니다. 펜던트에서 에러를 해제하세요." << std::endl;
            return false;
        }

        draf::_set_safety_mode(m_control, SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);

        ActiveMotion current_mode = m_active_motion.load();
        
        if (current_mode == ActiveMotion::TRAPJ) {
            std::cout << "[Resume] TrapJ 궤적을 현재 위치에서 재계획합니다." << std::endl;
            m_traj_gen.trapj(m_saved_goal_angles, angles_t::Zero());
        } 
        else if (current_mode == ActiveMotion::ATTRL) {
            std::cout << "[Resume] AttrL 궤적을 현재 위치에서 이어서 당깁니다." << std::endl;
            m_traj_gen.attrl(m_saved_goal_tmat, m_saved_kp);
        }

        m_is_paused.store(false);
        return true;
    }

    // ==============================================================
    // 🌟 안전 해제 및 궤적 처음부터 가기 (Cancel)
    // ==============================================================

    [[nodiscard]] bool cancel_trajectory() {
        if (!m_is_paused.load()) return true;

        if (draf::_get_robot_state(m_control) != STATE_STANDBY) {
            std::cerr << "아직 로봇이 Standby 상태가 아닙니다. 펜던트에서 에러를 해제하세요." << std::endl;
            return false;
        }

        // 서보 온 및 제어 가능 상태로만 복구
        draf::_set_safety_mode(m_control, SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);

        // 궤적을 재계획하지 않고, 변수만 초기화하여 현재 위치에 대기시킴
        m_active_motion.store(ActiveMotion::NONE);
        m_is_paused.store(false);
        
        std::cout << "[Cancel] 기존 궤적을 폐기하고 대기 상태로 전환했습니다." << std::endl;
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

        bool is_physical_stop = (rt_data->robot_state == STATE_SAFE_STOP || 
                                 rt_data->robot_state == STATE_ERROR ||
                                 rt_data->robot_state == STATE_EMERGENCY_STOP);

        if (is_physical_stop && !m_is_paused.load()) {
            m_is_paused.store(true);
            
            const float* act_q = rt_data->actual_joint_position;
            const float* act_dq = rt_data->actual_joint_velocity;
            
            angles_t sync_q = Eigen::Map<const Eigen::Matrix<float, 6, 1>>(act_q).cast<double>();
            angles_t sync_dq = Eigen::Map<const Eigen::Matrix<float, 6, 1>>(act_dq).cast<double>();
            
            m_traj_gen.initialize(m_model, sync_q, sync_dq, angles_t::Zero());
            std::cout << "[Safety Sync] 로봇 정지 감지! 제어기 좌표를 실제 좌표로 동기화했습니다." << std::endl;
        }

        if (m_is_paused.load()) {
            float q[6], q_d1[6] = {0}, q_d2[6] = {0};
            for (int i = 0; i < 6; i++) q[i] = static_cast<float>(m_traj_gen.angles()(i));
            draf::_servoj_rt(m_control_rt, q, q_d1, q_d2, m_servoj_target_time);
            return; 
        }

        const double dt = std::chrono::duration_cast<std::chrono::milliseconds>(
                              m_update_timer.interval).count() / 1000.0;

        m_traj_gen.update(dt);

        float q[6], q_d1[6], q_d2[6];
        for (int i = 0; i < 6; i++) {
            q[i] = static_cast<float>(m_traj_gen.angles()(i));
            q_d1[i] = static_cast<float>(m_traj_gen.angvels()(i));
            q_d2[i] = static_cast<float>(m_traj_gen.angaccs()(i));
        }
        
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

    // 상태 백업 변수
    std::atomic<bool> m_is_paused{};
    std::atomic<ActiveMotion> m_active_motion{};
    angles_t m_saved_goal_angles;
    tmat_t m_saved_goal_tmat;
    value_t m_saved_kp = 50.0;
};

template <size_t ID> Robot<ID> *Robot<ID>::m_robot_instance = nullptr;

} // namespace dsr_control