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

#include "timer.hpp"

#include "../drfl/include/DRFLEx.h"

// 기존 dsr_ 데이터 대신 rt_ 데이터로 변경
#include "rt_data.hpp"
#include "rt_enum.hpp"
#include "model/model.hpp"
#include "trajectory/trajectory_generator.hpp"

namespace rt_control {

constexpr size_t MAX_LOG_QUEUE_SIZE = 10;

namespace draf = DRAFramework;

template <size_t ID = 0> class Robot {
  public:
    using traj_gen_t = trajectory::TrajGenerator;

    using angles_t = rt_control::angles_t;
    using value_t = rt_control::value_t;
    using tmat_t = Eigen::Isometry3d;
    using jmat_t = model::RobotModel::jacobian_t;
    using a_t = Eigen::Matrix<value_t, 6, 1>;
    using angles_set_t = trajectory::TrajGenerator::angles_set_t;

  public:
    constexpr static size_t id = ID;

  public:
    Robot(const std::string& model_name = "m1013",
          const std::chrono::milliseconds update_dt = std::chrono::milliseconds(1))
        : m_model(model_name),
          m_traj_gen(m_model, angles_t::Zero(), angles_t::Zero(), angles_t::Zero()),
          m_update_timer(std::chrono::milliseconds(update_dt), [this]() { update(); }),
          m_is_tp_initialized(false), 
          m_has_control_right(false),
          m_is_rt_control_ready(false), 
          m_servoj_target_time(0.05) {
        
        // 🌟 생성자에서 모델 확인 로그 출력
        if (m_model.get_model_name() != "m1013") {
            std::cerr << "[RT_CONTROL] Warning: 현재 두산 API는 'm1013' 모델만 지원합니다. 통신 기능이 제한됩니다." << std::endl;
        }
        
        m_robot_instance = this;
    }

  public:
    [[nodiscard]] OpenConnError open_connection(
        const std::string &strIpAddr = "192.168.137.100",
        const uint32_t usPort = 12345,
        const std::chrono::milliseconds timeout_init_tp = std::chrono::milliseconds(100),
        const std::chrono::milliseconds timeout_get_ctrl = std::chrono::milliseconds(500)) 
    {
        // 🌟 "m1013" 모델이 아니면 API 연동 차단
        if (m_model.get_model_name() != "m1013") {
            std::cerr << "[RT_CONTROL] Error: 두산 로봇 제어기는 'm1013' 모델만 연결할 수 있습니다." << std::endl;
            return OpenConnError::CREATE_ROBOT_CONTROL_ERROR;
        }

        std::lock_guard<std::mutex> lock(m_control_mutex);
        if (m_control) { m_control.reset(); }

        m_is_tp_initialized.store(false);
        m_has_control_right.store(false);

        draf::_set_on_monitoring_speed_mode(m_control.get(), TOnMonitoringSpeedModeCB);
        draf::_set_on_monitoring_access_control(m_control.get(), TOnMonitoringAccessControlCB);
        draf::_set_on_log_alarm(m_control.get(), TOnLogAlarmCB);
        draf::_set_on_tp_initializing_completed(m_control.get(), TOnTpInitializingCompletedCB);
        draf::_set_on_disconnected(m_control.get(), TOnDisconnectedCB);

        if (!draf::_open_connection(m_control.get(), strIpAddr.c_str(), usPort)) {
            m_control.reset();
            return OpenConnError::OPEN_CONNECTION_ERROR;
        }

        if (!wait_for([this]() { return m_is_tp_initialized.load(); }, timeout_init_tp)) {
            m_control.reset();
            return OpenConnError::INITIALIZE_TP_ERROR;
        }

        if (!draf::_manage_access_control(m_control.get(), MANAGE_ACCESS_CONTROL_FORCE_REQUEST)) {
            m_control.reset();
            return OpenConnError::MANAGE_ACCESS_CONTROL_ERROR;
        }

        if (!wait_for([this]() { return m_has_control_right.load(); }, timeout_get_ctrl)) {
            m_control.reset();
            return OpenConnError::GET_CONTROL_RIGHT_ERROR;
        }

        return OpenConnError::NO_ERROR;
    }

    [[nodiscard]] bool connect_rt(
        const std::string &strIpAddr = "192.168.137.100",
        const uint32_t usPort = 12347) 
    {
        std::lock_guard<std::mutex> lock(m_control_rt_mutex);
        if (m_control_rt) { m_control_rt.reset(); }

        m_control_rt = std::unique_ptr<draf::LPROBOTCONTROL, decltype(&close_and_destroy_ctrl_udp)>(
            static_cast<draf::LPROBOTCONTROL *>(draf::_create_robot_control_udp()), close_and_destroy_ctrl_udp);

        if (!m_control_rt) { return false; }

        m_is_rt_control_ready.store(false);

        if (!draf::_connect_rt_control(m_control_rt.get(), strIpAddr.c_str(), usPort)) {
            m_control_rt.reset();
            return false;
        }

        if (!draf::_set_rt_control_output(m_control_rt.get(), "v1.0", 0.001, 4)) {
            m_control_rt.reset();
            return false;
        }

        if (!draf::_start_rt_control(m_control_rt.get())) {
            m_control_rt.reset();
            return false;
        }

        m_is_rt_control_ready.store(true);
        return true;
    }

    [[nodiscard]] ServoOnError servo_on(
        const std::chrono::milliseconds timeout = std::chrono::milliseconds(3000)) 
    {
        // 🌟 하드웨어 안전을 위해 한 번 더 모델 확인
        if (m_model.get_model_name() != "m1013") {
            return ServoOnError::NO_ROBOT_CONTROL_ERROR;
        }

        std::lock_guard<std::mutex> lock(m_control_mutex);
        if (!m_control) { return ServoOnError::NO_ROBOT_CONTROL_ERROR; }

        if (!draf::_set_robot_mode(m_control.get(), ROBOT_MODE_AUTONOMOUS)) {
            return ServoOnError::SET_ROBOT_MODE_ERROR;
        }

        if (!draf::_set_robot_control(m_control.get(), CONTROL_RESET_SAFET_OFF)) {
            return ServoOnError::SET_ROBOT_CONTROL_ERROR;
        }

        if (!wait_for([this]() { return draf::_get_robot_state(m_control.get()) == STATE_STANDBY; }, timeout)) {
            return ServoOnError::GET_STATE_STANDBY_ERROR;
        }

        if (m_is_rt_control_ready.load()) {
            if (!draf::_set_safety_mode(m_control.get(), SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE)) {
                return ServoOnError::SET_SAFETY_MODE_ERROR;
            }
        }

        const auto start_angles = get_current_angles();
        const auto start_angvels = get_current_angvels();
        if (!start_angles.has_value() || !start_angvels.has_value()) {
            return ServoOnError::GET_ROBOT_STATE_ERROR;
        }

        m_traj_gen.initialize(m_model, start_angles.value(), start_angvels.value(), angles_t::Zero());
        m_update_timer.start();

        return ServoOnError::NO_ERROR;
    }

    [[nodiscard]] ServoOffError servo_off(
        const std::chrono::milliseconds timeout = std::chrono::milliseconds(100)) 
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        std::lock_guard<std::mutex> lock(m_control_mutex);
        if (!m_control) { return ServoOffError::NO_ROBOT_CONTROL_ERROR; }

        m_update_timer.stop();

        if (!draf::_servo_off(m_control.get(), STOP_TYPE_QUICK)) {
            return ServoOffError::SET_SERVO_OFF_ERROR;
        }

        if (!wait_for([this]() { return draf::_get_robot_state(m_control.get()) == STATE_SAFE_OFF; }, timeout)) {
            return ServoOffError::GET_ROBOT_STATE_ERROR;
        }

        return ServoOffError::NO_ERROR;
    }

    void disconnect_rt() {
        std::lock_guard<std::mutex> lock(m_control_rt_mutex);
        if (!m_control_rt) { return; }

        draf::_stop_rt_control(m_control_rt.get());
        m_control_rt.reset();
    }

    [[nodiscard]] CloseConnError close_connection(
        const std::chrono::milliseconds timeout = std::chrono::milliseconds(100)) 
    {
        std::lock_guard<std::mutex> lock(m_control_mutex);
        if (!m_control) { return CloseConnError::NO_ROBOT_CONTROL_ERROR; }

        m_control.reset();

        if (!wait_for([this]() { return !m_has_control_right.load(); }, timeout)) {
            return CloseConnError::LOSE_CONTROL_RIGHT_ERROR;
        }

        return CloseConnError::NO_ERROR;
    }

    //////////////////////////////////////////////////////////////

  public:
    void set_tcp_tmat(const tmat_t &new_shift_tmat) noexcept {
        m_traj_gen.set_tcp_tmat(new_shift_tmat);
    }

    [[nodiscard]] tmat_t get_tcp_tmat() const noexcept {
        return traj_gen().get_tcp_tmat();
    }

    [[nodiscard]] bool set_tool(const std::string &lpszSymbol) noexcept {
        if (!m_control) { return false; }
        return draf::_set_tool(m_control.get(), lpszSymbol.c_str());
    }

    [[nodiscard]] std::optional<std::string> get_tool() const noexcept {
        if (!m_control) { return std::nullopt; }
        return draf::_get_tool(m_control.get());
    }

    [[nodiscard]] bool add_tool(const std::string &lpszSymbol,
                                const float fWeight, std::array<float, 3> &fCog,
                                std::array<float, 6> &fInertia) noexcept {
        if (!m_control) { return false; }
        return draf::_add_tool(m_control.get(), lpszSymbol.c_str(), fWeight, fCog.data(), fInertia.data());
    }

    [[nodiscard]] bool del_tool(const std::string &lpszSymbol) noexcept {
        if (!m_control) { return false; }
        return draf::_del_tool(m_control.get(), lpszSymbol.c_str());
    }

    [[nodiscard]] bool set_safety_mode(const SAFETY_MODE eSafetyMode,
                                       const SAFETY_MODE_EVENT eSafetyEvent) noexcept {
        if (!m_control) { return false; }
        return draf::_set_safety_mode(m_control.get(), eSafetyMode, eSafetyEvent);
    };

    [[nodiscard]] bool change_collision_sensitivity(const float fSensitivity) noexcept {
        if (!m_control) { return false; }
        return draf::_change_collision_sensitivity(m_control.get(), fSensitivity);
    };

    void set_servoj_target_time(const float &new_target_time) noexcept {
        m_servoj_target_time = std::max(new_target_time, 0.01f);
    }

    [[nodiscard]] const float &get_servoj_target_time() const noexcept {
        return m_servoj_target_time;
    }

    //////////////////////////////////////////////////////////////

  public:
    void stop() noexcept { m_traj_gen.stop(); }

    [[nodiscard]] bool stop_mwait(
        const std::chrono::milliseconds timeout = std::chrono::milliseconds(3000)) noexcept {
        stop();
        return wait_for([&]() { return get_traj_state() != trajectory::TrajState::STOPPING; }, 
                        timeout, std::chrono::milliseconds(1));
    }

    [[nodiscard]] bool trapj(
        const angles_t &goal_angles,
        const std::optional<angles_t> &goal_angvels = std::nullopt,
        const std::optional<angles_t> &peak_angvels = std::nullopt,
        const std::optional<angles_t> &peak_angaccs = std::nullopt,
        const std::optional<value_t> &duration = std::nullopt) noexcept 
    {
        if (!m_update_timer.is_running()) { return false; }
        return m_traj_gen.trapj(goal_angles, goal_angvels, peak_angvels, peak_angaccs, duration);
    }

    [[nodiscard]] bool trapj_mwait(
        const angles_t &goal_angles,
        const std::optional<angles_t> &goal_angvels = std::nullopt,
        const std::optional<angles_t> &peak_angvels = std::nullopt,
        const std::optional<angles_t> &peak_angaccs = std::nullopt,
        const std::optional<value_t> &duration = std::nullopt) noexcept 
    {
        if (!trapj(goal_angles, goal_angvels, peak_angvels, peak_angaccs, duration)) {
            return false;
        }
        return mwait(); 
    }

    [[nodiscard]] bool attrj(
        const angles_t &goal_angles, const value_t &kp,
        const std::optional<angles_t> &goal_angvels = std::nullopt,
        const std::optional<angles_t> &peak_angvels = std::nullopt,
        const std::optional<angles_t> &peak_angaccs = std::nullopt) noexcept 
    {
        if (!m_update_timer.is_running()) { return false; }
        return m_traj_gen.attrj(goal_angles, kp, goal_angvels, peak_angvels, peak_angaccs);
    }

    [[nodiscard]] bool attrj_mwait(
        const angles_t &goal_angles, const value_t &kp,
        const std::chrono::milliseconds timeout = std::chrono::milliseconds(10000)) noexcept 
    {
        if (!attrj(goal_angles, kp)) { return false; }
        return mwait(timeout);
    }

    [[nodiscard]] bool attrl(
        const tmat_t &goal_tmat, const value_t kp = 10.0,
        const std::optional<value_t> &peak_endvel = std::nullopt, 
        const std::optional<value_t> &peak_endacc = std::nullopt) noexcept 
    {
        if (!m_update_timer.is_running()) { return false; }
        return m_traj_gen.attrl(goal_tmat, kp, peak_endvel, peak_endacc);
    }

    [[nodiscard]] bool attrl_mwait(
        const tmat_t &goal_tmat, const value_t kp = 10.0,
        const std::chrono::milliseconds timeout = std::chrono::milliseconds(10000)) noexcept 
    {
        if (!attrl(goal_tmat, kp)) { return false; }
        return mwait(timeout);
    }

    [[nodiscard]] bool playj(
        const angles_set_t &goal_angles_set,
        const std::optional<angles_set_t> &goal_angvels_set = std::nullopt,
        const std::optional<angles_set_t> &goal_angaccs_set = std::nullopt,
        const std::optional<angles_t> &peak_angvels = std::nullopt,
        const std::optional<angles_t> &peak_angaccs = std::nullopt) noexcept 
    {
        if (!m_update_timer.is_running()) { return false; }
        return m_traj_gen.playj(goal_angles_set, goal_angvels_set, goal_angaccs_set, peak_angvels, peak_angaccs);
    }

    [[nodiscard]] bool playj_mwait(
        const angles_set_t &goal_angles_set,
        const std::optional<angles_set_t> &goal_angvels_set = std::nullopt,
        const std::optional<angles_set_t> &goal_angaccs_set = std::nullopt,
        const std::optional<angles_t> &peak_angvels = std::nullopt,
        const std::optional<angles_t> &peak_angaccs = std::nullopt,
        const std::chrono::milliseconds timeout = std::chrono::milliseconds(10000)) noexcept 
    {
        if (!playj(goal_angles_set, goal_angvels_set, goal_angaccs_set, peak_angvels, peak_angaccs)) {
            return false;
        }
        return wait_for([&]() { return get_traj_state() != trajectory::TrajState::PLAYJ; }, 
                        timeout, std::chrono::milliseconds(1));
    }

    [[nodiscard]] bool mwait(
        const std::chrono::milliseconds timeout = std::chrono::milliseconds(10000)) noexcept 
    {
        return wait_for([&]() { return get_goal_reached(); }, timeout, std::chrono::milliseconds(1));
    }

    //////////////////////////////////////////////////////////////

  public:
    [[nodiscard]] std::optional<ROBOT_STATE> get_robot_state() const noexcept {
        if (!m_control) { return std::nullopt; }
        return draf::_get_robot_state(m_control.get());
    };

    [[nodiscard]] std::optional<angles_t> get_current_angles() const noexcept {
        if (!m_control_rt) { return std::nullopt; }
        const float* pData = draf::_read_data_rt(m_robot_instance->m_control_rt.get())->actual_joint_position;
        return Eigen::Map<const Eigen::Matrix<float, 6, 1>>(pData).cast<double>();
    }

    [[nodiscard]] std::optional<angles_t> get_current_angvels() const noexcept {
        if (!m_control_rt) { return std::nullopt; }
        const float* pData = draf::_read_data_rt(m_robot_instance->m_control_rt.get())->actual_joint_velocity;
        return Eigen::Map<const Eigen::Matrix<float, 6, 1>>(pData).cast<double>();
    }

    [[nodiscard]] std::optional<tmat_t> get_current_tmat() const noexcept {
        const auto angles = get_current_angles();
        if (!angles.has_value()) { return std::nullopt; }
        return solve_forward(angles.value());
    }

    [[nodiscard]] std::optional<jmat_t> get_current_jmat() const noexcept {
        const auto angles = get_current_angles();
        if (!angles.has_value()) { return std::nullopt; }
        return m_model.jacobian(angles.value() * model::RobotModel::DEG2RAD);
    }

    [[nodiscard]] std::optional<a_t> get_current_a() const noexcept {
        const auto jmat = get_current_jmat();
        const auto angvels = get_current_angvels();
        if (!jmat.has_value() || !angvels.has_value()) { return std::nullopt; }
        return jmat.value() * (angvels.value() * model::RobotModel::DEG2RAD);
    }

    [[nodiscard]] std::optional<angles_t> get_current_joint_torque() const noexcept {
        if (!m_control_rt) { return std::nullopt; }
        const float* pData = draf::_read_data_rt(m_robot_instance->m_control_rt.get())->actual_joint_torque;
        return Eigen::Map<const Eigen::Matrix<float, 6, 1>>(pData).cast<double>();
    }

    [[nodiscard]] std::optional<angles_t> get_current_joint_external_torque() const noexcept {
        if (!m_control_rt) { return std::nullopt; }
        const float* pData = draf::_read_data_rt(m_robot_instance->m_control_rt.get())->external_joint_torque;
        return Eigen::Map<const Eigen::Matrix<float, 6, 1>>(pData).cast<double>();
    }

    [[nodiscard]] std::optional<a_t> get_current_end_external_force() const noexcept {
        const auto jmat = get_current_jmat();
        const auto torque = get_current_joint_external_torque();
        if (!jmat.has_value() || !torque.has_value()) { return std::nullopt; }
        return jmat.value().inverse() * torque.value();
    }

    //////////////////////////////////////////////////////////////

    [[nodiscard]] const auto &get_traj_state() const noexcept { return traj_gen().traj_state(); }
    [[nodiscard]] const auto &get_desired_angles() const noexcept { return traj_gen().angles(); }
    [[nodiscard]] const auto &get_desired_angvels() const noexcept { return traj_gen().angvels(); }
    [[nodiscard]] const auto &get_desired_angaccs() const noexcept { return traj_gen().angaccs(); }
    [[nodiscard]] const auto &get_desired_tmat() const noexcept { return traj_gen().tmat(); }
    [[nodiscard]] const auto &get_desired_a() const noexcept { return traj_gen().a(); }
    [[nodiscard]] const auto &get_desired_a_d1() const noexcept { return traj_gen().a_d1(); }

    //////////////////////////////////////////////////////////////

    [[nodiscard]] std::optional<angles_t> get_goal_angles() const noexcept { return traj_gen().goal_angles(); }
    [[nodiscard]] std::optional<angles_t> get_goal_angvels() const noexcept { return traj_gen().goal_angvels(); }
    [[nodiscard]] std::optional<tmat_t> get_goal_tmat() const noexcept { return traj_gen().goal_tmat(); }
    [[nodiscard]] std::optional<a_t> get_goal_a() const noexcept { return traj_gen().goal_a(); }

    [[nodiscard]] bool get_goal_reached() const noexcept {
        return traj_gen().goal_reached();
    }

    //////////////////////////////////////////////////////////////

    [[nodiscard]] tmat_t solve_forward(const angles_t& angles) const noexcept {
        return traj_gen().solve_forward(angles);
    }

    [[nodiscard]] auto solve_inverse(
        const angles_t& initial_angles, const tmat_t& target_tmat,
        const size_t step_max = 10000,
        const double &enorm_threshold = 1e-4,
        const double &damping = 0.01) const noexcept 
    {
        return traj_gen().solve_inverse(initial_angles, target_tmat, step_max, enorm_threshold, damping);
    }

    //////////////////////////////////////////////////////////////

  protected:
    void update() {
        if (!m_is_rt_control_ready) { return; }

        const double dt = std::chrono::duration_cast<std::chrono::milliseconds>(
                              m_update_timer.interval).count() / 1000.0;

        m_traj_gen.update(dt);

        float q[NUMBER_OF_JOINT];
        float q_d1[NUMBER_OF_JOINT];
        float q_d2[NUMBER_OF_JOINT];
        
        angles_t des_q = get_desired_angles();
        angles_t des_dq = get_desired_angvels();
        angles_t des_ddq = get_desired_angaccs();

        for (int i = 0; i < NUMBER_OF_JOINT; i++) {
            q[i] = static_cast<float>(des_q(i));
            q_d1[i] = static_cast<float>(des_dq(i));
            q_d2[i] = static_cast<float>(des_ddq(i));
        }

        draf::_servoj_rt(m_robot_instance->m_control_rt.get(), q, q_d1, q_d2, get_servoj_target_time());
    }

  private:
    static void TOnMonitoringSpeedModeCB(const MONITORING_SPEED eSpdMode) {
        if (eSpdMode == SPEED_REDUCED_MODE) {
            std::cerr << "[RT_CONTROL] SPEED MODE: REDUCED!!!" << std::endl;
        }
    }

    static void TOnMonitoringAccessControlCB(const MONITORING_ACCESS_CONTROL eAccCtrl) {
        switch (eAccCtrl) {
        case MONITORING_ACCESS_CONTROL_REQUEST:
            draf::_manage_access_control(m_robot_instance->m_control.get(), MANAGE_ACCESS_CONTROL_RESPONSE_NO);
            break;
        case MONITORING_ACCESS_CONTROL_DENY:
            break;
        case MONITORING_ACCESS_CONTROL_GRANT:
            m_robot_instance->m_has_control_right.store(true);
            break;
        case MONITORING_ACCESS_CONTROL_LOSS:
            m_robot_instance->m_has_control_right.store(false);
            break;
        default:
            break;
        }
    }

    static void TOnLogAlarmCB(const LPLOG_ALARM pLogAlarm) {
        const LogAlarm alarm = {std::chrono::system_clock::now(),
                                pLogAlarm->_iLevel, pLogAlarm->_iGroup, pLogAlarm->_iIndex,
                                pLogAlarm->_szParam[0], pLogAlarm->_szParam[1], pLogAlarm->_szParam[2]};

        {
            std::lock_guard<std::mutex> lock(m_robot_instance->m_log_queue_mutex);
            while (MAX_LOG_QUEUE_SIZE <= m_robot_instance->m_log_queue.size()) {
                m_robot_instance->m_log_queue.pop();
            }
            m_robot_instance->m_log_queue.push(alarm);
        }
    }

    static void TOnTpInitializingCompletedCB() {
        m_robot_instance->m_is_tp_initialized.store(true);
    }

    static void TOnDisconnectedCB() {
        m_robot_instance->m_has_control_right.store(false);
    }

  private:
    static void close_and_destroy_ctrl(draf::LPROBOTCONTROL pCtrl) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        { draf::_close_connection(pCtrl); }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        { draf::_DestroyRobotControl(pCtrl); }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    static void close_and_destroy_ctrl_udp(draf::LPROBOTCONTROL pCtrlUDP) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        { draf::_disconnect_rt_control(pCtrlUDP); }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        { draf::_destroy_robot_control_udp(pCtrlUDP); }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    template <typename T>
    static void clear_queue(std::queue<T> &queue, std::mutex &mutex) {
        std::lock_guard<std::mutex> lock(mutex);
        while (!queue.empty()) { queue.pop(); }
    }

    template <typename T, typename Func>
    static void process_queue(std::queue<T> &queue, std::mutex &mutex, Func process) {
        std::unique_lock<std::mutex> lock(mutex);
        if (!queue.empty()) {
            T data = queue.front();
            lock.unlock();
            if (process(data)) {
                lock.lock();
                queue.pop();
            }
        }
    }

    [[nodiscard]] static bool wait_for(
        std::function<bool()> func, const std::chrono::milliseconds timeout,
        const std::chrono::milliseconds delay = std::chrono::milliseconds(10)) 
    {
        const auto end_time = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < end_time) {
            if (func()) { return true; }
            std::this_thread::sleep_for(delay);
        }
        return false;
    }

  public:
    [[nodiscard]] const traj_gen_t &traj_gen() const noexcept { return m_traj_gen; }

  private:
    static Robot<ID> *m_robot_instance;
    rt_control::model::RobotModel m_model; 
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

    float m_servoj_target_time; 
};

template <size_t ID> Robot<ID> *Robot<ID>::m_robot_instance = nullptr;

} // namespace rt_control