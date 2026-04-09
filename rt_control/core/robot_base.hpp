#pragma once

#include <string>
#include <memory>
#include <optional>
#include <chrono>
#include <functional>
#include <atomic>

#include "core.hpp" 
#include "../timer.hpp"
#include "../model/model.hpp"
#include "../trajectory/trajectory_generator.hpp"

namespace rt_control {

class RobotBase {
public:
    using traj_gen_t = trajectory::TrajGenerator;
    using angles_t = rt_control::angles_t; // 명시적 네임스페이스 (경고 해결)
    using value_t = rt_control::value_t;   // 명시적 네임스페이스 (경고 해결)
    using tmat_t = Eigen::Isometry3d;
    using jmat_t = Eigen::Matrix<double, 6, 6>;
    using a_t = Eigen::Matrix<value_t, 6, 1>;

public:
    RobotBase(const std::string& model_name, 
              const std::chrono::milliseconds update_dt = std::chrono::milliseconds(1))
        : m_model(model_name),
          m_traj_gen(),
          m_update_timer(std::chrono::milliseconds(update_dt), [this]() { update(); })
    {
        m_traj_gen.initialize(m_model, angles_t::Zero(), angles_t::Zero(), angles_t::Zero());
    }

    virtual ~RobotBase() {
        m_update_timer.stop();
    }

    // ==============================================================
    // 🌟 공통 수학 및 기구학 API (자식 클래스들이 모두 물려받음)
    // ==============================================================
    void set_tcp(value_t x, value_t y, value_t z, value_t r_deg, value_t p_deg, value_t yaw_deg) noexcept {
        m_traj_gen.set_tcp(x, y, z, r_deg, p_deg, yaw_deg);
    }

    // 현재 TCP 포즈 
    [[nodiscard]] const tmat_t& get_current_pos() const noexcept { return m_traj_gen.tmat(); }
    
    // 🌟 신규 추가: 순수 로봇 손목(Flange) 포즈 반환
    [[nodiscard]] tmat_t get_current_flange_pos() const noexcept { return m_traj_gen.flange_tmat(); }

    [[nodiscard]] const jmat_t& get_jacobian() const noexcept { return m_traj_gen.jmat(); }
    [[nodiscard]] const a_t& get_task_vel() const noexcept { return m_traj_gen.a(); }
    [[nodiscard]] tmat_t solve_forward(const angles_t& q) const noexcept { return m_traj_gen.solve_forward(q); }

    // ==============================================================
    // 🌟 공통 궤적 제어 API (자식 클래스들이 모두 물려받음)
    // ==============================================================
    void stop() noexcept { m_traj_gen.stop(); }

    [[nodiscard]] bool trapj(const angles_t &goal_angles, const std::optional<angles_t> &goal_angvels = std::nullopt) noexcept {
        if (!m_update_timer.is_running()) return false;
        return m_traj_gen.trapj(goal_angles, goal_angvels.value_or(angles_t::Zero()));
    }

    [[nodiscard]] bool attrl(const tmat_t &goal_tmat, value_t kp = 50.0) noexcept {
        if (!m_update_timer.is_running()) return false;
        return m_traj_gen.attrl(goal_tmat, kp);
    }

    [[nodiscard]] bool attrl(const value_t x, const value_t y, const value_t z, 
                             const value_t r_deg, const value_t p_deg, const value_t yaw_deg, 
                             const value_t kp = 50.0) noexcept {
        if (!m_update_timer.is_running()) return false;
        return m_traj_gen.attrl(x, y, z, r_deg, p_deg, yaw_deg, kp);
    }

    // 🌟 TCP 정렬 API (제조사 상관없이 Generator에서 계산)
    [[nodiscard]] bool align_tcp_to_floor(double yaw_deg = 0.0, value_t kp = 100.0) noexcept {
        if (!m_update_timer.is_running()) return false;
        return m_traj_gen.align_tcp_to_floor(yaw_deg, kp);
    }

    [[nodiscard]] bool align_tcp_to_front(value_t kp = 100.0) noexcept {
        if (!m_update_timer.is_running()) return false;
        return m_traj_gen.align_tcp_to_front(kp);
    }

    [[nodiscard]] bool get_goal_reached(const std::optional<value_t> &q_th = std::nullopt,
                                        const std::optional<value_t> &p_th = std::nullopt,
                                        const std::optional<value_t> &r_th = std::nullopt) const noexcept {
        return m_traj_gen.goal_reached(q_th, p_th, r_th, std::nullopt, std::nullopt, std::nullopt);
    }

    // ==============================================================
    // 🔌 하드웨어 종속 API (자식 클래스인 dsr_robot 등이 오버라이딩함)
    // ==============================================================
    virtual bool open_connection(const std::string &strIpAddr = "192.168.1.30", const uint32_t usPort = 12345) = 0;
    virtual bool close_connection() = 0;
    virtual bool connect_rt(const std::string &strIpAddr = "192.168.1.30", const uint32_t usPort = 12347) = 0;
    virtual void disconnect_rt() = 0;
    
    virtual bool servo_on() = 0;
    virtual bool servo_off() = 0;

    virtual std::optional<angles_t> get_current_angles() const noexcept = 0;
    virtual std::optional<angles_t> get_current_angvels() const noexcept = 0;
    virtual void set_digital_output(int index, bool value) = 0;

protected:
    // 자식 클래스에서 실시간 통신 송수신을 구현할 함수
    virtual void update() = 0;

protected:
    model::RobotModel m_model;
    traj_gen_t m_traj_gen;
    uon::timer::Timer<int64_t, std::milli> m_update_timer;
};

} // namespace rt_control