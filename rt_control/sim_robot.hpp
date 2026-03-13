#pragma once

#include "model/model.hpp"
#include "rt_data.hpp"
#include "trajectory/trajectory_trapj.hpp"
#include "trajectory/trajectory_attrj.hpp"
#include "trajectory/trajectory_attrl.hpp"
#include "trajectory/trajectory_playj.hpp" // 🌟 PlayJ 추가

#include <memory>
#include <optional>

namespace rt_control {

class SimRobot {
public:
    using angles_t = rt_control::angles_t;
    using value_t = rt_control::value_t;

protected:
    std::shared_ptr<model::RobotModel> m_model; // 주입받은 모델
    TrajState m_traj_state = TrajState::STOP;

    // 궤적 엔진들
    std::unique_ptr<trajectory::TrajTrapJ> m_trapj;
    std::unique_ptr<trajectory::TrajAttrJ> m_attrj;
    std::unique_ptr<trajectory::TrajAttrL> m_attrl;
    std::unique_ptr<trajectory::TrajPlayJ> m_playj; // 🌟 PlayJ 엔진 추가

    // 내부 상태 (Deg 기준)
    angles_t m_angles = angles_t::Zero();
    angles_t m_angvels = angles_t::Zero();
    
    // 🌟 TCP 오프셋 (수학 엔진용)
    Eigen::Isometry3d m_tcp_tmat = Eigen::Isometry3d::Identity();

public:
    explicit SimRobot(std::shared_ptr<model::RobotModel> model) 
        : m_model(std::move(model)) {}

    virtual ~SimRobot() = default;

    // 🌟 [TCP 설정] 기구학 좌표계 업데이트
    void set_tcp_tmat(const Eigen::Matrix4d& tmat) { 
        m_tcp_tmat.matrix() = tmat; 
    }

    // 🌟 [동기화] 외부(Real/Python)에서 현재 로봇의 실제 값을 엔진에 주입
    void set_current_state(const angles_t& q_deg, const angles_t& dq_deg) {
        m_angles = q_deg;
        m_angvels = dq_deg;
    }

    // 🌟 [기구학 API] TCP 오프셋이 적용된 순기구학 결과 반환
    Eigen::Matrix4d solve_forward(const angles_t& q_deg) const {
        return (m_model->forward_kinematics(q_deg) * m_tcp_tmat).matrix();
    }

    // ==========================================================
    // 궤적 제어 로직 (엔진 객체 생성)
    // ==========================================================

    // 1. TrapJ
    virtual bool trapj(const angles_t& goal, const angles_t& goal_v, const angles_t& pk_v, const angles_t& pk_a) {
        m_traj_state = TrajState::TRAPJ;
        m_trapj = std::make_unique<trajectory::TrajTrapJ>(m_model.get(), m_angles, m_angvels, goal, goal_v, pk_v, pk_a);
        return true;
    }

    // 2. AttrJ
    virtual bool attrj(const angles_t& goal, const angles_t& kp) {
        m_traj_state = TrajState::ATTRJ;
        m_attrj = std::make_unique<trajectory::TrajAttrJ>(m_model.get(), m_angles, m_angvels, goal, kp);
        return true;
    }

    // 3. AttrL (Cartesian)
    virtual bool attrl(const Eigen::Matrix4d& goal_tmat, value_t kp) {
        m_traj_state = TrajState::ATTRL;
        // 도구 끝점(m_tcp_tmat) 오프셋을 엔진에 전달
        m_attrl = std::make_unique<trajectory::TrajAttrL>(m_model.get(), m_angles, m_angvels, m_tcp_tmat.matrix(), goal_tmat, kp);
        return true;
    }

    // 4. 🌟 PlayJ (연속 궤적 재생)
    virtual bool playj(const Eigen::MatrixXd& q_set, 
                       const std::optional<Eigen::MatrixXd>& v_set = std::nullopt,
                       const std::optional<Eigen::MatrixXd>& a_set = std::nullopt) {
        m_traj_state = TrajState::PLAYJ;
        m_playj = std::make_unique<trajectory::TrajPlayJ>(m_model.get(), m_angles, m_angvels, q_set, v_set, a_set);
        return true;
    }

    // ==========================================================
    // --- 1ms 실시간 업데이트 로직 (Sim/Real 공통) ---
    // ==========================================================
    virtual void update_sim(double dt = 0.001) {
        switch (m_traj_state) {
            case TrajState::TRAPJ:
                if (m_trapj) {
                    m_trapj->update(dt);
                    m_angles = m_trapj->angles();
                    if (m_trapj->goal_reached()) m_traj_state = TrajState::STOP;
                }
                break;

            case TrajState::ATTRJ:
                if (m_attrj) {
                    m_attrj->update(dt, m_angles, m_angvels);
                }
                break;

            case TrajState::ATTRL:
                if (m_attrl) {
                    // 현재 각도와 속도, 그리고 설정된 TCP 정보를 사용하여 업데이트
                    m_attrl->update(dt, m_angles, m_angvels, m_tcp_tmat.matrix());
                }
                break;

            case TrajState::PLAYJ: // 🌟 PlayJ 업데이트 로직 추가
                if (m_playj) {
                    m_playj->update(dt);
                    m_angles = m_playj->angles();
                    m_angvels = m_playj->angvels();
                    if (m_playj->goal_reached()) m_traj_state = TrajState::STOP;
                }
                break;

            default:
                break;
        }
    }

    // Getters
    angles_t get_angles() const { return m_angles; }
    angles_t get_angvels() const { return m_angvels; }
    TrajState get_traj_state() const { return m_traj_state; }
};

} // namespace rt_control