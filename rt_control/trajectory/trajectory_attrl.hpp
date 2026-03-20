#pragma once
#include "trajectory_attrj.hpp"
#include "trajectory_planning/cartesian_attractor.hpp"
#include "trajectory_IK/ik_solver.hpp"

namespace rt_control::trajectory {

class TrajAttrL : public TrajAttrJ {
private:
    using Base = TrajAttrJ;

public:
    using angles_t = Base::angles_t;
    using value_t = Base::value_t;
    using tmat_t = Eigen::Isometry3d;

    TrajAttrL() = default;

    TrajAttrL(const model::RobotModel* model, const angles_t &q, const angles_t &dq, 
              const angles_t &ddq, const Eigen::Isometry3d& tcp_offset)
        : Base(model, q, dq, ddq, model->get_max_angvels(), model->get_max_angaccs(true)),
          m_model(model), m_tcp_offset(tcp_offset) {
        
        auto [initial_pose, J] = ik::compute_forward_and_jacobian(m_model, q, m_tcp_offset);
        m_tmat = initial_pose;
        // Attractor 초기화
        m_attractor.init(m_tmat, 40.0, 0.5, 1.0, 10.0, 2.0);
    }

    // 🌟 Generator에서 발생하는 에러 해결을 위한 함수명 통일
    void set_goal_pose(const tmat_t& goal) { m_attractor.goal_pose = goal; }
    void set_kp_cartesian(value_t kp) { m_attractor.kp = kp; }
    
    [[nodiscard]] const tmat_t& goal_pose() const noexcept { return m_attractor.goal_pose; }
    [[nodiscard]] const tmat_t& tmat() const noexcept { return m_tmat; }

    [[nodiscard]] bool update(const value_t &dt) noexcept override {
        if (dt <= 0.0) return false;
        m_attractor.update(dt);

        auto [new_q, converged] = ik::IKSolver::solve(m_model, m_attractor.pose, Base::angles(), 
                                                      m_tcp_offset, 20, 1e-4, 0.01);
        if (!converged) return false;

        Base::set_goal_angles(new_q);
        if (!Base::update(dt)) return false;

        auto [cp, J] = ik::compute_forward_and_jacobian(m_model, this->m_angles, m_tcp_offset);
        m_tmat = cp;
        return true;
    }

protected:
    const model::RobotModel* m_model = nullptr;
    planning::CartesianAttractor m_attractor;
    tmat_t m_tmat;
    tmat_t m_tcp_offset;
};

}