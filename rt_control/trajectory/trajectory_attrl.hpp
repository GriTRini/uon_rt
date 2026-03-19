#pragma once

#include "trajectory_attrj.hpp"
#include "trajectory_planning/cartesian_attractor.hpp"
#include "trajectory_IK/ik_solver.hpp"

namespace rt_control {
namespace trajectory {

class TrajAttrL : public TrajAttrJ {
private:
    using Base = TrajAttrJ;

public:
    using angles_t = Base::angles_t;
    using value_t = Base::value_t;
    using tmat_t = Eigen::Isometry3d;

    constexpr static value_t IK_TOL = 1e-4;
    constexpr static value_t IK_DAMPING = 0.01;
    constexpr static size_t IK_MAX_ITER = 20;

public:
    TrajAttrL() = default;

    TrajAttrL(const model::RobotModel* model,
            const angles_t &q, const angles_t &dq, const angles_t &ddq)
        : Base(model, q, dq, ddq, 
            model->get_max_angvels(), 
            model->get_max_angaccs(true)),
        m_model(model)
    {
        m_tmat = m_model->forward_kinematics(q);
        double max_joint_vel = m_model->get_max_angvels().maxCoeff();
        double max_joint_acc = m_model->get_max_angaccs(true).maxCoeff();

        m_attractor.init(
            m_tmat, 
            50.0,
            0.5,
            max_joint_vel * (M_PI / 180.0),
            10.0,
            max_joint_acc * (M_PI / 180.0)
        );
        this->set_kp(angles_t::Constant(200.0));
    }

    void set_goal_pose(const tmat_t& goal) { 
        m_attractor.goal_pose = goal; 
    }

    void set_kp_cartesian(value_t kp) { 
        m_attractor.kp = kp; 
    }

    [[nodiscard]] bool update(const value_t &dt) noexcept override {
        if (dt <= 0.0) return false;
        m_attractor.update(dt);

        auto [new_q_deg, converged] = ik::IKSolver::solve(
            m_model, m_attractor.pose, Base::goal_angles(), 
            IK_MAX_ITER, IK_TOL, IK_DAMPING
        );

        if (!converged) {
            this->set_goal_angles(this->m_angles);
            return false;
        }

        Base::set_goal_angles(new_q_deg);
        if (!Base::update(dt)) return false;

        m_tmat = m_model->forward_kinematics(this->m_angles);
        return true;
    }

    // --- Getters (Generator 호환용) ---
    [[nodiscard]] const tmat_t& goal_pose() const noexcept { return m_attractor.goal_pose; }
    [[nodiscard]] const tmat_t& tmat() const noexcept { return m_tmat; }
    [[nodiscard]] const tmat_t& attractor_pose() const noexcept { return m_attractor.pose; }

protected:
    const model::RobotModel* m_model = nullptr;
    planning::CartesianAttractor m_attractor;
    tmat_t m_tmat;
};

} // namespace trajectory
} // namespace rt_control