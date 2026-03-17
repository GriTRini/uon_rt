#pragma once
#include "../core/core.hpp"
#include "../model/model.hpp"
#include <Eigen/Dense>

namespace rt_control {
namespace trajectory {

class TrajAttrJ {
  public:
    using angles_t = rt_control::angles_t;
    using value_t = rt_control::value_t;

  public:
    TrajAttrJ() = default;

    // 🌟 공식: 모델, 현재상태(q, dq, ddq), 한계치(v_lim, a_lim)
    TrajAttrJ(const model::RobotModel* robot,
              const angles_t &q, const angles_t &dq, const angles_t &ddq,
              const angles_t &max_v, const angles_t &max_a) noexcept
        : m_angles(q), m_angvels(dq), m_angaccs(ddq),
          m_goal_angles(q), m_goal_angvels(angles_t::Zero()),
          m_kp(angles_t::Constant(200.0)) 
    {
        m_min_angles = robot->get_min_angles();
        m_max_angles = robot->get_max_angles();
        m_max_angvels = max_v;
        m_max_angaccs = max_a;
        m_min_angvels = -m_max_angvels;
        m_min_angaccs = -m_max_angaccs;
        update_clip();
    }

    virtual bool update(const value_t &dt) noexcept {
        if (dt <= 0.0) return false;
        angles_t error = m_goal_angles - m_angles;
        angles_t target_vel = m_kp.cwiseProduct(error) + m_goal_angvels;
        target_vel = target_vel.cwiseMax(m_min_angvels).cwiseMin(m_max_angvels);
        
        angles_t target_acc = (target_vel - m_angvels) / dt;
        m_angaccs = target_acc.cwiseMax(m_min_angaccs).cwiseMin(m_max_angaccs);
        
        m_angvels += m_angaccs * dt;
        m_angles  += m_angvels * dt;
        update_clip();
        return true;
    }

    // Generator의 copy_state를 위한 Public Getters
    [[nodiscard]] const angles_t& angles() const noexcept { return m_angles; }
    [[nodiscard]] const angles_t& angvels() const noexcept { return m_angvels; }
    [[nodiscard]] const angles_t& angaccs() const noexcept { return m_angaccs; }
    [[nodiscard]] const angles_t& goal_angles() const noexcept { return m_goal_angles; }

    void set_goal_angles(const angles_t &new_goal) noexcept { m_goal_angles = new_goal.cwiseMax(m_min_angles).cwiseMin(m_max_angles); }
    void set_goal_angvels(const angles_t &new_goal_vel) noexcept { m_goal_angvels = new_goal_vel; }
    void set_kp(const angles_t &new_kp) noexcept { m_kp = new_kp; }

  protected:
    void update_clip() noexcept {
        m_angles = m_angles.cwiseMax(m_min_angles).cwiseMin(m_max_angles);
        m_angvels = m_angvels.cwiseMax(m_min_angvels).cwiseMin(m_max_angvels);
        m_angaccs = m_angaccs.cwiseMax(m_min_angaccs).cwiseMin(m_max_angaccs);
    }
    angles_t m_angles, m_angvels, m_angaccs, m_goal_angles, m_goal_angvels, m_kp;
    angles_t m_min_angles, m_max_angles, m_min_angvels, m_max_angvels, m_min_angaccs, m_max_angaccs;
};
}
}