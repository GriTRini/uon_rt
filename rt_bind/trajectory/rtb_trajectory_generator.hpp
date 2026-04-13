#pragma once

#include "../core/rtb_core.hpp"

namespace rtb {
namespace trajectory {

using TrajState = rt_control::trajectory::TrajState;

class TrajGenerator : public rt_control::trajectory::TrajGenerator {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Base = rt_control::trajectory::TrajGenerator;

    TrajGenerator() : Base() {}

    // 1. 초기화
    void initialize_py(const rt_control::model::RobotModel& model, 
                       const py::array_t<double>& q, const py::array_t<double>& dq, const py::array_t<double>& ddq) {
        Eigen::Map<const rt_control::angles_t, Eigen::Unaligned> m_q(q.data()), m_dq(dq.data()), m_ddq(ddq.data());
        Base::initialize(model, m_q, m_dq, m_ddq);
    }

    void update_py(double dt) { Base::update(static_cast<rt_control::value_t>(dt)); }
    void stop_py() { Base::stop(); }

    // 2. [Joint Space] TrapJ
    bool trapj_py(const py::array_t<double>& goal_q, const std::optional<py::array_t<double>>& goal_dq) {
        Eigen::Map<const rt_control::angles_t, Eigen::Unaligned> m_gq(goal_q.data());
        
        rt_control::angles_t m_gdq = rt_control::angles_t::Zero();
        if (goal_dq) {
            m_gdq = Eigen::Map<const rt_control::angles_t, Eigen::Unaligned>(goal_dq->data());
        }
        return Base::trapj(m_gq, m_gdq);
    }

    // 3. [Task Space] AttrL - tmat_t 대신 Eigen::Isometry3d 사용
    bool attrl_py(const py::array_t<double>& goal_tmat, double kp) {
        
        if (goal_tmat.size() != 16) throw std::runtime_error("goal_tmat must be 4x4");
        
        // NumPy 데이터를 Matrix4d로 매핑
        Eigen::Map<const Eigen::Matrix4d, Eigen::Unaligned> map_t(goal_tmat.data());
        
        // 🌟 명시적으로 Isometry3d 타입 사용
        Eigen::Isometry3d target_pose;
        target_pose.matrix() = map_t;

        // 엔진의 attrl 호출 (타입 변환이 필요할 수 있어 static_cast 추가)
        return Base::attrl(target_pose, static_cast<rt_control::value_t>(kp));
    }

    // --- Getters ---
    auto angles_py() const   { return rtb::to_pyarray(Base::angles()); }
    auto angvels_py() const  { return rtb::to_pyarray(Base::angvels()); }
    auto tmat_py() const     { return rtb::to_pyarray(Base::tmat()); }
    auto jmat_py() const     { return rtb::to_pyarray(Base::jmat()); }
    auto a_py() const        { return rtb::to_pyarray(Base::a()); }

    bool goal_reached_py(std::optional<double> q_th, std::optional<double> p_th, std::optional<double> r_th) const {
        return Base::goal_reached(q_th, p_th, r_th, std::nullopt, std::nullopt, std::nullopt);
    }
};

} // namespace trajectory
} // namespace rtb