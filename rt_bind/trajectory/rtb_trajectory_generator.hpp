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

    // 1. 초기화 및 기본 업데이트
    void initialize_py(const rt_control::model::RobotModel& model, 
                       const py::array_t<double>& q, const py::array_t<double>& dq, const py::array_t<double>& ddq) {
        Eigen::Map<const rt_control::angles_t, Eigen::Unaligned> m_q(q.data()), m_dq(dq.data()), m_ddq(ddq.data());
        Base::initialize(model, m_q, m_dq, m_ddq);
    }
    void update_py(double dt) { Base::update(static_cast<rt_control::value_t>(dt)); }
    void stop_py() { Base::stop(); }

    // 2. TCP 설정
    void set_tcp_py(double x, double y, double z, double r, double p, double yaw) {
        Base::set_tcp(x, y, z, r, p, yaw);
    }

    // 3. [Joint Space] TrapJ
   bool trapj_py(const py::array_t<double>& goal_q, const std::optional<py::array_t<double>>& goal_dq) {
        // 목표 각도 매핑
        Eigen::Map<const rt_control::angles_t, Eigen::Unaligned> m_gq(goal_q.data());
        
        // 목표 속도 처리 (삼항 연산자 대신 if-else 사용)
        rt_control::angles_t m_gdq;
        if (goal_dq.has_value()) {
            // optional에 값이 있으면 NumPy 데이터를 매핑해서 할당
            m_gdq = Eigen::Map<const rt_control::angles_t, Eigen::Unaligned>(goal_dq->data());
        } else {
            // 값이 없으면 Zero() 할당
            m_gdq = rt_control::angles_t::Zero();
        }
        
        return Base::trapj(m_gq, m_gdq);
    }

    // 4. [Task Space] AttrL & Align
    bool attrl_py(const py::array_t<double>& goal_tmat, double kp) {
        Eigen::Map<const Eigen::Matrix4d, Eigen::Unaligned> map_t(goal_tmat.data());
        Eigen::Isometry3d target; target.matrix() = map_t;
        return Base::attrl(target, kp);
    }

    bool align_tcp_to_floor_py(double yaw_deg, double kp) { return Base::align_tcp_to_floor(yaw_deg, kp); }
    bool align_tcp_to_front_py(double kp) { return Base::align_tcp_to_front(kp); }

    // 5. Getters (Eigen -> NumPy)
    auto angles_py() const   { return rtb::to_pyarray(Base::angles()); }
    auto tmat_py() const     { return rtb::to_pyarray(Base::tmat()); }
    auto flange_tmat_py() const { return rtb::to_pyarray(Base::flange_tmat()); }
    auto jmat_py() const     { return rtb::to_pyarray(Base::jmat()); }

    // 6. Error & Status Getters
    double angles_enorm_py() const { return Base::angles_enorm().value_or(-1.0); }
    double pos_enorm_py() const { return Base::pos_enorm().value_or(-1.0); }
    bool goal_reached_py(std::optional<double> q_th, std::optional<double> p_th, std::optional<double> r_th) const {
        return Base::goal_reached(q_th, p_th, r_th);
    }
};

} // namespace trajectory
} // namespace rtb