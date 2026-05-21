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
        Eigen::Map<const rt_control::angles_t, Eigen::Unaligned> m_gq(goal_q.data());
        
        rt_control::angles_t m_gdq;
        if (goal_dq.has_value()) {
            m_gdq = Eigen::Map<const rt_control::angles_t, Eigen::Unaligned>(goal_dq->data());
        } else {
            m_gdq = rt_control::angles_t::Zero();
        }
        
        return Base::trapj(m_gq, m_gdq);
    }

    // 4. [Task Space] AttrL & Align (🌟 행렬 매핑 수정됨)
    bool attrl_py(const py::array_t<double>& goal_tmat, double kp = 50.0, double target_speed = 0.20) {
        // NumPy(RowMajor) 데이터를 Eigen이 오해하지 않도록 RowMajor 레이아웃을 명시합니다.
        Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> map_t(goal_tmat.data());
        
        // Isometry3d에 대입하면 Eigen 내부적으로 최적화된 ColumnMajor 형태로 변환됩니다.
        Eigen::Isometry3d target; 
        target.matrix() = map_t; 
        
        // 🌟 수정 포인트: 추가된 target_speed 파라미터를 Base::attrl에 함께 전달합니다.
        return Base::attrl(target, kp, target_speed);
    }

    bool align_tcp_to_floor_py(double yaw_deg, double kp) { return Base::align_tcp_to_floor(yaw_deg, kp); }
    bool align_tcp_to_front_py(double kp) { return Base::align_tcp_to_front(kp); }

    // 5. Getters
    auto angles_py() const      { return rtb::to_pyarray(Base::angles()); }
    auto tmat_py() const        { return rtb::to_pyarray(Base::tmat()); }
    auto flange_tmat_py() const { return rtb::to_pyarray(Base::flange_tmat()); }
    auto jmat_py() const        { return rtb::to_pyarray(Base::jmat()); }

    // 6. Error & Status Getters
    double angles_enorm_py() const { return Base::angles_enorm().value_or(-1.0); }
    double pos_enorm_py() const { return Base::pos_enorm().value_or(-1.0); }
    bool goal_reached_py(std::optional<double> q_th, std::optional<double> p_th, std::optional<double> r_th) const {
        return Base::goal_reached(q_th, p_th, r_th);
    }
};

} // namespace trajectory
} // namespace rtb