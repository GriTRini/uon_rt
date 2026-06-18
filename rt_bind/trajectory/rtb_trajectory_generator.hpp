#pragma once

#include "../core/rtb_core.hpp"
#include <vector> // 추가

namespace rtb {
namespace trajectory {

using TrajState = rt_control::trajectory::TrajState;
using WaypointJ = rt_control::trajectory::WaypointJ; // 추가

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

    // ---------------------------------------------------------------------
    // 🌟 신규 추가: [Joint Space] PlayJ (시뮬레이션 용)
    // 파이썬 리스트(dict)를 파싱하는 부분은 Pybind 단에서 처리하고 여기선 파싱된 vector를 받음
    // ---------------------------------------------------------------------
    bool playj_py(const std::vector<WaypointJ>& waypoints,
                  const std::optional<py::array_t<double>>& user_peak_vels = std::nullopt,
                  const std::optional<py::array_t<double>>& user_peak_accs = std::nullopt,
                  double p_gain = 5.0) 
    {
        // 속도/가속도 한계가 입력되었다면 Eigen으로 매핑, 없으면 로봇 모델 최대치 사용
        rt_control::angles_t peak_vels = user_peak_vels.has_value() 
                                         ? Eigen::Map<const rt_control::angles_t, Eigen::Unaligned>(user_peak_vels->data()) 
                                         : m_model.get_max_angvels();
                                         
        rt_control::angles_t peak_accs = user_peak_accs.has_value() 
                                         ? Eigen::Map<const rt_control::angles_t, Eigen::Unaligned>(user_peak_accs->data()) 
                                         : m_model.get_max_angaccs();

        return Base::playj(waypoints, peak_vels, peak_accs, p_gain);
    }

    // 4. [Task Space] AttrL & Align
    bool attrl_py(const py::array_t<double>& goal_tmat, double attrl_kp = 50.0, double attrj_kp = 150.0, double target_speed = 0.20) {
        Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> map_t(goal_tmat.data());
        
        Eigen::Isometry3d target; 
        target.matrix() = map_t; 
        
        return Base::attrl(target, attrl_kp, attrj_kp, target_speed);
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