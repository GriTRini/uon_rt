#pragma once

#include "../core/rtb_core.hpp"

namespace rtb {
namespace trajectory {

// 실무자 네임스페이스 별칭
using TrajState = rt_control::trajectory::TrajState;

class TrajGenerator : public rt_control::trajectory::TrajGenerator {
public:
    using Base = rt_control::trajectory::TrajGenerator;

    TrajGenerator() : Base() {}

    // 1. 초기화 (파이썬 배열 -> Eigen 캐스팅)
    void initialize_py(const rt_control::model::RobotModel& model,
                    const py::array_t<double>& q,
                    const py::array_t<double>& dq,
                    const py::array_t<double>& ddq) {
        
        std::cerr << "[Binding] Step 1: Checking input size..." << std::endl;
        if (q.size() != 6) throw std::runtime_error("Joint size must be 6");

        // 🌟 Eigen::Map을 사용하여 복사 없이 안전하게 데이터를 읽어옵니다.
        std::cerr << "[Binding] Step 2: Mapping data..." << std::endl;
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> m_q(q.data());
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> m_dq(dq.data());
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> m_ddq(ddq.data());

        std::cerr << "[Binding] Step 3: Calling C++ Base::initialize..." << std::endl;
        Base::initialize(model, m_q, m_dq, m_ddq);
        std::cerr << "[Binding] Step 4: Success!" << std::endl;
    }

    // 2. 데이터 Getters (Eigen -> NumPy 변환)
    auto angles_py() const   { return rtb::to_pyarray(Base::angles()); }
    auto angvels_py() const  { return rtb::to_pyarray(Base::angvels()); }
    auto tmat_py() const     { return rtb::to_pyarray(Base::tmat()); }
    auto jmat_py() const     { return rtb::to_pyarray(Base::jmat()); }

    // 3. 제어 명령 (trapj)
    bool trapj_py(const py::array_t<double>& goal_q, 
                  const std::optional<py::array_t<double>>& goal_dq = std::nullopt) {
        rt_control::angles_t dq = goal_dq ? goal_dq->cast<rt_control::angles_t>() : rt_control::angles_t::Zero();
        return Base::trapj(goal_q.cast<rt_control::angles_t>(), dq);
    }

    // 4. 목표 도달 확인
    bool goal_reached_py(std::optional<double> q_th, std::optional<double> p_th, std::optional<double> r_th) const {
        // 내부 기본값(2.0, 0.002, 3.0)을 유지하면서 null 처리
        return Base::goal_reached(q_th, p_th, r_th, std::nullopt, std::nullopt, std::nullopt);
    }
};

} // namespace trajectory
} // namespace rtb