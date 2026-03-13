#pragma once

#include "trajectory_attrj.hpp"
#include "../model/model.hpp"

#include <cmath>

namespace rt_control {
namespace trajectory {

class TrajStop : public TrajAttrJ {
  public:
    using Base = TrajAttrJ;
    using angles_t = rt_control::angles_t;
    using value_t = rt_control::value_t;

    // 🌟 급제동을 위한 매우 강한 Kp (스프링 상수)
    constexpr static value_t STOP_KP = 100.0;
    
    // 🌟 파이썬 API 기준(Degree)을 C++ 코어 연산용(Radian)으로 사전 변환
    constexpr static value_t ANGLES_ENORM_THOLD_RAD = 1.0 * (M_PI / 180.0);
    constexpr static value_t ANGVELS_ENORM_THOLD_RAD = 1.0 * (M_PI / 180.0);

  public:
    TrajStop() = default;

    TrajStop(const model::RobotModel* robot,
             const angles_t &start_angles,
             const angles_t &start_angvels,
             const angles_t &start_angaccs) noexcept
        // 급제동을 위해 사용자가 아닌 로봇 하드웨어가 가진 최대 스펙(max_vel, max_acc)을 끌어다 씁니다.
        : Base(robot, 
               start_angles, start_angvels, start_angaccs,
               robot->get_max_angvels(), robot->get_max_angaccs()) 
    {
        // AttrJ의 P게인을 100으로 설정하여 현재 위치로 강력하게 고정시킴
        Base::set_kp(angles_t::Constant(STOP_KP));
    }

  public:
    [[nodiscard]] bool goal_reached() const noexcept {
        // 위치 오차와 속도 오차가 모두 임계값(1도, 1도/s) 이내로 들어오면 "완전 정지"로 판정
        return (Base::angles_enorm() <= ANGLES_ENORM_THOLD_RAD) && 
               (Base::angvels_enorm() <= ANGVELS_ENORM_THOLD_RAD);
    }
};

} // namespace trajectory
} // namespace rt_control