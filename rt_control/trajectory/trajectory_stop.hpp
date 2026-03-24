#pragma once

#include "trajectory_attrj.hpp"
#include "../model/model.hpp"

#include <cmath>

namespace rt_control {
namespace trajectory {

/**
 * @brief TrajStop 클래스
 * 현재 위치를 목표(Goal)로 설정하고, 강한 게인을 사용하여 
 * 스프링-댐퍼 시스템처럼 로봇을 급제동시키는 클래스입니다.
 */
class TrajStop : public TrajAttrJ {
  public:
    using Base = TrajAttrJ;
    using angles_t = rt_control::angles_t;
    using value_t = rt_control::value_t;

    // DSR 스타일의 정지 판단 임계값 (보통 0.1도 이내)
    constexpr static value_t STOP_KP = 100.0;
    constexpr static value_t ANGLES_ENORM_THOLD = 0.1; 
    constexpr static value_t ANGVELS_ENORM_THOLD = 0.5;

  public:
    TrajStop() = default;

    /**
     * @brief TrajGenerator에서 호출하는 DSR 스타일 생성자
     */
    TrajStop(const angles_t &start_angles,
             const angles_t &start_angvels,
             const angles_t &start_angaccs) noexcept
        // Base(TrajAttrJ) 생성자에 현재 상태와 기본 한계치들을 전달
        // 정지 모드이므로 min/max 값은 일반적인 로봇 스펙을 사용합니다.
        : Base(start_angles, start_angvels, start_angaccs,
               angles_t::Constant(-360.0), angles_t::Constant(360.0), // min/max angles
               angles_t::Constant(-250.0), angles_t::Constant(250.0), // min/max v
               angles_t::Constant(-500.0), angles_t::Constant(500.0)) // min/max a
    {
        // 현재 위치를 목표로 고정
        this->set_goal_angles(start_angles);
        this->set_goal_angvels(angles_t::Zero());
        
        // 정지를 위해 매우 높은 게인 설정 (Critical Damping은 내부에서 계산됨)
        this->set_kp(STOP_KP);
    }

  public:
    /**
     * @brief 목표 도달(정지 완료) 여부 확인
     * 위치 오차와 속도 오차가 모두 임계값 이내일 때 true 반환
     */
    [[nodiscard]] bool goal_reached() const noexcept {
        // TrajGenerator가 인지하는 'STOP' 상태로 전이하기 위한 조건
        value_t q_err = (this->angles() - this->goal_angles()).norm();
        value_t dq_err = this->angvels().norm();
        
        return (q_err <= ANGLES_ENORM_THOLD) && (dq_err <= ANGVELS_ENORM_THOLD);
    }
};

} // namespace trajectory
} // namespace rt_control