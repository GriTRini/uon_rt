#pragma once
#include "inverse_solver.hpp"

namespace rt_control::trajectory::ik {

class IKSolver {
public:
    /**
     * @brief TrajAttrL에서 호출하는 최종 진입점
     * @param tcp_offset 추가: TCP를 고려한 IK를 풀기 위해 반드시 필요합니다.
     */
    static std::pair<rt_control::angles_t, bool> solve(
        const model::RobotModel* model,
        const Eigen::Isometry3d& target_pose,
        const rt_control::angles_t& seed_q_deg,
        const Eigen::Isometry3d& tcp_offset, // 🌟 인자 추가
        size_t step_max,
        double enorm_threshold,
        double damping) 
    {
        // 🌟 하위 함수인 solve_step_dls로 tcp_offset을 전달합니다.
        return solve_step_dls(model, target_pose, seed_q_deg, tcp_offset, step_max, enorm_threshold, damping);
    }
};

} // namespace rt_control::trajectory::ik