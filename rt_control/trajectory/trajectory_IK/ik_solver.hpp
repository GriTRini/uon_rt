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
    const Eigen::Isometry3d& tcp_offset = Eigen::Isometry3d::Identity(),
    size_t step_max = 500,           // 🌟 기본값 추가
    double enorm_threshold = 1e-6,   // 🌟 기본값 추가
    double damping = 0.01)           // 🌟 기본값 추가
    {
        return solve_step_qp(model, target_pose, seed_q_deg, tcp_offset, step_max, enorm_threshold, damping);
    }
};

} // namespace rt_control::trajectory::ik