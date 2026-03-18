#pragma once
#include "inverse_solver.hpp"

namespace rt_control::trajectory::ik {

class IKSolver {
public:
    /**
     * @brief TrajAttrL에서 호출하는 최종 진입점
     * 내부적으로 분할된 모듈들을 조합하여 최적의 해를 반환합니다.
     */
    static std::pair<rt_control::angles_t, bool> solve(
        const model::RobotModel* model,
        const Eigen::Isometry3d& target_pose,
        const rt_control::angles_t& seed_q_deg,
        size_t step_max,
        double enorm_threshold,
        double damping) 
    {
        // rbs_kinematics_solve_inverse_traj 처럼 
        // 이전 시점의 seed를 사용하여 연속성을 보장하며 해를 구함
        return solve_step_dls(model, target_pose, seed_q_deg, step_max, enorm_threshold, damping);
    }
};

} // namespace rt_control::trajectory::ik