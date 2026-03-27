#include <iostream>
#include <fstream>
#include <iomanip>
#include <Eigen/Dense>
#include "../../../../rt_control/model/model.hpp"
#include "../../../../rt_control/trajectory/trajectory_generator.hpp"

using namespace rt_control;
using namespace rt_control::trajectory;

int main() {
    model::RobotModel model("m1013");
    
    // 1. 초기 자세 (약간 굽힌 자세)
    angles_t q_curr;
    q_curr << 0.0, -45.0, 90.0, 0.0, 45.0, 0.0; 
    
    // 현재 TCP 위치 확인 (FK)
    Eigen::Isometry3d start_pose = model.forward_kinematics(q_curr);
    
    // 2. 가상의 목표점 설정 (현재 위치에서 X+10cm, Z+10cm 떨어진 곳)
    Eigen::Isometry3d target_pose = start_pose;
    target_pose.translation().x() += 0.03;
    target_pose.translation().z() += 0.05;

    std::ofstream csv("ik_convergence_test.csv");
    csv << "Iteration,X,Y,Z,Goal_X,Goal_Y,Goal_Z,P_Err,R_Err,Q1,Q2,Q3,Q4,Q5,Q6\n";

    std::cout << "===== 🔍 IK Convergence Unit Test Start =====" << std::endl;
    std::cout << "Target Pose X: " << target_pose.translation().x() << " Z: " << target_pose.translation().z() << std::endl;

    // 3. IK 파라미터 설정 (테스트용)
    size_t max_iter = 50;        // 수렴 과정을 보기 위해 크게 설정
    double threshold = 1e-6;     // 매우 정밀한 타겟
    double damping = 0.01;       // 작은 댐핑부터 시작

    angles_t q_sol = q_curr; // Seed

    for (size_t i = 0; i < max_iter; ++i) {
        // 단일 스텝 IK 수행 (내부 루프를 1회씩 돌리며 상태 관찰)
        auto [next_q, converged] = ik::IKSolver::solve(&model, target_pose, q_sol, 1, threshold, damping);
        q_sol = next_q;

        // 현재 계산된 결과의 포즈 확인
        Eigen::Isometry3d curr_pose = model.forward_kinematics(q_sol);
        double p_err = (target_pose.translation() - curr_pose.translation()).norm();
        Eigen::AngleAxisd r_err_aa(target_pose.linear() * curr_pose.linear().transpose());
        double r_err = std::abs(r_err_aa.angle()) * (180.0 / M_PI);

        // CSV 기록
        csv << i << "," 
            << curr_pose.translation().x() << "," << curr_pose.translation().y() << "," << curr_pose.translation().z() << ","
            << target_pose.translation().x() << "," << target_pose.translation().y() << "," << target_pose.translation().z() << ","
            << p_err << "," << r_err << ","
            << q_sol(0) << "," << q_sol(1) << "," << q_sol(2) << "," 
            << q_sol(3) << "," << q_sol(4) << "," << q_sol(5) << "\n";

        if (i % 5 == 0) {
            std::cout << "Iter " << i << " | P_Err: " << p_err << "m | R_Err: " << r_err << "deg" << std::endl;
        }

        if (p_err < threshold && r_err < 1e-3) {
            std::cout << "✅ Converged at iteration " << i << "!" << std::endl;
            break;
        }
    }

    csv.close();
    return 0;
}