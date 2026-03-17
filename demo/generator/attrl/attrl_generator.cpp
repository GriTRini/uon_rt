#include <iostream>
#include <fstream>
#include <iomanip>
#include "../../../rt_control/model/model.hpp"
#include "../../../rt_control/trajectory/trajectory_generator.hpp"

using namespace rt_control;
using namespace rt_control::trajectory;

int main() {
    // 1. 모델 및 Generator 초기화
    rt_control::model::RobotModel model("m1013");
    angles_t q_start = angles_t::Zero();
    q_start(2) = 90.0; q_start(4) = 90.0; 
    
    TrajGenerator traj_gen;
    traj_gen.initialize(model, q_start, angles_t::Zero(), angles_t::Zero());

    // 2. CSV 설정
    std::ofstream csv("attrl_roundtrip_data.csv");
    csv << "Time,Step,TCP_Z,P_Err,V_Norm,Kp\n";

    double dt = 0.001;
    double current_time = 0.0;
    int total_step_count = 0;

    // 초기 포즈 저장 (나중에 복귀할 지점)
    Eigen::Isometry3d start_pose = traj_gen.tmat();
    Eigen::Isometry3d goal_pose = start_pose;
    goal_pose.translation().z() += 0.1; // 100mm UP

    // 통합 게인 설정
    double active_kp = 500.0; 
    traj_gen.set_combined_kp(active_kp);

    // ---------------------------------------------------------
    // 🚀 STEP 1: 100mm 들어 올리기 (UP)
    // ---------------------------------------------------------
    std::cout << "\n[Step 1] TCP 100mm 상승 시작 (Goal Z: " << goal_pose.translation().z() << ")" << std::endl;
    traj_gen.attrl(goal_pose, active_kp);

    // Reached가 될 때까지 무한 루프 (안전장치 5초)
    while (true) {
        traj_gen.update(dt);
        
        double p_err = (goal_pose.translation() - traj_gen.tmat().translation()).norm();
        csv << current_time << ",UP," << traj_gen.tmat().translation().z() << "," 
            << p_err << "," << traj_gen.angvels().norm() << "," << active_kp << "\n";

        if (total_step_count % 200 == 0) {
            std::cout << "  (UP) T: " << std::fixed << std::setprecision(3) << current_time 
                      << "s | Z: " << traj_gen.tmat().translation().z() 
                      << " | P_Err: " << std::setprecision(5) << p_err << "m" << std::endl;
        }

        // 🌟 Reached 판정 (인자값: q, pos, rot, dq, v, w)
        if (traj_gen.goal_reached(0.5, 0.002, 1.0, 1.0, 0.005, 5.0)) {
            std::cout << "✅ [Step 1] 상승 완료!" << std::endl;
            break;
        }

        current_time += dt;
        total_step_count++;

        if (current_time > 5.0) {
            std::cerr << "❌ [Step 1] 타임아웃! 도달 실패로 중단합니다." << std::endl;
            return -1;
        }
    }

    // ---------------------------------------------------------
    // 🚀 STEP 2: 원래 위치로 내리기 (DOWN)
    // ---------------------------------------------------------
    std::cout << "\n[Step 2] 원래 위치로 복귀 시작 (Goal Z: " << start_pose.translation().z() << ")" << std::endl;
    traj_gen.attrl(start_pose, active_kp);

    while (true) {
        traj_gen.update(dt);

        double p_err = (start_pose.translation() - traj_gen.tmat().translation()).norm();
        csv << current_time << ",DOWN," << traj_gen.tmat().translation().z() << "," 
            << p_err << "," << traj_gen.angvels().norm() << "," << active_kp << "\n";

        if (total_step_count % 200 == 0) {
            std::cout << "  (DOWN) T: " << std::fixed << std::setprecision(3) << current_time 
                      << "s | Z: " << traj_gen.tmat().translation().z() 
                      << " | P_Err: " << std::setprecision(5) << p_err << "m" << std::endl;
        }

        // 🌟 Reached 판정
        if (traj_gen.goal_reached(0.5, 0.002, 1.0, 1.0, 0.005, 5.0)) {
            std::cout << "✅ [Step 2] 복귀 완료!" << std::endl;
            break;
        }

        current_time += dt;
        total_step_count++;

        if (current_time > 10.0) { // 총 시간 기준
            std::cerr << "❌ [Step 2] 타임아웃! 도달 실패." << std::endl;
            break;
        }
    }

    csv.close();
    std::cout << "\n🎉 모든 기동이 성공적으로 완료되었습니다." << std::endl;
    return 0;
}