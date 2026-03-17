#include <iostream>
#include <fstream>
#include <iomanip>
#include "../../../rt_control/model/model.hpp"
#include "../../../rt_control/trajectory/trajectory_generator.hpp"

using namespace rt_control;
using namespace rt_control::trajectory;

int main() {
    // 1. 모델 초기화
    rt_control::model::RobotModel model("m1013");
    
    // 시작 자세: J3=90, J5=90
    angles_t q_start = angles_t::Zero();
    q_start(2) = 90.0;
    q_start(4) = 90.0;

    TrajGenerator traj_gen;
    traj_gen.initialize(model, q_start, angles_t::Zero(), angles_t::Zero());

    // 2. CSV 파일 설정
    std::ofstream csv("attrj_roundtrip_data.csv");
    csv << "Time,Step,J3_Pos,J3_Err,J3_Vel,Kp\n";

    double dt = 0.001;
    double current_time = 0.0;
    int total_step_count = 0;

    // 목표 자세 설정 (J3: 90 -> 110)
    angles_t q_goal = q_start;
    q_goal(2) = 110.0;

    // 통합 게인 설정 (AttrJ에서도 set_kp로 동작)
    double active_kp = 50.0; // AttrJ는 보통 낮은 게인에서도 부드럽게 작동함
    traj_gen.set_combined_kp(active_kp);

    // ---------------------------------------------------------
    // 🚀 STEP 1: J3 상승 (90 -> 110)
    // ---------------------------------------------------------
    std::cout << "\n[Step 1] AttrJ 상승 시작 (Goal J3: 110.0)" << std::endl;
    traj_gen.attrj(q_goal, active_kp);

    while (true) {
        traj_gen.update(dt);
        
        double j3_err = std::abs(q_goal(2) - traj_gen.angles()(2));
        csv << current_time << ",UP," << traj_gen.angles()(2) << "," 
            << j3_err << "," << traj_gen.angvels()(2) << "," << active_kp << "\n";

        if (total_step_count % 200 == 0) {
            std::cout << "  (UP) T: " << std::fixed << std::setprecision(3) << current_time 
                      << "s | J3: " << traj_gen.angles()(2) 
                      << " | Err: " << std::setprecision(5) << j3_err << "deg" << std::endl;
        }

        // 🌟 Reached 판정 (Joint 전용이므로 첫 번째 인자 q_thold가 중요)
        if (traj_gen.goal_reached(0.1, 0.002, 1.0, 0.5, 0.005, 5.0)) {
            std::cout << "✅ [Step 1] 상승 완료!" << std::endl;
            break;
        }

        current_time += dt;
        total_step_count++;

        if (current_time > 5.0) {
            std::cerr << "❌ [Step 1] 타임아웃!" << std::endl;
            return -1;
        }
    }

    // ---------------------------------------------------------
    // 🚀 STEP 2: J3 복귀 (110 -> 90)
    // ---------------------------------------------------------
    std::cout << "\n[Step 2] AttrJ 복귀 시작 (Goal J3: 90.0)" << std::endl;
    traj_gen.attrj(q_start, active_kp);

    while (true) {
        traj_gen.update(dt);

        double j3_err = std::abs(q_start(2) - traj_gen.angles()(2));
        csv << current_time << ",DOWN," << traj_gen.angles()(2) << "," 
            << j3_err << "," << traj_gen.angvels()(2) << "," << active_kp << "\n";

        if (total_step_count % 200 == 0) {
            std::cout << "  (DOWN) T: " << std::fixed << std::setprecision(3) << current_time 
                      << "s | J3: " << traj_gen.angles()(2) 
                      << " | Err: " << std::setprecision(5) << j3_err << "deg" << std::endl;
        }

        if (traj_gen.goal_reached(0.1, 0.002, 1.0, 0.5, 0.005, 5.0)) {
            std::cout << "✅ [Step 2] 복귀 완료!" << std::endl;
            break;
        }

        current_time += dt;
        total_step_count++;

        if (current_time > 10.0) break;
    }

    csv.close();
    std::cout << "\n🎉 AttrJ 왕복 기동 테스트 완료." << std::endl;
    return 0;
}