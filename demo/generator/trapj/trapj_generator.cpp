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
    
    // 시작 자세: 0, 0, 90, 0, 90, 0 (AttrL 테스트와 동일 자세)
    angles_t q_start = angles_t::Zero();
    q_start(2) = 90.0;
    q_start(4) = 90.0;

    TrajGenerator traj_gen;
    traj_gen.initialize(model, q_start, angles_t::Zero(), angles_t::Zero());

    // 2. CSV 파일 설정
    std::ofstream csv("trapj_reached_test.csv");
    csv << "Time,Step,J3_Pos,J3_Vel,J3_Acc\n";

    double dt = 0.001; // 1ms
    double current_time = 0.0;
    int step_count = 0;

    // ---------------------------------------------------------
    // 🚀 STEP 1: J3 -> 100도 이동 (전진)
    // ---------------------------------------------------------
    angles_t q_goal = q_start;
    q_goal(2) = 100.0; 
    
    std::cout << "\n🚀 [Step 1] TrapJ 전진 시작: J3 (90 -> 100)" << std::endl;
    traj_gen.trapj(q_goal);

    // 인자값 없이 호출하여 기본 임계값(0.1도 등) 테스트
    while (!traj_gen.goal_reached()) {
        traj_gen.update(dt);
        
        csv << current_time << ",FWD," << traj_gen.angles()(2) << "," 
            << traj_gen.angvels()(2) << "," << traj_gen.angaccs()(2) << "\n";
        
        if (step_count % 200 == 0) {
            std::cout << "[FWD] T: " << std::fixed << std::setprecision(3) << current_time 
                      << "s | J3_Pos: " << traj_gen.angles()(2) 
                      << " | Vel: " << traj_gen.angvels()(2) << std::endl;
        }
        current_time += dt;
        step_count++;

        if (current_time > 5.0) break; // 안전장치
    }
    std::cout << "✅ [Step 1] 도달 완료! 최종 J3: " << traj_gen.angles()(2) << std::endl;

    // ---------------------------------------------------------
    // 🚀 STEP 2: J3 -> 90도 복귀 (복귀)
    // ---------------------------------------------------------
    std::cout << "\n🚀 [Step 2] TrapJ 복귀 시작: J3 (100 -> 90)" << std::endl;
    traj_gen.trapj(q_start);

    while (!traj_gen.goal_reached()) {
        traj_gen.update(dt);
        
        csv << current_time << ",BCK," << traj_gen.angles()(2) << "," 
            << traj_gen.angvels()(2) << "," << traj_gen.angaccs()(2) << "\n";
        
        if (step_count % 200 == 0) {
            std::cout << "[BCK] T: " << std::fixed << std::setprecision(3) << current_time 
                      << "s | J3_Pos: " << traj_gen.angles()(2) 
                      << " | Vel: " << traj_gen.angvels()(2) << std::endl;
        }
        current_time += dt;
        step_count++;

        if (current_time > 10.0) break;
    }
    std::cout << "✅ [Step 2] 도달 완료! 최종 J3: " << traj_gen.angles()(2) << std::endl;

    csv.close();
    std::cout << "\n🏁 TrapJ 테스트 종료." << std::endl;

    return 0;
}