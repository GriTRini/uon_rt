#include <iostream>
#include <fstream>
#include <iomanip>
#include <Eigen/Dense>

#include "../rt_control/model/model.hpp"
#include "../rt_control/trajectory/trajectory_generator.hpp"

using namespace rt_control;
using namespace rt_control::trajectory;

int main() {
    // 1. 모델 및 Generator 초기화
    rt_control::model::RobotModel model("m1013");
    angles_t q_start = angles_t::Zero(); // 초기 0도 자세
    
    rt_control::trajectory::TrajGenerator traj_gen;
    traj_gen.initialize(model, q_start, angles_t::Zero(), angles_t::Zero());

    // 2. CSV 기록 설정
    std::ofstream csv("/home/uon/uon_rt/demo/gen_integration_test.csv");
    csv << "Time,State,Joint_J0,TCP_Z,V_Norm\n";

    double current_time = 0.0;
    double dt = 0.001; 

    // ---------------------------------------------------------
    // 🚀 STEP 1: TRAPJ 테스트 (J0를 45도로 이동)
    // ---------------------------------------------------------
    angles_t q_goal_trap = angles_t::Zero();
    q_goal_trap(0) = 45.0; // 1번 축 45도 이동
    
    std::cout << "[Step 1] TRAPJ 시작 (J0: 0 -> 45deg)" << std::endl;
    traj_gen.trapj(q_goal_trap); 

    // TrapJ 루프 (도달 시까지)
    for(int i = 0; i < 2000; ++i) { 
        traj_gen.update(dt);
        
        csv << current_time << ",TRAPJ,"
            << traj_gen.angles()(0) << ","
            << traj_gen.tmat().translation().z() << ","
            << traj_gen.angvels().norm() << "\n";
        
        current_time += dt;
        
        // TrapJ는 각도 오차로 판단 (내부 goal_reached 로직 활용)
        if ((traj_gen.angles() - q_goal_trap).norm() < 0.01) {
            std::cout << "[Step 1] TRAPJ 도달 완료 (Time: " << current_time << "s)" << std::endl;
            break;
        }
    }

    // ---------------------------------------------------------
    // 🚀 STEP 2: ATTRL 테스트 (현재 위치에서 Z축 +10cm)
    // ---------------------------------------------------------
    Eigen::Isometry3d target_tmat = traj_gen.tmat();
    double target_z = target_tmat.translation().z() + 0.1;
    target_tmat.translation().z() = target_z;

    std::cout << "[Step 2] ATTRL 시작 (Z-Axis +100mm)" << std::endl;
    
    // Attrl 실행 (게인 200, 속도 0.5)
    if (!traj_gen.attrl(target_tmat, 200.0, 0.5)) {
        std::cerr << "Attrl 실행 실패!" << std::endl;
        return -1;
    }

    // Attrl 루프
    for(int i = 0; i < 2000; ++i) {
        traj_gen.update(dt);
        
        csv << current_time << ",ATTRL,"
            << traj_gen.angles()(0) << ","
            << traj_gen.tmat().translation().z() << ","
            << traj_gen.angvels().norm() << "\n";
        
        current_time += dt;

        if (traj_gen.goal_reached()) {
            std::cout << "[Step 2] ATTRL 도달 완료 (Time: " << current_time << "s)" << std::endl;
            break;
        }
    }

    csv.close();
    std::cout << "테스트 완료: gen_integration_test.csv 저장됨" << std::endl;

    return 0;
}