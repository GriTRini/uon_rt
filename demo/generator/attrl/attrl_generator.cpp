#include <iostream>
#include <fstream>
#include <iomanip>
#include "../../../rt_control/model/model.hpp"
#include "../../../rt_control/trajectory/trajectory_generator.hpp"

using namespace rt_control;
using namespace rt_control::trajectory;

int main() {
    // 1. 모델 및 Generator 초기화 (m1013 전용 리밋 로드)
    rt_control::model::RobotModel model("m1013");
    
    // 초기 자세: J3=90, J5=90 (ㄱ자 포즈)
    angles_t q_start = angles_t::Zero();
    q_start(2) = 90.0;
    q_start(4) = 90.0;
    
    TrajGenerator traj_gen;
    traj_gen.initialize(model, q_start, angles_t::Zero(), angles_t::Zero());

    // 2. CSV 데이터 기록 설정
    std::ofstream csv("attrl_final_test.csv");
    csv << "Time,Step,TCP_Z,P_Err,J3_Acc,Kp\n";

    double dt = 0.001;
    double current_time = 0.0;
    int total_steps = 0;

    // 3. 목표 포즈 계산 (Z +0.1m 상승)
    Eigen::Isometry3d start_pose = traj_gen.tmat();
    Eigen::Isometry3d up_pose = start_pose;
    up_pose.translation().z() += 0.1; 

    // 통합 게인 설정 (상당히 강력한 500.0)
    double active_kp = 50.0; 
    traj_gen.set_combined_kp(active_kp);

    std::cout << "====================================================" << std::endl;
    std::cout << "🚀 AttrL 왕복 테스트 시작 (J3=90, J5=90)" << std::endl;
    std::cout << "   - 가속도 리밋: AttrL 전용 상향 모드 (10배)" << std::endl;
    std::cout << "   - 이동: Z축 100mm 상승 후 복귀" << std::endl;
    std::cout << "====================================================" << std::endl;

    // ---------------------------------------------------------
    // 🚀 STEP 1: 상승 기동 (UP)
    // ---------------------------------------------------------
    std::cout << "\n[Step 1] 상승 기동 (Z: " << start_pose.translation().z() 
              << " -> " << up_pose.translation().z() << ")" << std::endl;
    
    traj_gen.attrl(up_pose, active_kp);

    while (true) {
        traj_gen.update(dt);
        
        double p_err = (up_pose.translation() - traj_gen.tmat().translation()).norm();
        csv << current_time << ",UP," << traj_gen.tmat().translation().z() << "," 
            << p_err << "," << traj_gen.angaccs()(2) << "," << active_kp << "\n";

        if (total_steps % 250 == 0) {
            std::cout << "  (UP) T: " << std::fixed << std::setprecision(3) << current_time 
                      << "s | Z: " << traj_gen.tmat().translation().z() 
                      << " | P_Err: " << std::setprecision(5) << p_err << "m" << std::endl;
        }

        // 도달 판정 (2mm 이내 수렴 시)
        if (traj_gen.goal_reached(0.5, 0.002, 1.0, 1.0, 0.005, 5.0)) {
            std::cout << "✅ [Step 1] 도달 완료! 즉시 하강을 시작합니다." << std::endl;
            break;
        }

        current_time += dt;
        total_steps++;
        if (current_time > 5.0) { std::cerr << "❌ 타임아웃!"; return -1; }
    }

    // ---------------------------------------------------------
    // 🚀 STEP 2: 복귀 기동 (DOWN)
    // ---------------------------------------------------------
    std::cout << "\n[Step 2] 하강 복귀 시작 (Goal Z: " << start_pose.translation().z() << ")" << std::endl;
    traj_gen.attrl(start_pose, active_kp);

    while (true) {
        traj_gen.update(dt);

        double p_err = (start_pose.translation() - traj_gen.tmat().translation()).norm();
        csv << current_time << ",DOWN," << traj_gen.tmat().translation().z() << "," 
            << p_err << "," << traj_gen.angaccs()(2) << "," << active_kp << "\n";

        if (total_steps % 250 == 0) {
            std::cout << "  (DOWN) T: " << std::fixed << std::setprecision(3) << current_time 
                      << "s | Z: " << traj_gen.tmat().translation().z() 
                      << " | P_Err: " << std::setprecision(5) << p_err << "m" << std::endl;
        }

        if (traj_gen.goal_reached(0.5, 0.002, 1.0, 1.0, 0.005, 5.0)) {
            std::cout << "✅ [Step 2] 복귀 완료! 모든 테스트 성공." << std::endl;
            break;
        }

        current_time += dt;
        total_steps++;
        if (current_time > 10.0) break;
    }

    csv.close();
    std::cout << "\n📊 결과 저장: attrl_final_test.csv" << std::endl;
    return 0;
}