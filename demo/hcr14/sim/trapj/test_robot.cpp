#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <chrono>
#include "../../../../rt_control/model/model.hpp"
#include "../../../../rt_control/trajectory/trajectory_generator.hpp"

using namespace rt_control;
using namespace rt_control::trajectory;

int main() {
    // 1. HCR-14 로봇 모델 로드
    rt_control::model::RobotModel model("hcr14");
    
    // 🌟 한화 로봇 기본 홈 포지션 (J2 = 90도) 적용
    angles_t q_start;   q_start   << 0.0, -90.0, 90.0, 0.0, 0.0, 0.0;
    
    // 타겟 설정 (홈 포지션을 기준으로 충돌하지 않게 적절히 움직이도록 설정)
    angles_t q_target1; q_target1 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; 
    angles_t q_target2; q_target2 << 0.0, -90.0, 0.0, 0.0, 0.0, 0.0; 
    angles_t q_target3; q_target3 << 0.0, -90.0, 90.0, 0.0, 0.0, 0.0; // 다시 홈으로 복귀

    TrajGenerator traj_gen;
    traj_gen.initialize(model, q_start, angles_t::Zero(), angles_t::Zero());

    // 2. CSV 파일 오픈
    std::ofstream csv("hcr14_kinematics_data.csv");
    csv << "Time,J1,J2,J3,J4,J5,J6\n";

    double dt = 0.001; 
    double current_time = 0.0;
    std::vector<angles_t> test_goals = {q_target1, q_target2, q_target3};
    
    std::cout << "🚀 HCR-14 궤적 생성 및 CSV 로깅 시작..." << std::endl;
    std::cout << "🏠 Home Position: [" << q_start.transpose() << "]" << std::endl;

    for (size_t i = 0; i < test_goals.size(); ++i) {
        traj_gen.trapj(test_goals[i]);
        
        while (!traj_gen.goal_reached(0.1)) {
            traj_gen.update(dt); 
            angles_t cur_q = traj_gen.angles();

            // 데이터 기록
            csv << std::fixed << std::setprecision(5) << current_time << ",";
            for(int j=0; j<6; ++j) csv << cur_q(j) << (j==5 ? "" : ",");
            csv << "\n";

            current_time += dt;
        }
        // 안정화 대기
        for(int j=0; j<200; ++j) { traj_gen.update(dt); current_time += dt; }
    }

    csv.close();
    std::cout << "🏁 CSV 저장 완료! (hcr14_kinematics_data.csv)" << std::endl;
    return 0;
}