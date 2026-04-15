#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <chrono>
#include <numeric>
#include <algorithm>
#include "../../../../rt_control/model/model.hpp"
#include "../../../../rt_control/trajectory/trajectory_generator.hpp"

using namespace rt_control;
using namespace rt_control::trajectory;

int main() {
    // 1. 모델 설정 (M1013)
    rt_control::model::RobotModel model("m1013");
    
    // 과격한 움직임을 위해 큰 변위 설정 (0도 -> 120도 급가속/급감속 유도)
    angles_t q_start;   q_start   << 0, 0, 0, 0, 0, 0;
    angles_t q_target1; q_target1 << 120, 45, 90, 150, 90, 150; 
    angles_t q_target2; q_target2 << -120, -45, -90, -150, -90, -150;
    angles_t q_target3; q_target3 << 0, 0, 0, 0, 0, 0;

    TrajGenerator traj_gen;
    traj_gen.initialize(model, q_start, angles_t::Zero(), angles_t::Zero());

    // 2. CSV 파일 생성 및 헤더 작성
    std::ofstream csv("joint_dynamics_test.csv");
    csv << "Time,Step";
    for(int i=1; i<=6; ++i) {
        csv << ",J" << i << "_Pos,J" << i << "_Vel,J" << i << "_Acc";
    }
    csv << ",Q_Error_Norm\n";

    double dt = 0.001; // 1ms 제어 주기
    double current_time = 0.0;
    std::vector<angles_t> test_goals = {q_target1, q_target2, q_target3};

    std::cout << "===== 🚀 Dynamic Trajectory Logging Start =====" << std::endl;

    for (size_t i = 0; i < test_goals.size(); ++i) {
        traj_gen.trapj(test_goals[i]);
        std::cout << "[Step " << i + 1 << "] Target: " << test_goals[i].transpose() << std::endl;

        // goal_reached 판정 기준 (각도 오차 0.05도 이하 등)
        while (!traj_gen.goal_reached(0.05, std::nullopt, std::nullopt, 0.1)) {
            // 궤적 업데이트 (내부적으로 m_angles, m_angvels, m_angaccs 갱신)
            traj_gen.update(dt); 

            // 데이터 수집
            const auto& q = traj_gen.angles();
            const auto& dq = traj_gen.angvels();
            const auto& ddq = traj_gen.angaccs();
            double q_err = traj_gen.angles_enorm().value_or(0.0);

            // CSV 기록
            csv << std::fixed << std::setprecision(5) << current_time << "," << i + 1;
            for(int j=0; j<6; ++j) {
                csv << "," << q(j) << "," << dq(j) << "," << ddq(j);
            }
            csv << "," << q_err << "\n";

            current_time += dt;

            // 안전 타임아웃
            if (current_time > 30.0) break; 
        }
        
        // 스텝 완료 후 약간의 휴지기 (안정화 데이터 수집)
        for(int j=0; j<100; ++j) {
            traj_gen.update(dt);
            current_time += dt;
        }
    }

    csv.close();
    std::cout << "🏁 Logging Finished. 'joint_dynamics_test.csv' saved." << std::endl;
    return 0;
}