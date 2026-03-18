#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include "../../../rt_control/model/model.hpp"
#include "../../../rt_control/trajectory/trajectory_generator.hpp"

using namespace rt_control;
using namespace rt_control::trajectory;

int main() {
    rt_control::model::RobotModel model("m1013");
    
    // 1. 시작 자세 및 목표 자세 시퀀스 설정 (6축 전체)
    angles_t q_start;  q_start << 0, 0, 90, 0, 90, 0;
    angles_t q_goal1; q_goal1 << 30, -20, 110, 45, 60, 90;
    angles_t q_goal2; q_goal2 << -30, 20, 70, -45, 120, -90;

    TrajGenerator traj_gen;
    traj_gen.initialize(model, q_start, angles_t::Zero(), angles_t::Zero());

    // CSV 파일 설정
    std::ofstream csv("attrj_debug_report.csv");
    csv << "Time,Step,J1_Pos,J2_Pos,J3_Pos,J4_Pos,J5_Pos,J6_Pos,Q_Err,DQ_Norm,Reached\n";

    double dt = 0.001; // 1ms
    double current_time = 0.0;
    
    std::vector<angles_t> targets = {q_goal1, q_goal2, q_start};
    std::vector<std::string> step_names = {"MOVE_1", "MOVE_2", "RETURN"};

    // 제어 게인 (Kp=10.0, 내부에서 Kd=2*sqrt(10) 자동 계산)
    double active_kp = 10.0; 

    std::cout << "===== 🛠️ AttrJ (Joint Space Attractor) Debug Start =====" << std::endl;

    for (size_t i = 0; i < targets.size(); ++i) {
        std::cout << "\n🚀 [Step " << i + 1 << "] " << step_names[i] << " 시작" << std::endl;
        
        // AttrJ 실행
        (void)traj_gen.attrj(targets[i], active_kp);

        double step_start_time = current_time;
        bool is_reached = false;
        int step_loop_cnt = 0;

        while (true) {
            traj_gen.update(dt);
            step_loop_cnt++;

            // 오차 계산
            double q_err = (targets[i] - traj_gen.angles()).norm();
            double dq_norm = traj_gen.angvels().norm();

            // 🌟 도착 판정 (임계값: 각도 오차 0.1도, 속도 0.5deg/s 이내)
            // attrl과 달리 Joint 임계값만 엄격하게 체크
            is_reached = traj_gen.goal_reached(std::nullopt, std::nullopt, 0.1, 0.5);

            // 데이터 기록
            csv << std::fixed << std::setprecision(6)
                << current_time << "," << i + 1 << "," 
                << traj_gen.angles()(0) << "," << traj_gen.angles()(1) << ","
                << traj_gen.angles()(2) << "," << traj_gen.angles()(3) << ","
                << traj_gen.angles()(4) << "," << traj_gen.angles()(5) << ","
                << q_err << "," << dq_norm << "," << (is_reached ? 1 : 0) << "\n";

            // 200ms 마다 모니터링 출력
            if (step_loop_cnt % 200 == 0) {
                std::cout << "Step: " << step_names[i] << " | T: " << current_time 
                          << "s | Q_Err: " << q_err << " | Vel: " << dq_norm << std::endl;
            }

            if (is_reached && step_loop_cnt > 100) {
                std::cout << "✅ SUCCESS: Step " << i + 1 << " 도달 완료!" << std::endl;
                break; 
            }

            current_time += dt;

            // 안전장치 (타임아웃 5초)
            if (current_time - step_start_time > 5.0) {
                std::cout << "⚠️ TIMEOUT: Step " << i + 1 << " 다음 단계 강제 전환" << std::endl;
                break;
            }
        }
        
        // 안정화를 위한 짧은 대기
        for(int j=0; j<100; ++j) { traj_gen.update(dt); current_time += dt; }
    }

    csv.close();
    std::cout << "\n🏁 테스트 종료. 'attrj_debug_report.csv' 확인 요망." << std::endl;
    return 0;
}