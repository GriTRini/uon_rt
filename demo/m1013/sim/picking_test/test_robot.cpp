#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>
#include <chrono>
#include <optional>
#include "../../../../rt_control/model/model.hpp"
#include "../../../../rt_control/trajectory/trajectory_generator.hpp"

using namespace rt_control;
using namespace rt_control::trajectory;

int main() {
    rt_control::model::RobotModel model("m1013");
    TrajGenerator traj_gen;

    // 초기화
    traj_gen.initialize(model, angles_t::Zero(), angles_t::Zero(), angles_t::Zero());

    // 🌟 물리 로봇 테스트와 완벽히 동일한 TCP 설정
    traj_gen.set_tcp(-0.1219, -0.1219, 0.26611, 60.0, 0.0, -45.0);

    // 🌟 속도(V), 가속도(A) 발산을 보기 위해 컬럼 대폭 추가
    std::ofstream csv("generator_diagnosis_log.csv");
    csv << "Time,StepInfo,"
        << "J1,J2,J3,J4,J5,J6,"
        << "V1,V2,V3,V4,V5,V6,"
        << "A1,A2,A3,A4,A5,A6,"
        << "TCP_X,TCP_Y,TCP_Z\n";

    double dt = 0.001; // 1ms 주기
    double current_time = 0.0;
    
    // 람다 함수: 제너레이터 업데이트 및 기록
    auto run_sim = [&](const std::string& step_info, double timeout = 10.0) {
        int loop_count = 0;
        
        while (!traj_gen.goal_reached(0.001, 0.01, 1.0, std::nullopt, std::nullopt, std::nullopt)) {
            traj_gen.update(dt); 
            
            angles_t cur_q = traj_gen.angles();
            angles_t cur_v = traj_gen.angvels(); // 각속도
            angles_t cur_a = traj_gen.angaccs(); // 각가속도
            Eigen::Isometry3d cur_tcp = traj_gen.tmat();

            // 10ms에 한 번씩만 출력 (터미널 버벅임 방지)
            if (loop_count % 10 == 0) {
                csv << std::fixed << std::setprecision(5) << current_time << "," << step_info << ",";
                for(int j=0; j<6; ++j) csv << cur_q(j) << ",";
                for(int j=0; j<6; ++j) csv << cur_v(j) << ",";
                for(int j=0; j<6; ++j) csv << cur_a(j) << ",";
                csv << cur_tcp.translation().x() << "," << cur_tcp.translation().y() << "," << cur_tcp.translation().z() << "\n";
            }

            current_time += dt;
            loop_count++;
            
            // 시뮬레이션 타임아웃 (무한루프 방지)
            if ((current_time) > timeout) {
                std::cout << "⚠️ [" << step_info << "] 타임아웃 발생!" << std::endl;
                break;
            }
        }
        std::cout << "✅ [" << step_info << "] 도달 완료. (소요 시간: " << current_time << "s)" << std::endl;
        
        // 동작 끝난 후 0.2초 안정화 (실제 로봇과 유사하게)
        for(int j=0; j<200; ++j) { traj_gen.update(dt); current_time += dt; }
    };

    std::cout << "🚀 진단용 시뮬레이션 시작..." << std::endl;

    // [Step 1] 초기 자세 (TrapJ)
    angles_t q_start = angles_t::Zero();
    q_start(0) = -90.0;
    q_start(2) = -90.0; 
    q_start(4) = -90.0;
    (void)traj_gen.trapj(q_start);
    run_sim("1_TrapJ_Init", current_time + 10.0);

    // [Step 2] 바닥 정렬
    (void)traj_gen.align_tcp_to_floor(90.0, 100.0);
    run_sim("2_Align_Floor", current_time + 10.0);

    // [Step 3] 문제의 좌표로 접근 (Z + 0.05m)
    double target_x = 0.0, target_y = 1.1, target_z = 0.045;
    
    Eigen::Isometry3d step3_target = traj_gen.tmat();
    step3_target.translation() << target_x, target_y, target_z + 0.05;
    
    std::cout << "▶ [접근] 목표 지점으로 Attrl 제어 시작..." << std::endl;
    // 💡 문제가 생겼던 150.0 게인 그대로 시뮬레이션
    (void)traj_gen.attrl(step3_target, 150.0); 
    run_sim("3_Approach", current_time + 10.0);

    // [Step 4] 하강
    Eigen::Isometry3d step4_target = traj_gen.tmat();
    step4_target.translation() << target_x, target_y, target_z;
    
    std::cout << "▶ [하강] 바닥으로 Attrl 제어 시작..." << std::endl;
    (void)traj_gen.attrl(step4_target, 150.0);
    run_sim("4_Reach", current_time + 10.0);

    csv.close();
    std::cout << "🏁 시뮬레이션 완료. 'generator_diagnosis_log.csv' 파일을 확인하세요." << std::endl;
    return 0;
}