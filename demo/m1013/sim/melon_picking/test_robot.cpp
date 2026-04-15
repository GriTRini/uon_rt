#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>
#include "../../../../rt_control/model/model.hpp"
#include "../../../../rt_control/trajectory/trajectory_generator.hpp"

using namespace rt_control;
using namespace rt_control::trajectory;

int main() {
    // 1. 모델 및 제너레이터 초기화
    rt_control::model::RobotModel model("m1013");
    TrajGenerator traj_gen;
    traj_gen.initialize(model, angles_t::Zero(), angles_t::Zero(), angles_t::Zero());

    // 실제 물리 로봇 TCP 설정 (동일하게 유지)
    traj_gen.set_tcp(-0.029, 0.0, -0.3819, 0.0, 0.0, 0.0);

    // 2. CSV 헤더 확장 (J1~J6 모든 데이터 포함)
    std::ofstream csv("full_picking_log.csv");
    csv << "Time,Step";
    for(int i=1; i<=6; ++i) csv << ",J" << i << "_p"; // Positions
    for(int i=1; i<=6; ++i) csv << ",J" << i << "_v"; // Velocities
    for(int i=1; i<=6; ++i) csv << ",J" << i << "_a"; // Accelerations
    csv << ",TCP_X,TCP_Y,TCP_Z\n";

    double dt = 0.001;
    double current_time = 0.0;

    // --- 실행 및 전체 로깅용 람다 함수 ---
    auto execute_motion = [&](const std::string& step_name, double timeout = 10.0) {
        int loop_cnt = 0;
        double start_step_time = current_time;
        
        // 도달 판정: 위치(1mm), 각도(0.1도), 속도(0.5deg/s) 기준
        while (!traj_gen.goal_reached(0.1, 0.001, 1.0, 0.5)) {
            traj_gen.update(dt);
            
            // 10ms 주기로 CSV 기록 (데이터 해상도 유지)
            if (loop_cnt % 10 == 0) {
                csv << std::fixed << std::setprecision(5) << current_time << "," << step_name;
                
                // J1 ~ J6 모든 데이터 기록
                for(int j=0; j<6; ++j) csv << "," << traj_gen.angles()(j);
                for(int j=0; j<6; ++j) csv << "," << traj_gen.angvels()(j);
                for(int j=0; j<6; ++j) csv << "," << traj_gen.angaccs()(j);
                
                // TCP 좌표
                auto pos = traj_gen.tmat().translation();
                csv << "," << pos.x() << "," << pos.y() << "," << pos.z() << "\n";
            }
            
            current_time += dt;
            loop_cnt++;
            if (current_time - start_step_time > timeout) break;
        }

        // 🌟 모드 전환 전 정적 안정화 (E-Stop 방지 핵심)
        traj_gen.stop();
        for(int j=0; j<300; ++j) {
            traj_gen.update(dt);
            current_time += dt;
        }
        std::cout << ">> Completed: " << step_name << std::endl;
    };

    // --- 시나리오 좌표 설정 ---
    angles_t q_home; q_home << -90, 0, -90, 0, -90, 90;
    double pick_x = 0.0, pick_y = 1.1, pick_z = 0.045;
    double place_x = 0.0, place_y = 0.5, place_z = 0.045;

    std::cout << "🚀 Full Dynamic Picking Cycle Simulation Start..." << std::endl;

    // [Step 1] Home 이동
    (void)traj_gen.trapj(q_home);
    execute_motion("1_Go_Home");

    // [Step 2] Picking 접근
    Eigen::Isometry3d target = traj_gen.tmat();
    target.translation() << pick_x, pick_y, pick_z + 0.1;
    (void)traj_gen.attrl(target, 100.0);
    execute_motion("2_Approach_Pick");

    // [Step 3] Picking 하강
    target.translation() << pick_x, pick_y, pick_z;
    (void)traj_gen.attrl(target, 80.0);
    execute_motion("3_Picking");

    // [Step 4] Placing 위치 이동
    target.translation() << place_x, place_y, place_z;
    (void)traj_gen.attrl(target, 100.0);
    execute_motion("4_Moving_to_Place");

    // [Step 5] Home 복귀 (Attrl -> TrapJ 전환점)
    (void)traj_gen.trapj(q_home);
    execute_motion("5_Return_Home");

    csv.close();
    std::cout << "🏁 Logged to 'full_picking_log.csv'." << std::endl;
    return 0;
}