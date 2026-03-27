#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>
#include <chrono>
#include <numeric>
#include <algorithm>
#include "../../../../rt_control/model/model.hpp"
#include "../../../../rt_control/trajectory/trajectory_generator.hpp"

using namespace rt_control;
using namespace rt_control::trajectory;

int main() {
    rt_control::model::RobotModel model("m1013");
    TrajGenerator traj_gen;

    // 🌟 1. 설정: 홈 포인트 (J2: -10, J3: -100, J5: -70)
    angles_t q_start = angles_t::Zero();
    angles_t q_home; 
    q_home << 0, -10, -100, 0, -70, 0;

    traj_gen.initialize(model, q_start, angles_t::Zero(), angles_t::Zero());

    std::ofstream csv("complex_sim_data.csv");
    csv << "Time,Step,J1,J2,J3,J4,J5,J6,Q_Error,Exec_us\n";

    double dt = 0.001; 
    double current_time = 0.0;
    
    // 람다 함수: 제너레이터 업데이트 및 기록
    auto run_sim = [&](int step_id) {
        int loop_count = 0;
        while (!traj_gen.goal_reached(0.01, std::nullopt, std::nullopt, 0.01)) {
            auto start_tick = std::chrono::high_resolution_clock::now();
            
            traj_gen.update(dt); 
            
            auto end_tick = std::chrono::high_resolution_clock::now();
            double elapsed_us = std::chrono::duration<double, std::micro>(end_tick - start_tick).count();

            angles_t cur_q = traj_gen.angles();
            double q_err = traj_gen.angles_enorm().value_or(0.0);

            csv << std::fixed << std::setprecision(4) << current_time << "," << step_id << ",";
            for(int j=0; j<6; ++j) csv << cur_q(j) << ",";
            csv << q_err << "," << elapsed_us << "\n";

            current_time += dt;
            loop_count++;
            if (loop_count > 10000) break; // 세이프티 타임아웃
        }
        // 정지 구간 (0.2초)
        for(int j=0; j<200; ++j) { traj_gen.update(dt); current_time += dt; }
    };

    std::cout << "🚀 시뮬레이션 시작..." << std::endl;

    // [Step 1] 홈 포인트로 이동 (trapj)
    std::cout << "[Step 1] Moving to Home..." << std::endl;
    traj_gen.trapj(q_home);
    run_sim(1);

    // [Step 2] Z축 15cm 내리기 (attrl)
    std::cout << "[Step 2] Attrl: Z -15cm..." << std::endl;
    Eigen::Isometry3d T_home = traj_gen.tmat();
    Eigen::Isometry3d T_down = T_home;
    T_down.translation().z() -= 0.5; // 15cm 내림
    traj_gen.attrl(T_down, 40.0);
    run_sim(2);

    // [Step 3] Z축 다시 올리기 (attrl)
    std::cout << "[Step 3] Attrl: Z Return..." << std::endl;
    traj_gen.attrl(T_home, 40.0);
    run_sim(3);

    // [Step 4] J1 90도 회전 (trapj)
    std::cout << "[Step 4] J1 Rotate 90..." << std::endl;
    angles_t q_rot = q_home;
    q_rot(0) = 90.0;
    traj_gen.trapj(q_rot);
    run_sim(4);

    // [Step 5] 회전된 자세에서 Z축 15cm 내리기 (attrl)
    std::cout << "[Step 5] Attrl: Z -15cm (Rotated)..." << std::endl;
    Eigen::Isometry3d T_rot = traj_gen.tmat();
    Eigen::Isometry3d T_rot_down = T_rot;
    T_rot_down.translation().z() -= 0.15;
    traj_gen.attrl(T_rot_down, 40.0);
    run_sim(5);

    // [Step 6] Z축 다시 올리기 (attrl)
    std::cout << "[Step 6] Attrl: Z Return (Rotated)..." << std::endl;
    traj_gen.attrl(T_rot, 40.0);
    run_sim(6);

    // [Step 7] 다시 홈 포인트로 복귀 (trapj)
    std::cout << "[Step 7] Final Return to Home..." << std::endl;
    traj_gen.trapj(q_home);
    run_sim(7);

    csv.close();
    std::cout << "🏁 시뮬레이션 완료. 'complex_sim_data.csv' 저장됨." << std::endl;
    return 0;
}