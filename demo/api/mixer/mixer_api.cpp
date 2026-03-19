#include <iostream>
#include <vector>
#include <iomanip>
#include <fstream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <Eigen/Dense>
#include "../../../rt_control/rt_robot.hpp" 

using namespace rt_control;

std::atomic<bool> keep_running(true);

void sigint_handler(int signum) {
    std::cout << "\n\n⚠️ [알림] 강제 종료 감지! 안전 정지 및 연결 해제 중...\n" << std::endl;
    keep_running = false;
}

int main() {
    std::signal(SIGINT, sigint_handler);

    // 🌟 사용자가 정의한 홈포인트 설정
    angles_t q_home = angles_t::Zero();
    q_home(2) = 90.0; // Joint 3 = 90 deg
    q_home(4) = 90.0; // Joint 5 = 90 deg

    std::cout << "===== 🚀 홈포인트(J3=90, J5=90) 기준 복합 제어 시퀀스 =====" << std::endl;
    
    Robot<0> robot("m1013", std::chrono::milliseconds(1));
    const std::string robot_ip = "192.168.1.30";

    std::ofstream csv_file("home_based_sequence_log.csv");
    csv_file << "Time,Step,Target_Info,Actual_J1,Actual_J2,Actual_J3,Actual_J4,Actual_J5,Actual_J6,Q_Error,Exec_us\n";

    if (robot.open_connection(robot_ip, 12345) != OpenConnError::NO_ERROR) return -1;
    if (!robot.connect_rt(robot_ip, 12347)) return -1;
    
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (robot.servo_on(std::chrono::milliseconds(5000)) != ServoOnError::NO_ERROR) return -1;

    robot.set_servoj_target_time(0.003f); 

    try {
        // --- 루프 유틸리티 함수 (제어 루프 실행 및 데이터 기록) ---
        auto run_until_reached = [&](int step_id, const std::string& info, double& total_time) {
            int loop_count = 0;
            auto next_wake_time = std::chrono::high_resolution_clock::now();
            while (!robot.get_goal_reached() && keep_running) {
                auto start_tick = std::chrono::high_resolution_clock::now();
                auto cur_q = robot.get_current_angles();
                
                if (cur_q.has_value()) {
                    const auto& actual_q = *cur_q;
                    const auto& desired_q = robot.get_desired_angles();
                    double q_err = (desired_q - actual_q).norm();
                    
                    auto end_tick = std::chrono::high_resolution_clock::now();
                    double elapsed_us = std::chrono::duration<double, std::micro>(end_tick - start_tick).count();

                    csv_file << std::fixed << std::setprecision(4) << total_time << "," << step_id << "," << info << ",";
                    for(int j=0; j<6; ++j) csv_file << actual_q(j) << ",";
                    csv_file << q_err << "," << elapsed_us << "\n";
                    csv_file.flush();
                    
                    if (loop_count % 500 == 0) {
                        std::cout << "[Step " << step_id << "] " << info << " | Q_Error: " << q_err << std::endl;
                    }
                }
                next_wake_time += std::chrono::microseconds(1000);
                std::this_thread::sleep_until(next_wake_time);
                total_time += 0.001;
                loop_count++;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        };

        double total_time = 0.0;

        // 🌟 [Step 1] 홈포인트(J3=90, J5=90)로 이동 (trapj)
        std::cout << "\n🏠 [Step 1] 홈포인트로 이동 시작..." << std::endl;
        if (robot.trapj(q_home)) run_until_reached(1, "Move_to_Home", total_time);

        // 🌟 [Step 2] 홈포인트 자세에서 Z축 -15cm 하강 후 복귀 (attrl)
        Eigen::Isometry3d T_home = robot.get_tcp_tmat();
        Eigen::Isometry3d T_down = T_home; 
        T_down.translation().z() -= 0.15; // 15cm 하강
        
        std::cout << "📏 [Step 2] 홈 기준 Z축 -15cm 하강 및 복귀" << std::endl;
        if (robot.attrl(T_down, 45.0)) run_until_reached(2, "Home_Z_Down", total_time);
        if (robot.attrl(T_home, 45.0)) run_until_reached(2, "Home_Z_Return", total_time);

        // 🌟 [Step 3] J1=90도 회전 (나머지는 홈포인트 유지)
        angles_t q_rotated = q_home;
        q_rotated(0) = 90.0; q_rotated(2) = 90.0; q_rotated(4) = 90.0; // J1=90, J3=90, J5=90 유지
        std::cout << "🔄 [Step 3] J1 90도 회전 (홈자세 유지)" << std::endl;
        if (robot.trapj(q_rotated)) run_until_reached(3, "J1_90_Rotate", total_time);

        // // 🌟 [Step 4] 회전된 위치에서 다시 Z축 -15cm 하강 후 복귀 (attrl)
        // Eigen::Isometry3d T_rot_base = robot.get_tcp_tmat();
        // Eigen::Isometry3d T_rot_down = T_rot_base; 
        // T_rot_down.translation().z() -= 0.15;

        // std::cout << "📏 [Step 4] 회전 위치에서 Z축 -15cm 하강 및 복귀" << std::endl;
        // if (robot.attrl(T_rot_down, 45.0)) run_until_reached(4, "Rot_Z_Down", total_time);
        // if (robot.attrl(T_rot_base, 45.0)) run_until_reached(4, "Rot_Z_Return", total_time);

        // // 🌟 [Step 5] 다시 처음 홈포인트(J3=90, J5=90, J1=0)로 최종 복귀
        // std::cout << "🏠 [Step 5] 최종 홈포인트 복귀" << std::endl;
        // if (robot.trapj(q_home)) run_until_reached(5, "Final_Home_Return", total_time);

    } catch (const std::exception& e) { std::cerr << "Error: " << e.what() << std::endl; }

    csv_file.close();
    robot.stop();
    (void)robot.servo_off();
    robot.disconnect_rt();
    (void)robot.close_connection();
    
    std::cout << "\n🏁 모든 시퀀스 완료. 'home_based_sequence_log.csv' 파일이 생성되었습니다." << std::endl;
    return 0;
}