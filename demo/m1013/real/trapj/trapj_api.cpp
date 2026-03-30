#include <iostream>
#include <vector>
#include <iomanip>
#include <fstream>
#include <chrono>
#include <thread>
#include <numeric>
#include <algorithm>
#include <csignal>   // 🌟 시그널 처리를 위해 추가
#include <atomic>    // 🌟 안전한 상태 플래그를 위해 추가
#include <Eigen/Dense>
#include "../../../rt_control/dsr/dsr_robot.hpp" 

using namespace dsr_control;

// 🌟 강제 종료(Ctrl+C) 감지용 전역 플래그
std::atomic<bool> keep_running(true);

void sigint_handler(int signum) {
    std::cout << "\n\n⚠️ [알림] 강제 종료(Ctrl+C) 감지! 현재까지의 CSV 데이터를 저장하고 로봇을 안전하게 정지합니다...\n" << std::endl;
    keep_running = false;
}

int main() {
    std::signal(SIGINT, sigint_handler);

    std::cout << "===== 🚀 실시간 두산 로봇(m1013) RT 제어 벤치마킹 (0도 정렬 포함) =====" << std::endl;
    
    Robot<0> robot("m1013", std::chrono::milliseconds(1));
    const std::string robot_ip = "192.168.1.30";

    std::ofstream csv_file("real_robot_bench_data.csv");
    csv_file << "Time,Step,Target_J1,Target_J2,Target_J3,Target_J4,Target_J5,Target_J6,"
             << "Actual_J1,Actual_J2,Actual_J3,Actual_J4,Actual_J5,Actual_J6,Q_Error,Exec_us\n";

    if (robot.open_connection(robot_ip, 12345) != OpenConnError::NO_ERROR) return -1;
    if (!robot.connect_rt(robot_ip, 12347)) return -1;
    
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (robot.servo_on(std::chrono::milliseconds(5000)) != ServoOnError::NO_ERROR) return -1;

    try {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // 🌟 1. 사전 단계: 모든 관절 0도로 정렬 (Step 0)
        angles_t q_zero = angles_t::Zero();
        std::cout << "\n🔄 [Step 0] 0도 정렬 시작 (기준점 확보)..." << std::endl;
        
        if (robot.trapj(q_zero)) {
            while (!robot.get_goal_reached() && keep_running) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            std::cout << "✅ 0도 정렬 완료. 잠시 후 메인 기동을 시작합니다." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // 🌟 2. 메인 타겟 설정 (이제 0도 기준이므로 q_home 대신 Zero() 사용 가능)
        angles_t q_home = angles_t::Zero(); 
        angles_t q_target1 = q_home; q_target1(2) += 60.0; q_target1(4) += 30.0; 
        angles_t q_target2 = q_home;                       
        angles_t q_target3 = q_home; q_target3(2) += 60.0; q_target3(4) += 30.0; 

        std::vector<angles_t> test_goals = {q_target1, q_target2, q_target3};
        double dt = 0.001; 
        double total_time = 0.0;
        std::vector<double> exec_times;

        // 🌟 3. 본격적인 벤치마킹 루프
        for (size_t i = 0; i < test_goals.size() && keep_running; ++i) {
            std::cout << "\n🚀 [Main Step " << i + 1 << "] 기동 시작 -> Target: " << test_goals[i].transpose() << std::endl;
            
            if (robot.trapj(test_goals[i])) {
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
                        exec_times.push_back(elapsed_us);

                        // CSV 기록 및 Flush
                        csv_file << std::fixed << std::setprecision(4) << total_time << "," << i + 1 << ",";
                        for(int j=0; j<6; ++j) csv_file << desired_q(j) << ",";
                        for(int j=0; j<6; ++j) csv_file << actual_q(j) << ",";
                        csv_file << q_err << "," << elapsed_us << "\n";
                        csv_file.flush(); 

                        if (loop_count % 200 == 0) {
                            std::cout << "[RUN] T: " << std::fixed << std::setprecision(3) << total_time << "s | Q: [";
                            for(int j=0; j<6; ++j) std::cout << std::setw(7) << actual_q(j) << (j==5 ? "" : ", ");
                            std::cout << "] | Err: " << std::setprecision(3) << q_err << std::endl;
                        }
                    }

                    next_wake_time += std::chrono::microseconds(1000);
                    std::this_thread::sleep_until(next_wake_time);
                    total_time += dt;
                    loop_count++;
                }
                
                // --- Step 완료 후 통계 및 1초 대기 로직 동일 ---
                if (!exec_times.empty()) {
                    double avg = std::accumulate(exec_times.begin(), exec_times.end(), 0.0) / exec_times.size();
                    std::cout << "✅ Step " << i + 1 << " Done. Avg Exec: " << avg << " us" << std::endl;
                }
                exec_times.clear();

                for (int cnt = 0; cnt < 1000 && keep_running; ++cnt) {
                    next_wake_time += std::chrono::microseconds(1000);
                    std::this_thread::sleep_until(next_wake_time);
                    total_time += dt;
                }
            }
        }
    } catch (const std::exception& e) { std::cerr << "Exception: " << e.what() << std::endl; }

    csv_file.close();
    robot.stop();
    (void)robot.servo_off();
    robot.disconnect_rt();
    (void)robot.close_connection();
    
    std::cout << "\n🏁 프로그램 종료." << std::endl;
    return 0;
}