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
#include "../../../rt_control/rt_robot.hpp" 

using namespace rt_control;

// 🌟 강제 종료(Ctrl+C) 감지용 전역 플래그
std::atomic<bool> keep_running(true);

void sigint_handler(int signum) {
    std::cout << "\n\n⚠️ [알림] 강제 종료(Ctrl+C) 감지! 현재까지의 CSV 데이터를 저장하고 로봇을 안전하게 정지합니다...\n" << std::endl;
    keep_running = false;
}

int main() {
    // 🌟 Ctrl+C 시그널과 핸들러 연결
    std::signal(SIGINT, sigint_handler);

    std::cout << "===== 🚀 실시간 두산 로봇(m1013) RT 제어 벤치마킹 =====" << std::endl;
    std::cout << "Target Frequency: 1000 Hz (Cycle: 1000 us)" << std::endl;

    Robot<0> robot("m1013", std::chrono::milliseconds(1));
    const std::string robot_ip = "192.168.1.30";

    std::ofstream csv_file("real_robot_bench_data.csv");
    csv_file << "Time,Step,Target_J1,Target_J2,Target_J3,Target_J4,Target_J5,Target_J6,"
             << "Actual_J1,Actual_J2,Actual_J3,Actual_J4,Actual_J5,Actual_J6,Q_Error,Exec_us\n";

    if (robot.open_connection(robot_ip, 12345) != OpenConnError::NO_ERROR) return -1;
    if (!robot.connect_rt(robot_ip, 12347)) return -1;
    
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (robot.servo_on(std::chrono::milliseconds(5000)) != ServoOnError::NO_ERROR) return -1;

    robot.set_servoj_target_time(0.003f); // 응답성 최적화

    try {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        angles_t q_home = robot.get_current_angles().value_or(angles_t::Zero());
        
        angles_t q_target1 = q_home; q_target1(2) += 45.0; 
        angles_t q_target2 = q_home;                       
        angles_t q_target3 = q_home; q_target3(2) += 45.0; 

        std::vector<angles_t> test_goals = {q_target1, q_target2, q_target3};
        double dt = 0.001; 
        double total_time = 0.0;
        std::vector<double> exec_times;

        // 🌟 keep_running 플래그 확인 추가
        for (size_t i = 0; i < test_goals.size() && keep_running; ++i) {
            std::cout << "\n[Step " << i + 1 << "] 기동 시작 -> Target: ";
            for(int j=0; j<6; ++j) std::cout << std::setw(5) << test_goals[i](j) << " ";
            std::cout << std::endl;
            
            if (robot.trapj(test_goals[i])) {
                int loop_count = 0;
                auto next_wake_time = std::chrono::high_resolution_clock::now();

                // 🌟 keep_running 플래그 확인 추가
                while (!robot.get_goal_reached() && keep_running) {
                    auto start_tick = std::chrono::high_resolution_clock::now();

                    auto cur_q = robot.get_current_angles();
                    auto des_q = robot.get_desired_angles();
                    
                    auto end_tick = std::chrono::high_resolution_clock::now();
                    double elapsed_us = std::chrono::duration<double, std::micro>(end_tick - start_tick).count();
                    exec_times.push_back(elapsed_us);

                    if (cur_q.has_value()) { // des_q는 레퍼런스를 반환하므로 cur_q만 체크해도 무방
                        const auto& actual_q = *cur_q;      // 실제 로봇 값 (optional에서 꺼냄)
                        const auto& desired_q = robot.get_desired_angles(); // 계산기 값 (reference)

                        double q_err = (desired_q - actual_q).norm();

                        csv_file << std::fixed << std::setprecision(4) << total_time << "," << i + 1 << ",";
                        for(int j=0; j<6; ++j) csv_file << desired_q(j) << ","; // *des_q 대신 desired_q 사용
                        for(int j=0; j<6; ++j) csv_file << actual_q(j) << ",";
                        csv_file << q_err << "," << elapsed_us << "\n";
                        
                        // 🌟 핵심: 메모리에 남겨두지 않고 즉시 디스크(CSV)에 쓰기
                        csv_file.flush(); 

                        if (loop_count % 200 == 0) {
                            std::cout << "[RUN] T: " << std::fixed << std::setprecision(3) << total_time << "s | Q: [";
                            for(int j=0; j<6; ++j) {
                                std::cout << std::setw(7) << std::fixed << std::setprecision(2) << (*cur_q)(j) << (j==5 ? "" : ", ");
                            }
                            std::cout << "] | Err: " << std::setprecision(3) << q_err 
                                      << " | Exec: " << std::setw(5) << (int)elapsed_us << " us" << std::endl;
                        }
                    }

                    next_wake_time += std::chrono::microseconds(1000);
                    std::this_thread::sleep_until(next_wake_time);
                    
                    total_time += dt;
                    loop_count++;
                    
                    if (loop_count * dt > 15.0) break; // 15초 타임아웃
                }

                if (!exec_times.empty()) {
                    double sum = std::accumulate(exec_times.begin(), exec_times.end(), 0.0);
                    double avg = sum / exec_times.size();
                    double max_t = *std::max_element(exec_times.begin(), exec_times.end());
                    auto final_q = robot.get_current_angles().value_or(angles_t::Zero());
                    
                    std::cout << "--------------------------------------------------------------------------------" << std::endl;
                    std::cout << "✅ Step " << i + 1 << (keep_running ? " Done." : " 중단됨.") << " (Final Q: ";
                    for(int j=0; j<6; ++j) std::cout << std::fixed << std::setprecision(3) << final_q(j) << " ";
                    std::cout << ")" << std::endl;
                    std::cout << "📊 Stats: Avg Exec: " << std::fixed << std::setprecision(2) << avg 
                              << " us | Max: " << max_t << " us" << std::endl;
                    std::cout << "--------------------------------------------------------------------------------" << std::endl;
                }
                exec_times.clear();

                // Step 간 1초 대기 (이때도 1000Hz 유지)
                for (int cnt = 0; cnt < 1000 && keep_running; ++cnt) {
                    next_wake_time += std::chrono::microseconds(1000);
                    std::this_thread::sleep_until(next_wake_time);
                    total_time += dt;
                }
            }
        }
    } catch (const std::exception& e) { std::cerr << "Exception: " << e.what() << std::endl; }

    // 🌟 루프를 무사히 빠져나오거나 Ctrl+C로 탈출했을 때 항상 정상적으로 닫음
    csv_file.close();
    
    // 로봇 안전하게 연결 해제
    robot.stop(); // 궤적 생성기 중지
    (void)robot.servo_off();
    robot.disconnect_rt();
    (void)robot.close_connection();
    
    std::cout << "\n🏁 프로그램 종료. Data saved to 'real_robot_bench_data.csv'" << std::endl;
    return 0;
}