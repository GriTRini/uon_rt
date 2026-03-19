#include <iostream>
#include <vector>
#include <iomanip>
#include <fstream>
#include <chrono>
#include <thread>
#include <numeric>
#include <algorithm>
#include <csignal>
#include <atomic>
#include <Eigen/Dense>
#include "../../../rt_control/rt_robot.hpp" 

using namespace rt_control;

std::atomic<bool> keep_running(true);

void sigint_handler(int signum) {
    std::cout << "\n\n⚠️ [알림] 강제 종료 감지! 데이터를 저장하고 정지합니다...\n" << std::endl;
    keep_running = false;
}

int main() {
    std::signal(SIGINT, sigint_handler);

    std::cout << "===== 🚀 실시간 두산 로봇(m1013) ATTRL (Cartesian) 제어 벤치마킹 =====" << std::endl;
    
    Robot<0> robot("m1013", std::chrono::milliseconds(1));
    const std::string robot_ip = "192.168.1.30";

    std::ofstream csv_file("real_robot_attrl_data.csv");
    csv_file << "Time,Step,Target_X,Target_Y,Target_Z,Actual_X,Actual_Y,Actual_Z,Q_Error,Exec_us\n";

    if (robot.open_connection(robot_ip, 12345) != OpenConnError::NO_ERROR) return -1;
    if (!robot.connect_rt(robot_ip, 12347)) return -1;
    
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (robot.servo_on(std::chrono::milliseconds(5000)) != ServoOnError::NO_ERROR) return -1;

    robot.set_servoj_target_time(0.003f); 

    try {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
        // 🌟 1. 현재 로봇의 TCP 위치(T-Matrix) 가져오기
        // get_tcp_tmat()는 현재 제어기가 계산 중인 TCP 위치를 반환합니다.
        Eigen::Isometry3d T_current = robot.get_tcp_tmat();
        std::cout << "\n📍 현재 TCP 위치 (Z): " << T_current.translation().z() << " m" << std::endl;

        // 🌟 2. 목표 위치 설정 (Z축 -10cm, 다시 복귀)
        // 단위는 미터(m)이므로 10cm = 0.1입니다.
        Eigen::Isometry3d T_down = T_current;
        T_down.translation().x() -= 0.15; 

        Eigen::Isometry3d T_up = T_current;

        std::vector<Eigen::Isometry3d> test_goals = {T_down, T_up, T_down, T_up};
        double kp_cartesian = 40.0; // 🌟 작업 공간 어트랙터 강도 (보통 attrj보다 높게 설정)
        
        double dt = 0.001; 
        double total_time = 0.0;
        std::vector<double> exec_times;

        for (size_t i = 0; i < test_goals.size() && keep_running; ++i) {
            double target_x = test_goals[i].translation().x();
            std::cout << "\n📏 [ATTRL Step " << i + 1 << "] 이동 시작 -> Target X: " << target_x << " m" << std::endl;
            
            // 🌟 attrl 호출
            if (robot.attrl(test_goals[i], kp_cartesian)) {
                int loop_count = 0;
                auto next_wake_time = std::chrono::high_resolution_clock::now();

                while (!robot.get_goal_reached() && keep_running) {
                    auto start_tick = std::chrono::high_resolution_clock::now();
                    
                    auto cur_q = robot.get_current_angles();
                    // attrl 제어 중에도 제어기 내부의 목표 각도(Inverse Kinematics 결과)를 가져옵니다.
                    const auto& desired_q = robot.get_desired_angles(); 
                    
                    // 현재 실제 TCP 위치 계산 (Forward Kinematics 혹은 직접 읽기 필요할 수 있음)
                    // 여기서는 디버깅 편의를 위해 Q_Error를 관절 오차로 기록합니다.
                    if (cur_q.has_value()) {
                        const auto& actual_q = *cur_q;
                        double q_err = (desired_q - actual_q).norm();
                        
                        auto end_tick = std::chrono::high_resolution_clock::now();
                        double elapsed_us = std::chrono::duration<double, std::micro>(end_tick - start_tick).count();
                        exec_times.push_back(elapsed_us);

                        // CSV 기록: 이번에는 Cartesian 좌표 위주로 기록
                        // 실제 하드웨어의 실시간 TCP 좌표를 API에서 지원한다면 해당 값을 넣는 것이 좋습니다.
                        csv_file << std::fixed << std::setprecision(4) << total_time << "," << i + 1 << ","
                                 << test_goals[i].translation().x() << "," << test_goals[i].translation().y() << "," << target_x << ","
                                 << "0,0,0," << q_err << "," << elapsed_us << "\n"; // Actual X,Y,Z는 필요시 FK 연산 추가
                        csv_file.flush(); 

                        if (loop_count % 200 == 0) {
                            std::cout << "[RUN] T: " << std::fixed << std::setprecision(3) << total_time << "s | Err: " << q_err << std::endl;
                        }
                    }

                    next_wake_time += std::chrono::microseconds(1000);
                    std::this_thread::sleep_until(next_wake_time);
                    total_time += dt;
                    loop_count++;
                    
                    if (loop_count * dt > 10.0) break; 
                }
                
                std::cout << "✅ Step " << i + 1 << " 도달 완료." << std::endl;
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
    
    std::cout << "\n🏁 ATTRL 테스트 종료." << std::endl;
    return 0;
}