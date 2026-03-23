#include <iostream>
#include <vector>
#include <iomanip>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <string>
#include "../../../rt_control/rt_robot.hpp" 

using namespace rt_control;

// 디지털 출력 9번 인덱스 설정
const GPIO_CTRLBOX_DIGITAL_INDEX GRIPPER_DO_CH = GPIO_CTRLBOX_DIGITAL_INDEX_9;

std::atomic<bool> keep_running(true);
void sigint_handler(int signum) { 
    keep_running = false; 
}

int main() {
    std::signal(SIGINT, sigint_handler);

    // [설정] 로봇 모델 및 IP (1ms 주기 설정)
    Robot<0> robot("hcr14", std::chrono::milliseconds(1));
    const std::string robot_ip = "192.168.1.30";
    
    const double tx = -0.121, ty = -0.121, tz = 0.266;
    const double tr = 60.0, tp = 37.76, tyw = -135.0;

    if (robot.open_connection(robot_ip, 12345) != OpenConnError::NO_ERROR) return -1;
    if (!robot.connect_rt(robot_ip, 12347)) return -1;
    if (robot.servo_on(std::chrono::milliseconds(5000)) != ServoOnError::NO_ERROR) return -1;

    robot.set_tcp(tx, ty, tz, tr, tp, tyw);
    robot.set_digital_output(GRIPPER_DO_CH, false);

    // 🌟 [변경] 카운트 기반 대기 헬퍼 함수
    // 목표 도달 후 지정된 ms만큼 루프를 돌며 대기합니다.
    auto wait_with_count = [&](const std::string& task_name, int wait_ms = 0) {
        auto next_wake_time = std::chrono::high_resolution_clock::now();
        
        // 1. 목표 도달 확인 루프
        while (keep_running) {
            if (robot.get_goal_reached()) break;
            
            next_wake_time += std::chrono::milliseconds(1);
            std::this_thread::sleep_until(next_wake_time);
        }

        // 2. 추가 카운트 대기 (그리퍼 작동 시간 등)
        for (int i = 0; i < wait_ms; ++i) {
            if (!keep_running) break;
            next_wake_time += std::chrono::milliseconds(1);
            std::this_thread::sleep_until(next_wake_time);
        }
        
        std::cout << "   -> ✅ " << task_name << " 완료" << std::endl;
    };

    try {
        while (keep_running) {
            double mm_x, mm_y, mm_z;
            
            std::cout << "\n📍 목표 좌표 입력 (mm) >> ";
            if (!(std::cin >> mm_x >> mm_y >> mm_z)) {
                std::cin.clear();
                std::cin.ignore(1000, '\n');
                continue;
            }

            double target_x = mm_x / 1000.0;
            double target_y = mm_y / 1000.0;
            double target_z = mm_z / 1000.0;

            // --- 시퀀스 시작 ---
            
            // 1. Approach (100mm 위)
            if (robot.movel(target_x, target_y, target_z + 0.1)) {
                wait_with_count("Approach");

                // 2. Down
                robot.movel(target_x, target_y, target_z);
                wait_with_count("Down");

                // 3. Grip (잡기 명령 후 1000카운트 대기)
                std::cout << "✊ 잡기 작동 중..." << std::endl;
                robot.set_digital_output(GRIPPER_DO_CH, true);
                wait_with_count("Gripping", 1000); // 1초 대기

                // 4. Lift Up
                robot.movel(target_x, target_y, target_z + 0.15);
                wait_with_count("Lift_Up");
                
                std::cout << "🎉 피킹 시퀀스 종료" << std::endl;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "❌ 에러: " << e.what() << std::endl;
    }

    // 종료 정리
    robot.set_digital_output(GRIPPER_DO_CH, false);
    robot.stop();
    (void)robot.servo_off();
    robot.disconnect_rt();
    (void)robot.close_connection();
    
    return 0;
}