#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <Eigen/Dense>
#include "../../../rt_control/rt_robot.hpp" 

using namespace rt_control;

const GPIO_CTRLBOX_DIGITAL_INDEX GRIPPER_DO_CH = GPIO_CTRLBOX_DIGITAL_INDEX_9;
std::atomic<bool> keep_running(true);
void sigint_handler(int signum) { keep_running = false; }

int main() {
    std::signal(SIGINT, sigint_handler);

    // 1. 로봇 초기화 (HCR14 모델 사용)
    Robot<0> robot("hcr14", std::chrono::milliseconds(1));
    const std::string robot_ip = "192.168.1.30";
    
    // TCP 설정 (그리퍼 오프셋)
    const double tx = -0.121, ty = -0.121, tz = 0.266;
    const double tr = 60.0, tp = 37.76, tyw = -135.0;

    if (robot.open_connection(robot_ip, 12345) != OpenConnError::NO_ERROR) return -1;
    if (!robot.connect_rt(robot_ip, 12347)) return -1;
    if (robot.servo_on(std::chrono::milliseconds(5000)) != ServoOnError::NO_ERROR) return -1;

    robot.set_tcp(tx, ty, tz, tr, tp, tyw);
    robot.set_digital_output(GRIPPER_DO_CH, false);

    // 대기 헬퍼 함수
    auto wait_until_done = [&](const std::string& task_name, int extra_wait_ms = 0) {
        auto next_wake_time = std::chrono::high_resolution_clock::now();
        while (keep_running) {
            if (robot.get_goal_reached()) break;
            next_wake_time += std::chrono::milliseconds(1);
            std::this_thread::sleep_until(next_wake_time);
        }
        for (int i = 0; i < extra_wait_ms; ++i) {
            if (!keep_running) break;
            next_wake_time += std::chrono::milliseconds(1);
            std::this_thread::sleep_until(next_wake_time);
        }
        std::cout << "   -> ✅ " << task_name << " 완료" << std::endl;
    };

    try {
        // --- [STEP 0] trapj를 이용한 홈 포지션 이동 ---
        std::cout << "\n🏠 Home Position으로 이동 (trapj)..." << std::endl;
        angles_t q_home;
        q_home << 0.0, 0.0, -90.0, 0.0, -90.0, 0.0; // 초기 각도 설정
        
        // trapj(목표각도, 속도비율, 가속도비율)
        if (robot.trapj(q_home, 0.5, 0.5)) { 
            wait_until_done("Home 이동");
        }

        while (keep_running) {
            std::cout << "\n🚀 시퀀스 시작 (Enter 입력 시 시작, Ctrl+C 종료) >> ";
            std::cin.get(); 

            // --- [STEP 1] 하강 (attrl) ---
            std::cout << "👇 하강 시작 (0.1m)..." << std::endl;
            
            // 현재의 TCP 좌표(Isometry3d) 가져오기
            Eigen::Isometry3d current_p = robot.get_task_pos(); 
            Eigen::Isometry3d down_p = current_p;
            down_p.translation().z() -= 0.1; // 100mm 아래

            // attrl(목표자세, 속도제한)
            if (robot.attrl(down_p, 0.1)) { // 0.1 m/s 속도로 하강
                wait_until_done("Down (attrl)");
            }

            // --- [STEP 2] 그리퍼 작동 ---
            std::cout << "✊ Gripping..." << std::endl;
            robot.set_digital_output(GRIPPER_DO_CH, true);
            wait_until_done("Grip wait", 1000); // 1초 대기

            // --- [STEP 3] 상승 (attrl) ---
            std::cout << "👆 상승 시작 (0.1m)..." << std::endl;
            Eigen::Isometry3d up_p = robot.get_task_pos();
            up_p.translation().z() += 0.1; // 다시 100mm 위로

            if (robot.attrl(up_p, 0.1)) {
                wait_until_done("Up (attrl)");
            }

            std::cout << "🎉 한 사이클 완료!" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "❌ 에러 발생: " << e.what() << std::endl;
    }

    // 종료 절차
    robot.set_digital_output(GRIPPER_DO_CH, false);
    robot.stop();
    robot.servo_off();
    robot.disconnect_rt();
    robot.close_connection();

    return 0;
}