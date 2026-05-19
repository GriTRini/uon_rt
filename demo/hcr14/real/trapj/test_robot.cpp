#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <string>
#include <optional>

// 🌟 팩토리 대신 한화 로봇 전용 헤더를 직접 인클루드 합니다.
#include "../../../../rt_control/hcr/hcr_robot.hpp"

using namespace rt_control;

std::atomic<bool> keep_running(true);
void sigint_handler(int signum) { keep_running = false; }

int main() {
    std::signal(SIGINT, sigint_handler);

    std::cout << "==========================================" << std::endl;
    std::cout << " [Hanwha HCR14] TrapJ (Single Joint) Test " << std::endl;
    std::cout << "==========================================\n" << std::endl;

    // 🌟 Factory를 쓰지 않고 객체를 직접 생성 (포인터가 아닌 일반 객체)
    HcrRobot robot("hcr14", std::chrono::milliseconds(1));
    const std::string robot_ip = "192.168.100.200"; // 로봇 IP

    if (!robot.open_connection(robot_ip)) return -1;
    if (!robot.connect_rt(robot_ip)) return -1;
    if (!robot.servo_on()) return -1;

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // ==========================================================
    // 공통 대기(Monitoring) 함수
    // ==========================================================
    auto wait_goal = [&](const std::string& info, double q_th = 2.0) -> bool { 
        std::cout << "▶ [" << info << "] 이동 중..." << std::endl;
        while (keep_running) {
            if (robot.pop_alarm()) {
                std::cerr << "❌ 알람 발생! 동작 중지." << std::endl;
                return false;
            }
            if (robot.goal_reached(q_th, std::nullopt, std::nullopt, std::nullopt)) {
                std::cout << "\n✅ [" << info << "] 도달 완료!\n" << std::endl;
                return true;
            }
            
            if (auto cur_q = robot.get_current_angles()) {
                std::cout << "\r   [모니터링] J: [" << std::fixed << std::setprecision(2) << (*cur_q).transpose() << "]" << std::flush;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        return false;
    };

    // ==========================================================
    // [Step 1] 홈(Home) 포인트로 이동
    // ==========================================================
    angles_t home_q = angles_t::Zero();
    home_q(0) = 1.24;    // J1
    home_q(1) = -93.14;  // J2
    home_q(2) = -88.68;  // J3
    home_q(3) = -88.17;  // J4
    home_q(4) = 90.0;    // J5
    home_q(5) = -88.75;  // J6

    if (robot.trapj(home_q)) {
        if (!wait_goal("Home 이동", 2.0)) goto END_TEST;
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // ==========================================================
    // [Step 2] 홈 포인트에서 단일 관절(J1)만 움직임
    // ==========================================================
    {
        angles_t target_q = home_q; // 홈 각도 복사
        target_q(0) = 30.0; // 🌟 J1만 -90도로 변경 (다른 축은 유지)

        if (robot.trapj(target_q)) {
            wait_goal("J1 단일 관절 이동 (-90도)", 2.0);
        }
    }

END_TEST:
    std::cout << "\n🏁 안전 종료 시퀀스 시작..." << std::endl;

    // 1. 타이머 루프를 강제로 멈춥니다 (가장 중요)
    // RobotBase의 stop()이나 소멸자가 이 역할을 해야 합니다.
    robot.stop(); 
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // 2. 순서대로 안전하게 종료
    robot.servo_off();
    robot.disconnect_rt();
    robot.close_connection(); 
    
    std::cout << "🏁 TrapJ 단일 관절 테스트 정상 종료." << std::endl;
    
    // 3. 프로세스를 확실하게 종료시킵니다.
    std::exit(0);
}