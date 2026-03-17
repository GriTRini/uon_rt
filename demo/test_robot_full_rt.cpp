#include <iostream>
#include <thread>
#include <chrono>
#include <Eigen/Dense>

#include "../rt_control/rt_robot.hpp"

using namespace rt_control;

int main() {
    Robot<0> robot("m1013", std::chrono::milliseconds(1));

    std::string robot_ip = "192.168.1.30";
    std::cout << "--- 두산 로봇 실시간 제어(RT) 풀 패키지 테스트 시작 ---" << std::endl;
    std::cout << "Target IP: " << robot_ip << std::endl;

    auto conn_ret = robot.open_connection(robot_ip);
    if (conn_ret != OpenConnError::NO_ERROR) {
        std::cerr << "[Fail] open_connection 에러 코드: " << (int)conn_ret << std::endl;
        return -1;
    }
    std::cout << "1. open_connection 성공" << std::endl;

    if (!robot.connect_rt(robot_ip)) {
        std::cerr << "[Fail] connect_rt 실패! (네트워크 설정 및 UDP 포트 확인 필요)" << std::endl;
        (void)robot.close_connection(); // 🌟 (void) 처리
        return -1;
    }
    std::cout << "2. connect_rt 성공" << std::endl;

    auto servo_ret = robot.servo_on();
    if (servo_ret != ServoOnError::NO_ERROR) {
        std::cerr << "[Fail] servo_on 실패! 에러 코드: " << (int)servo_ret << std::endl;
        robot.disconnect_rt();
        (void)robot.close_connection(); // 🌟 (void) 처리
        return -1;
    }
    std::cout << "3. servo_on 성공! 로봇 제어 루프가 가동 중입니다." << std::endl;

    try {
        auto start_q = robot.get_current_angles();
        if (start_q) {
            std::cout << "\n[Status] 현재 관절 각도: " << start_q->transpose() << std::endl;
            
            angles_t target_q = start_q.value();
            target_q(0) += 5.0; 

            std::cout << "[Action] 목표 지점으로 이동 시작 (+5 deg)..." << std::endl;
            (void)robot.trapj(target_q); // 🌟 (void) 처리

            if (robot.mwait(std::chrono::seconds(5))) {
                std::cout << "[Result] 목표 도달 완료!" << std::endl;
            } else {
                std::cout << "[Result] 도달 시간 초과" << std::endl;
            }

            std::this_thread::sleep_for(std::chrono::seconds(1));

            std::cout << "[Action] 원래 위치로 복귀 시작..." << std::endl;
            (void)robot.trapj(start_q.value()); // 🌟 (void) 처리
            (void)robot.mwait(std::chrono::seconds(5)); // 🌟 (void) 처리
        }
    } catch (const std::exception& e) {
        std::cerr << "런타임 예외 발생: " << e.what() << std::endl;
    }

    std::cout << "\n--- 테스트 종료 시퀀스 시작 ---" << std::endl;
    (void)robot.servo_off(); // 🌟 (void) 처리
    robot.disconnect_rt();
    (void)robot.close_connection(); // 🌟 (void) 처리
    std::cout << "모든 연결이 안전하게 해제되었습니다." << std::endl;

    return 0;
}
