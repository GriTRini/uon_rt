#include <iostream>
#include <thread>
#include <chrono>
#include <Eigen/Dense>

#include "../rt_control/rt_robot.hpp"

using namespace rt_control;

int main() {
    // 1. 로봇 객체 생성 (m1013 모델, 1ms 주기)
    Robot<0> robot("m1013", std::chrono::milliseconds(1));

    std::string robot_ip = "192.168.1.30";
    std::cout << "--- 두산 로봇 실시간 제어(RT) 풀 패키지 테스트 시작 ---" << std::endl;
    std::cout << "Target IP: " << robot_ip << std::endl;

    // 🌟 [Step 1] 일반 이더넷 연결 (포트 12345)
    auto conn_ret = robot.open_connection(robot_ip);
    if (conn_ret != OpenConnError::NO_ERROR) {
        std::cerr << "[Fail] open_connection 에러 코드: " << (int)conn_ret << std::endl;
        return -1;
    }
    std::cout << "1. open_connection 성공" << std::endl;

    // 🌟 [Step 2] 실시간 제어 채널 연결 (포트 12347)
    if (!robot.connect_rt(robot_ip)) {
        std::cerr << "[Fail] connect_rt 실패! (네트워크 설정 및 UDP 포트 확인 필요)" << std::endl;
        robot.close_connection();
        return -1;
    }
    std::cout << "2. connect_rt 성공" << std::endl;

    // 🌟 [Step 3] 서보 온 (이 시점부터 타이머 스레드가 구동되어 servoj_rt를 전송합니다)
    auto servo_ret = robot.servo_on();
    if (servo_ret != ServoOnError::NO_ERROR) {
        std::cerr << "[Fail] servo_on 실패! 에러 코드: " << (int)servo_ret << std::endl;
        robot.disconnect_rt();
        robot.close_connection();
        return -1;
    }
    std::cout << "3. servo_on 성공! 로봇 제어 루프가 가동 중입니다." << std::endl;

    try {
        // 현재 위치 읽기 (RT 채널을 통해 읽어옴)
        auto start_q = robot.get_current_angles();
        if (start_q) {
            std::cout << "\n[Status] 현재 관절 각도: " << start_q->transpose() << std::endl;
            
            // 테스트 동작: 1번 조인트만 +5도 이동 (안전을 위해 짧게 이동)
            angles_t target_q = start_q.value();
            target_q(0) += 5.0; 

            std::cout << "[Action] 목표 지점으로 이동 시작 (+5 deg)..." << std::endl;
            robot.trapj(target_q);

            // 도달 대기 (최대 5초)
            if (robot.mwait(std::chrono::seconds(5))) {
                std::cout << "[Result] 목표 도달 완료!" << std::endl;
            } else {
                std::cout << "[Result] 도달 시간 초과" << std::endl;
            }

            std::this_thread::sleep_for(std::chrono::seconds(1));

            // 원점 복귀
            std::cout << "[Action] 원래 위치로 복귀 시작..." << std::endl;
            robot.trapj(start_q.value());
            robot.mwait(std::chrono::seconds(5));
        }
    } catch (const std::exception& e) {
        std::cerr << "런타임 예외 발생: " << e.what() << std::endl;
    }

    // 종료 시퀀스
    std::cout << "\n--- 테스트 종료 시퀀스 시작 ---" << std::endl;
    robot.servo_off();
    robot.disconnect_rt();
    robot.close_connection();
    std::cout << "모든 연결이 안전하게 해제되었습니다." << std::endl;

    return 0;
}