#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "../rt_control/rt_robot.hpp"

using namespace rt_control;

int main() {
    // 1. 로봇 객체 생성 (m1013 모델)
    Robot<0> robot("m1013", std::chrono::milliseconds(1));
    std::string robot_ip = "192.168.1.30";

    if (robot.open_connection(robot_ip) != OpenConnError::NO_ERROR) return -1;
    if (!robot.connect_rt(robot_ip)) { (void)robot.close_connection(); return -1; }
    if (robot.servo_on() != ServoOnError::NO_ERROR) {
        (void)robot.disconnect_rt(); (void)robot.close_connection(); return -1;
    }

    // 🌟 AttrJ 설정: 목표 지점 정의
    angles_t q_zero = angles_t::Zero();
    angles_t q_step = angles_t::Zero(); 
    q_step(0) = 10.0; // 1번 조인트 10도

    std::vector<angles_t> sequence = {q_zero, q_step, q_zero};
    
    // Kp 값 설정 (수렴 속도 조절, 2.0 ~ 10.0 사이 추천)
    const double Kp = 7.0; 
    const int wait_counts = 500; // 500ms 대기

    std::cout << "--- AttrJ 실물 로봇 반복 테스트 시작 ---" << std::endl;

    try {
        for (int i = 0; i < 2; ++i) { // 2회 반복
            std::cout << "\n[반복 " << i + 1 << "회차]" << std::endl;

            for (const auto& target : sequence) {
                // 1. AttrJ 명령 하달 (비차단)
                if (!robot.attrj(target, Kp)) {
                    std::cerr << "AttrJ 생성 실패!" << std::endl;
                    goto cleanup;
                }

                // 2. goal_reached() 모니터링 루프
                // AttrJ는 점근적 수렴이므로 Tolerance 이내 진입 여부를 체크합니다.
                while (!robot.get_goal_reached()) {
                    auto curr_q = robot.get_current_angles();
                    if (curr_q) {
                        std::cout << "\r이동 중... Pos: " << (*curr_q)(0) << " deg" << std::flush;
                    }
                    std::this_thread::yield(); 
                }
                
                std::cout << "\n[도달] 대기 중 (Count base)..." << std::endl;

                // 3. Count 기반 비차단 대기
                int count = 0;
                while (count < wait_counts) {
                    // 대기 중에도 로봇 상태 실시간 감시 가능
                    count++;
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            }
        }
    } catch (...) {}

cleanup:
    std::cout << "\n테스트 종료. 서보 오프 및 연결 해제..." << std::endl;
    (void)robot.servo_off();
    (void)robot.disconnect_rt();
    (void)robot.close_connection();

    return 0;
}