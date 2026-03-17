#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "../rt_control/rt_robot.hpp"

using namespace rt_control;

int main() {
    Robot<0> robot("m1013", std::chrono::milliseconds(1));
    std::string robot_ip = "192.168.1.30";

    if (robot.open_connection(robot_ip) != OpenConnError::NO_ERROR) return -1;
    if (!robot.connect_rt(robot_ip)) { (void)robot.close_connection(); return -1; }
    if (robot.servo_on() != ServoOnError::NO_ERROR) {
        (void)robot.disconnect_rt(); (void)robot.close_connection(); return -1;
    }

    // 목표 지점 설정
    angles_t q_zero = angles_t::Zero();
    angles_t q_step = angles_t::Zero(); q_step(0) = 10.0;
    std::vector<angles_t> sequence = {q_zero, q_step, q_zero};

    const int wait_counts = 500; // 1ms 주기 기준 500 count = 0.5초 대기

    try {
        for (int i = 0; i < 3; ++i) { // 3회 반복
            std::cout << "\n[반복 " << i + 1 << "회차]" << std::endl;

            for (const auto& target : sequence) {
                // 1. 명령 전달
                (void)robot.trapj(target);

                // 2. 이동 모니터링 (비차단)
                while (!robot.get_goal_reached()) {
                    // 🌟 실시간 처리가 필요한 다른 로직이 있다면 여기서 수행 가능
                    auto curr_q = robot.get_current_angles();
                    if (curr_q) {
                        std::cout << "\r이동 중... Pos: " << (*curr_q)(0) << std::flush;
                    }
                    // 최소한의 yield만 주거나, 실제 RT 루프라면 이조차도 없이 count 처리
                    std::this_thread::yield(); 
                }
                
                std::cout << "\n[도달] 대기 시작..." << std::endl;

                // 3. 🌟 Sleep 대신 Count 기반 비차단 대기
                int count = 0;
                while (count < wait_counts) {
                    // 대기 중에도 로봇 상태 체크 가능
                    count++;
                    std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 주기를 맞추기 위한 최소 sleep
                }
                std::cout << "[대기 완료] 다음 지점으로 이동" << std::endl;
            }
        }
    } catch (...) {}

    (void)robot.servo_off();
    (void)robot.disconnect_rt();
    (void)robot.close_connection();
    return 0;
}