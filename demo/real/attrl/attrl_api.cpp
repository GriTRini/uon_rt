#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <vector>
#include <Eigen/Dense>
#include "../../../rt_control/rt_robot.hpp" 

using namespace rt_control;

// 전역 변수 및 시그널 핸들러 (Ctrl+C 종료용)
std::atomic<bool> keep_running(true);
void sigint_handler(int signum) { keep_running = false; }

int main() {
    std::signal(SIGINT, sigint_handler);

    // 1. 로봇 객체 생성 (m1013 모델, 1ms 주기)
    Robot<0> robot("m1013", std::chrono::milliseconds(1));
    const std::string robot_ip = "192.168.1.30";
    
    // 🌟 이번 테스트는 Flange 기준이므로 TCP를 Identity(0)로 설정하거나 설정하지 않습니다.
    // 만약 그리퍼가 달려있다면 set_tcp를 호출하세요.
    // robot.set_tcp(0, 0, 0, 0, 0, 0); 

    // 연결 및 서보 온
    if (robot.open_connection(robot_ip, 12345) != OpenConnError::NO_ERROR) return -1;
    if (!robot.connect_rt(robot_ip, 12347)) return -1;
    if (robot.servo_on(std::chrono::milliseconds(5000)) != ServoOnError::NO_ERROR) return -1;

    // 동작 완료 대기 헬퍼 함수
    auto wait_until_done = [&](const std::string& task_name) {
        auto next_wake_time = std::chrono::high_resolution_clock::now();
        while (keep_running) {
            // goal_reached(조인트오차 0.1도, 위치오차 2mm, 회전오차 1도)
            if (robot.get_goal_reached(0.1, 0.002, 1.0)) break;
            
            next_wake_time += std::chrono::milliseconds(1);
            std::this_thread::sleep_until(next_wake_time);
        }
        std::cout << "   -> ✅ " << task_name << " 완료" << std::endl;
    };

    try {
        // --- [STEP 1] trapj를 이용한 시작 포지션 이동 ---
        std::cout << "\n📍 사각형 시작 지점으로 이동 (trapj)..." << std::endl;
        angles_t q_start;
        q_start << 0.0, 0.0, -90.0, 0.0, -90.0, 0.0; 
        
        if (robot.trapj(q_start)) { 
            wait_until_done("시작 지점 도달");
        }

        if (keep_running) {
            std::cout << "\n🚀 네모 그리기 시퀀스를 시작합니다. (3초 후 시작)" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(3));

            // 현재 위치(m_tmat) 가져오기
            Eigen::Isometry3d start_pose = robot.get_task_pos();
            Eigen::Isometry3d target_pose = start_pose;

            // 사각형 꼭짓점 오프셋 (단위: m)
            // 200mm x 200mm 사각형
            std::vector<Eigen::Vector3d> move_offsets = {
                {0.2, 0.0, 0.0},  # 1. 우측 이동
                {0.0, 0.2, 0.0},  # 2. 전방 이동
                {-0.2, 0.0, 0.0}, # 3. 좌측 이동
                {0.0, -0.2, 0.0}  # 4. 후방 이동 (원점 복귀)
            };

            for (size_t i = 0; i < move_offsets.size(); ++i) {
                if (!keep_running) break;

                // 목표 포즈 갱신
                target_pose.translation() += move_offsets[i];
                
                std::cout << "[" << i + 1 << "/4] 변 이동 중..." << std::endl;
                
                // attrl 실행 (속도 제한 0.1 m/s)
                if (robot.attrl(target_pose, 40.0)) {
                    wait_until_done("변 " + std::to_string(i + 1));
                }
            }
        }

        std::cout << "\n🎉 모든 동작이 성공적으로 완료되었습니다." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "❌ 에러 발생: " << e.what() << std::endl;
    }

    // 종료 정리
    robot.stop();
    robot.servo_off();
    robot.disconnect_rt();
    robot.close_connection();
    
    return 0;
}