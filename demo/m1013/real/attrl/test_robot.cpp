#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <vector>
#include <Eigen/Dense>

// 🌟 새로 만든 팩토리 헤더를 인클루드합니다 (경로는 프로젝트 트리에 맞게 조정)
#include "../../../rt_control/core/robot_factory.hpp" 

// 🌟 네임스페이스를 새 아키텍처에 맞게 변경합니다
using namespace rt_control; 

// 전역 변수 및 시그널 핸들러 (Ctrl+C 종료용)
std::atomic<bool> keep_running(true);
void sigint_handler(int signum) { keep_running = false; }

int main() {
    std::signal(SIGINT, sigint_handler);

    // 1. 로봇 객체 동적 생성 (팩토리 패턴 사용)
    // "m1013" 입력 시 내부적으로 DsrRobot 객체가 생성되어 반환됩니다.
    std::unique_ptr<RobotBase> robot = RobotFactory::create("m1013");
    
    if (!robot) {
        std::cerr << "❌ 로봇 객체 생성 실패! 모델명을 확인하세요." << std::endl;
        return -1;
    }

    const std::string robot_ip = "192.168.1.30";
    
    // 🌟 이번 테스트는 Flange 기준이므로 TCP를 Identity(0)로 설정하거나 설정하지 않습니다.
    // 만약 그리퍼가 달려있다면 set_tcp를 호출하세요.
    // robot->set_tcp(0, 0, 0, 0, 0, 0); 

    // 2. 연결 및 서보 온 (포인터 방식 적용 및 bool 반환값 체크)
    std::cout << "로봇 연결 중..." << std::endl;
    if (!robot->open_connection(robot_ip, 12345)) {
        std::cerr << "❌ open_connection 실패" << std::endl;
        return -1;
    }
    if (!robot->connect_rt(robot_ip, 12347)) {
        std::cerr << "❌ connect_rt 실패" << std::endl;
        return -1;
    }
    if (!robot->servo_on()) {
        std::cerr << "❌ servo_on 실패" << std::endl;
        return -1;
    }
    std::cout << "✅ 로봇 연결 및 서보 온 완료!" << std::endl;

    // 동작 완료 대기 헬퍼 함수
    auto wait_until_done = [&](const std::string& task_name) {
        auto next_wake_time = std::chrono::high_resolution_clock::now();
        while (keep_running) {
            // goal_reached(조인트오차 0.1도, 위치오차 2mm, 회전오차 1도)
            if (robot->get_goal_reached(0.1, 0.002, 1.0)) break;
            
            next_wake_time += std::chrono::milliseconds(1);
            std::this_thread::sleep_until(next_wake_time);
        }
        std::cout << "   -> ✅ " << task_name << " 완료" << std::endl;
    };

    try {
        // --- [STEP 1] trapj를 이용한 시작 포지션 이동 ---
        std::cout << "\n📍 사각형 시작 지점으로 이동 (trapj)..." << std::endl;
        
        // angles_t 타입을 가져올 때 RobotBase 내부에 선언된 using을 활용
        RobotBase::angles_t q_start;
        q_start << -90.0, 0.0, -90.0, 0.0, -90.0, 0.0; 
        
        if (robot->trapj(q_start)) { 
            wait_until_done("시작 지점 도달");
        }

        if (keep_running) {
            std::cout << "\n🚀 네모 그리기 시퀀스를 시작합니다. (3초 후 시작)" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(3));

            // 🌟 현재 위치 가져오기 (get_task_pos -> get_current_pos)
            Eigen::Isometry3d start_pose = robot->get_current_pos();
            Eigen::Isometry3d target_pose = start_pose;

            // 사각형 꼭짓점 오프셋 (단위: m)
            // 200mm x 200mm 사각형 (파이썬 스타일 주석 #을 C++ 스타일 //로 수정)
            std::vector<Eigen::Vector3d> move_offsets = {
                {0.2, 0.0, 0.0},  // 1. 우측 이동
                {0.0, 0.2, 0.0},  // 2. 전방 이동
                {-0.2, 0.0, 0.0}, // 3. 좌측 이동
                {0.0, -0.2, 0.0}  // 4. 후방 이동 (원점 복귀)
            };

            for (size_t i = 0; i < move_offsets.size(); ++i) {
                if (!keep_running) break;

                // 목표 포즈 갱신
                target_pose.translation() += move_offsets[i];
                
                std::cout << "[" << i + 1 << "/4] 변 이동 중..." << std::endl;
                
                // attrl 실행 (속도 제한/제어 게인: 150.0)
                if (robot->attrl(target_pose, 150.0)) {
                    wait_until_done("변 " + std::to_string(i + 1));
                }
            }
        }

        std::cout << "\n🎉 모든 동작이 성공적으로 완료되었습니다." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "❌ 에러 발생: " << e.what() << std::endl;
    }

    // 종료 정리
    std::cout << "\n프로그램 종료 및 로봇 연결 해제..." << std::endl;
    robot->stop();
    robot->servo_off();
    robot->disconnect_rt();
    robot->close_connection();
    
    return 0;
}