#include <iostream>
#include <vector>
#include <chrono>
#include <Eigen/Dense>
#include "../rt_control/rt_robot.hpp"

using namespace rt_control;

int main() {
    // 1. 로봇 및 모델 초기화 (m1013, 1ms 주기)
    Robot<0> robot("m1013", std::chrono::milliseconds(1));
    std::string robot_ip = "192.168.1.30";

    if (robot.open_connection(robot_ip) != OpenConnError::NO_ERROR) return -1;
    if (!robot.connect_rt(robot_ip)) { (void)robot.close_connection(); return -1; }
    if (robot.servo_on() != ServoOnError::NO_ERROR) {
        (void)robot.disconnect_rt(); (void)robot.close_connection(); return -1;
    }

    // 제어 파라미터
    const double Kp_cartesian = 200.0;         // 충분한 추종 성능을 위한 게인
    const std::chrono::milliseconds stab_duration(500); 
    const std::chrono::seconds move_timeout(10); // 최대 이동 대기 시간

    try {
        // --- STEP 1: 초기 자세 이동 (J3: 90도) ---
        std::cout << "\n[Step 1] TrapJ 기동: J3=90도 자세로 이동..." << std::endl;
        angles_t q_ready = angles_t::Zero();
        q_ready(2) = 90.0;
        q_ready(4) = 90.0; // J5도 90도로 설정하여 보다 안정적인 초기 자세 확보

        if (robot.trapj(q_ready)) {
            while (!robot.get_goal_reached()) { std::this_thread::yield(); }
            
            // 타임스탬프 기반 비차단 안정화 대기
            auto start_time = std::chrono::steady_clock::now();
            while (std::chrono::steady_clock::now() - start_time < stab_duration) {
                std::this_thread::yield();
            }
            std::cout << ">> 초기 자세 안착 완료." << std::endl;
        }

        // --- STEP 2: Z축 +5cm 이동 (Attrl) ---
        // 🌟 수정: Y축 대신 Z축 방향으로 목표 설정
        Eigen::Isometry3d target_tmat = robot.traj_gen().tmat();
        double start_z = target_tmat.translation().z();
        double target_z = start_z + 0.1; // 현재 높이에서 +5cm (0.05m)
        target_tmat.translation().z() = target_z; 

        std::cout << "[Step 2] Attrl 기동: Z축 +5cm 직선 상승 시작..." << std::endl;
        std::cout << "Target Z: " << target_z << " m" << std::endl;
        
        if (robot.attrl(target_tmat, Kp_cartesian)) {
            auto move_start_time = std::chrono::steady_clock::now();
            
            while (!robot.get_goal_reached()) {
                auto current_pos = robot.traj_gen().tmat().translation();
                double error = std::abs(target_z - current_pos.z());
                
                // 실시간 위치 및 오차 출력
                std::cout << "\r현재 Z: " << current_pos.z() 
                          << " | 오차: " << error * 1000.0 << " mm" << std::flush;

                // 🌟 소프트웨어적 도달 판정 (1.5mm 이내 도달 시 통과)
                if (error < 0.0015) {
                    std::cout << "\n>> 목표 높이 도달 완료." << std::endl;
                    break;
                }

                // 타임아웃 체크
                if (std::chrono::steady_clock::now() - move_start_time > move_timeout) {
                    std::cout << "\n>> [경고] 이동 타임아웃 발생." << std::endl;
                    break;
                }
                
                std::this_thread::yield();
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "\n[에러 발생] " << e.what() << std::endl;
    }

    // 종료 절차
    std::cout << "\n테스트 종료. 안전하게 연결 해제 중..." << std::endl;
    (void)robot.servo_off();
    (void)robot.disconnect_rt();
    (void)robot.close_connection();

    return 0;
}