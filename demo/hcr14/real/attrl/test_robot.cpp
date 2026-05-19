#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <Eigen/Dense>

// 🌟 작성하신 RobotFactory 헤더 인클루드
#include "core/robot_factory.hpp"

std::atomic<bool> keep_running(true);
void sigint_handler(int signum) { keep_running = false; }

int main() {
    std::signal(SIGINT, sigint_handler);

    std::cout << "==========================================" << std::endl;
    std::cout << " [Hanwha HCR14] AttrL (T-mat) Sequence Test " << std::endl;
    std::cout << "==========================================\n" << std::endl;

    auto robot = rt_control::RobotFactory::create("hcr14");
    if (!robot) return -1;

    const std::string robot_ip = "192.168.1.132"; // 로봇 IP

    if (!robot->open_connection(robot_ip)) return -1;
    if (!robot->connect_rt(robot_ip)) return -1;
    if (!robot->servo_on()) return -1;

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // ==========================================================
    // 공통 대기(Monitoring) 함수
    // ==========================================================
    auto wait_goal = [&](const std::string& info, double p_th = 0.01) -> bool { 
        std::cout << "▶ [" << info << "] 이동 중..." << std::endl;
        while (keep_running) {
            if (robot->pop_alarm()) {
                std::cerr << "❌ 알람 발생! 동작 중지." << std::endl;
                return false;
            }
            if (robot->goal_reached(std::nullopt, p_th, std::nullopt, std::nullopt)) {
                std::cout << "\n✅ [" << info << "] 도달 완료!\n" << std::endl;
                return true;
            }
            
            auto cur_q = robot->get_current_angles();
            if (cur_q) {
                if (auto cur_tmat = robot->solve_forward(*cur_q)) {
                    std::cout << "\r   [TCP 위치] X: " << std::fixed << std::setprecision(3) << cur_tmat->translation().x()
                              << " Y: " << cur_tmat->translation().y() 
                              << " Z: " << cur_tmat->translation().z() << std::flush;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        return false;
    };

    // ==========================================================
    // [Step 1] 안전한 위치(Home)로 초기 이동 (TrapJ 활용)
    // ==========================================================
    rt_control::angles_t home_q = rt_control::angles_t::Zero();
    home_q(1) = -15.0; home_q(2) = -90.0; home_q(4) = -90.0;
    
    std::cout << "[Step 1] 기준이 될 Home 포인트로 이동합니다." << std::endl;
    if (robot->trapj(home_q)) {
        if (!robot->goal_reached(2.0, std::nullopt, std::nullopt, std::nullopt)) {
            while(keep_running && !robot->goal_reached(2.0, std::nullopt, std::nullopt, std::nullopt)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }
    std::cout << "✅ Home 안착 완료\n" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // ==========================================================
    // [Step 2] 현재 T-mat 복사
    // ==========================================================
    auto cur_q = robot->get_current_angles();
    if (!cur_q) goto END_TEST;
    
    // 현재 각도를 이용해 Forward Kinematics로 T-mat 획득
    Eigen::Isometry3d home_tmat;
    if (auto tmat_opt = robot->solve_forward(*cur_q)) {
        home_tmat = *tmat_opt;
    } else {
        std::cerr << "❌ T-mat(Forward Kinematics) 획득 실패" << std::endl;
        goto END_TEST;
    }

    double kp = 30.0; // TCP 제어 게인

    // ==========================================================
    // [Step 3] 시퀀스: X += 0.1 -> Home -> Y += 0.1
    // ==========================================================
    
    // 3-1. X축으로 +0.1m 이동
    {
        Eigen::Isometry3d target_tmat = home_tmat; // 홈 T-mat 복사
        target_tmat.translation().x() += 0.1;      // 🌟 X 좌표 + 10cm
        
        if (robot->attrl(target_tmat.matrix(), kp)) {
            if (!wait_goal("X축 +0.1m 이동", 0.01)) goto END_TEST;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // 3-2. 다시 홈 포인트(초기 T-mat)로 복귀
    {
        if (robot->attrl(home_tmat.matrix(), kp)) {
            if (!wait_goal("Home 복귀", 0.01)) goto END_TEST;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // 3-3. Y축으로 +0.1m 이동
    {
        Eigen::Isometry3d target_tmat = home_tmat; // 다시 홈 T-mat 복사
        target_tmat.translation().y() += 0.1;      // 🌟 Y 좌표 + 10cm
        
        if (robot->attrl(target_tmat.matrix(), kp)) {
            wait_goal("Y축 +0.1m 이동", 0.01);
        }
    }

END_TEST:
    robot->stop();
    robot->servo_off();
    robot->disconnect_rt();
    robot->close_connection();
    
    std::cout << "\n🏁 AttrL T-mat 시퀀스 테스트 종료." << std::endl;
    return 0;
}