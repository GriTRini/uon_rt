#include <iostream>
#include <vector>
#include <iomanip>
#include <fstream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include "../../../rt_control/rt_robot.hpp" 

using namespace rt_control;

std::atomic<bool> keep_running(true);
void sigint_handler(int signum) { keep_running = false; }

int main() {
    std::signal(SIGINT, sigint_handler);

    // [설정] 로봇 모델 및 IP
    Robot<0> robot("m1013", std::chrono::milliseconds(1));
    const std::string robot_ip = "192.168.1.30";
    
    // [설정] 테스트용 TCP 오프셋 (Roll 30도 꺾임)
    const double tx = -0.1218, ty = -0.1218, tz = 0.2661;
    const double tr = 30.0, tp = 0.0, tyw = 0.0;

    // [파일] 관절 정보 저장용 CSV
    std::ofstream csv("tcp_sequence_test_log.csv");
    csv << "Time,Step,J1,J2,J3,J4,J5,J6,TCP_X,TCP_Y,TCP_Z\n";

    if (robot.open_connection(robot_ip, 12345) != OpenConnError::NO_ERROR) return -1;
    if (!robot.connect_rt(robot_ip, 12347)) return -1;
    if (robot.servo_on(std::chrono::milliseconds(5000)) != ServoOnError::NO_ERROR) return -1;

    double total_time = 0.0;

    // 🌟 실행 및 로깅 헬퍼 함수
    auto run_until_reached = [&](const std::string& info, int stabilize_count = 500) {
        auto next_wake_time = std::chrono::high_resolution_clock::now();
        int loop_count = 0;
        
        while (keep_running) {
            bool reached = robot.get_goal_reached();
            auto cur_q = robot.get_current_angles();
            auto cur_tcp = robot.get_tcp_tmat();

            if (cur_q) {
                csv << std::fixed << std::setprecision(5) << total_time << "," << info << ",";
                for(int i=0; i<6; ++i) csv << (*cur_q)(i) << ",";
                csv << cur_tcp.translation().x() << "," << cur_tcp.translation().y() << "," << cur_tcp.translation().z() << "\n";
            }

            if (loop_count % 500 == 0) {
                std::cout << std::fixed << std::setprecision(4)
                          << "[T: " << std::setw(6) << total_time << "s] " << std::setw(18) << info 
                          << " | J: [" << (*cur_q).transpose() << "]" << std::endl;
            }

            if (reached && stabilize_count-- <= 0) break;

            next_wake_time += std::chrono::milliseconds(1);
            std::this_thread::sleep_until(next_wake_time);
            total_time += 0.001;
            loop_count++;
        }
        std::cout << "      ✅ " << info << " 완료\n" << std::endl;
    };

    // 적용 대기 시간 헬퍼
    auto wait_for_apply = [&]() {
        for(int i=0; i<300; ++i) { 
            std::this_thread::sleep_for(std::chrono::milliseconds(1)); 
            total_time += 0.001; 
        }
    };

    try {
        // ==========================================================
        // --- [Step 0] 초기 관절 지정된 각도로 꺾기 ---
        // ==========================================================
        auto start_q = robot.get_current_angles();
        angles_t q_pose = *start_q; // 타입 안전을 위해 복사 후 덮어쓰기
        q_pose(0) = 0.0;  // J1
        q_pose(1) = 0.0;  // J2
        q_pose(2) = 90.0; // J3
        q_pose(3) = 0.0;  // J4
        q_pose(4) = 90.0; // J5
        q_pose(5) = 0.0;  // J6

        std::cout << "\n0️⃣ [초기화] 지정된 위치 [0, 0, 90, 0, 90, 0] 로 이동 시작..." << std::endl;
        if (robot.trapj(q_pose)) run_until_reached("Pose_Setup");


        // ==========================================================
        // --- [Step 1] TCP 없음 (Flange 기준 이동) ---
        // ==========================================================
        std::cout << "1️⃣ [TCP 끄기] 오프셋 0 초기화" << std::endl;
        robot.set_tcp(0, 0, 0, 0, 0, 0); 
        wait_for_apply();
        
        std::cout << "   -> [하강] Flange 기준 Z -15cm" << std::endl;
        if (robot.attrl(0.0, 0.0, -0.15)) run_until_reached("1_NoTCP_Down");
        
        std::cout << "   -> [복귀] Flange 기준 Z +15cm" << std::endl;
        if (robot.attrl(0.0, 0.0, 0.15)) run_until_reached("1_NoTCP_Up");


        // ==========================================================
        // --- [Step 2] TCP 있음 (Roll 30도 적용 이동) ---
        // ==========================================================
        std::cout << "2️⃣ [TCP 켜기] RPY(" << tr << ", " << tp << ", " << tyw << ") 적용" << std::endl;
        robot.set_tcp(tx, ty, tz, tr, tp, tyw);
        wait_for_apply();

        std::cout << "   -> [하강] TCP(도구끝) 기준 Z -15cm" << std::endl;
        // 🌟 내부에서 30도 꺾임을 상쇄하여 손목이 자동으로 비틀립니다.
        if (robot.attrl(0.0, 0.0, -0.15)) run_until_reached("2_WithTCP_Down");
        
        std::cout << "   -> [복귀] TCP(도구끝) 기준 Z +15cm" << std::endl;
        if (robot.attrl(0.0, 0.0, 0.15)) run_until_reached("2_WithTCP_Up");


        // ==========================================================
        // --- [Step 3] TCP 다시 없음 (초기화 후 이동 확인) ---
        // ==========================================================
        std::cout << "3️⃣ [TCP 다시 끄기] 오프셋 0 초기화" << std::endl;
        robot.set_tcp(0, 0, 0, 0, 0, 0); 
        wait_for_apply();

        std::cout << "   -> [하강] Flange 기준 Z -15cm" << std::endl;
        if (robot.attrl(0.0, 0.0, -0.15)) run_until_reached("3_NoTCP_Down_Again");
        
        std::cout << "   -> [복귀] Flange 기준 Z +15cm" << std::endl;
        if (robot.attrl(0.0, 0.0, 0.15)) run_until_reached("3_NoTCP_Up_Again");

    } catch (const std::exception& e) {
        std::cerr << "❌ Error: " << e.what() << std::endl;
    }

    // 종료 처리
    csv.close();
    robot.stop();
    (void)robot.servo_off();
    robot.disconnect_rt();
    (void)robot.close_connection();
    std::cout << "\n🏁 [테스트 종료] 모든 시퀀스가 정상적으로 완료되었습니다." << std::endl;
    return 0;
}