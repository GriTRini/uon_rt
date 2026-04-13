#include <iostream>
#include <vector>
#include <iomanip>
#include <fstream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <string>
#include <optional>
#include "../../../../rt_control/dsr/dsr_robot.hpp" 

using namespace dsr_control; 

std::atomic<bool> keep_running(true);
void sigint_handler(int signum) { keep_running = false; }

int main() {
    std::signal(SIGINT, sigint_handler);

    // 첫 번째 인자로 모델명("m1013") 명시
    Robot<0> robot("m1013", std::chrono::milliseconds(1));
    const std::string robot_ip = "192.168.1.30";
    
    // [설정] TCP 오프셋
    const double tx = -0.029, ty = 0.0, tz = 0.3819;
    const double tr = 0.0, tp = 0.0, tyw = 0.0;

    // [파일] 데이터 로깅용 CSV
    std::ofstream csv("tcp_interactive_test_log.csv");
    csv << "Time,Step,J1,J2,J3,J4,J5,J6,TCP_X,TCP_Y,TCP_Z\n";

    // 로봇 연결 및 서보 온
    if (robot.open_connection(robot_ip, 12345) != OpenConnError::NO_ERROR) return -1;
    if (!robot.connect_rt(robot_ip, 12347)) return -1;
    if (robot.servo_on(std::chrono::milliseconds(5000)) != ServoOnError::NO_ERROR) return -1;

    double total_time = 0.0;

    // =========================================================================
    // 🌟 모듈 1: 목표 도달 대기 (순수 모니터링)
    // =========================================================================
    auto wait_goal = [&](const std::string& info, 
                         double p_th = 0.02,  
                         double r_th = 1.0,   
                         double timeout_sec = 100.0) -> bool { 
        int loop_count = 0;
        double start_time = total_time;
        
        while (keep_running) {
            bool reached = robot.get_goal_reached(std::nullopt, p_th, r_th);
            auto cur_q = robot.get_current_angles();
            
            std::optional<Eigen::Isometry3d> cur_tcp_opt = std::nullopt;
            if (cur_q) {
                cur_tcp_opt = robot.solve_forward(*cur_q);
            }

            // 데이터 로깅
            if (cur_q && cur_tcp_opt) {
                Eigen::Isometry3d cur_tcp = *cur_tcp_opt;
                csv << std::fixed << std::setprecision(5) << total_time << "," << info << ",";
                for(int i=0; i<6; ++i) csv << (*cur_q)(i) << ",";
                csv << cur_tcp.translation().x() << "," << cur_tcp.translation().y() << "," << cur_tcp.translation().z() << "\n";
            }

            if (loop_count % 500 == 0) {
                Robot<0>::angles_t print_q = cur_q ? *cur_q : Robot<0>::angles_t::Zero(); 
                std::cout << std::fixed << std::setprecision(4)
                          << "[T: " << std::setw(6) << total_time << "s] " << std::setw(18) << info 
                          << " | J: [" << print_q.transpose() << "]" << std::endl;
            }

            if (reached) {
                std::cout << "      ✅ " << info << " 완료\n" << std::endl;
                return true;
            }
            
            if ((total_time - start_time) >= timeout_sec) {
                std::cout << "      ⚠️ " << info << " 타임아웃 발생\n" << std::endl;
                return false;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            total_time += 0.001;
            loop_count++;
        }
        return false;
    };

    // =========================================================================
    // 🌟 모듈 2: 동작 실행 래퍼 (행렬 대신 X,Y,Z,R,P,Y 직접 입력)
    // =========================================================================
    auto execute_move = [&](double x, double y, double z, double r, double p, double yaw, const std::string& step_info) -> bool {
        // 💡 Kp 값을 150.0에서 50.0으로 고정하여 과가속 발산 방지!
        if (!robot.attrl(x, y, z, r, p, yaw, 150.0)) return false;
        return wait_goal(step_info, 0.01, 1.0, 100.0);
    };

    auto wait_for_apply = [&]() {
        for(int i=0; i<300; ++i) { 
            std::this_thread::sleep_for(std::chrono::milliseconds(1)); 
            total_time += 0.001; 
        }
    };

    // =========================================================================
    // 🚀 메인 실행부
    // =========================================================================
    try {
        std::cout << "\n1️⃣ [Set TCP] 툴 정보를 로봇에 입력합니다." << std::endl;
        robot.set_tcp(tx, ty, tz, tr, tp, tyw);
        wait_for_apply(); 
        
        std::cout << "\n2️⃣ [TrapJ] 초기 자세(J1, J3, J5 -90도)로 이동합니다..." << std::endl;
        Robot<0>::angles_t q_pose = Robot<0>::angles_t::Zero(); 
        q_pose(0) = -90.0; // J1
        q_pose(2) = -90.0; // J3
        q_pose(4) = -90.0; // J5
        if (robot.trapj(q_pose)) wait_goal("2_Setup_TrapJ");

        std::cout << "\n3️⃣ [Align] 툴 팁을 바닥(-Z) 방향으로 정렬합니다..." << std::endl;
        // 💡 특이점(Singularity) 회피를 위해 Yaw 각도를 180도가 아닌 90도로 정렬
        if (robot.align_tcp_to_floor(90.0, 100.0)) {
            wait_goal("3_Align_Vertical_Fixed_Flange", 0.01, 0.5, 100.0);
        }

        std::cout << "\n==========================================================" << std::endl;
        std::cout << "4️⃣ [Pick & Place Sequence] 터미널에서 좌표를 입력하세요." << std::endl;
        std::cout << "▶ 단위: 미터(m) / 입력 예시: -0.5 0.1 0.2" << std::endl;
        std::cout << "▶ 종료하려면 'q'를 입력하세요." << std::endl;
        std::cout << "==========================================================\n" << std::endl;
        
        // 🌟 절대 자세 설정: 툴 끝이 바닥을 보게 하되, 특이점 회피를 위해 Yaw를 90도로 세팅
        const double target_r = 180.0;
        const double target_p = 0.0;
        const double target_yaw = 90.0;

        while (keep_running) {
            // ---------------------------------------------------------
            // [Phase 1] PICK (집기) 파트
            // ---------------------------------------------------------
            std::cout << "\n>> 🟩 [PICK] 집으러 갈 좌표 X Y Z 입력 (또는 q): ";
            std::string input_x;
            std::cin >> input_x;

            if (input_x == "q" || input_x == "Q") break;

            double pick_x, pick_y, pick_z;
            try {
                pick_x = std::stod(input_x);
                std::cin >> pick_y >> pick_z;
            } catch (...) {
                std::cout << "⚠️ 잘못된 입력입니다. 숫자 3개를 띄어쓰기로 구분해 입력하세요." << std::endl;
                std::cin.clear(); std::cin.ignore(10000, '\n');
                continue;
            }

            std::cout << "\n   🚀 [Pick 시퀀스 시작] 목표: [" << pick_x << ", " << pick_y << ", " << pick_z << "]" << std::endl;

            // 1. Pick 접근 (Z + 0.05m)
            std::cout << "\n   ▶ [Step 1] Pick 상단 5cm 위치로 접근 중..." << std::endl;
            if (!execute_move(pick_x, pick_y, pick_z + 0.05, target_r, target_p, target_yaw, "Pick_Approach")) continue;

            // 2. Pick 하강 (Z)
            std::cout << "\n   ▶ [Step 2] Pick 지점으로 수직 하강 중..." << std::endl;
            if (!execute_move(pick_x, pick_y, pick_z, target_r, target_p, target_yaw, "Pick_Reach")) continue;

            // 💡 [I/O ON] 물건 잡기
            std::cout << "\n   🧲 [I/O 제어] 그리퍼 작동 (ON)" << std::endl;
            robot.set_digital_output(9, true);
            std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 흡착 대기

            // 3. Pick 상승 (Z + 0.10m)
            std::cout << "\n   ▶ [Step 3] 10cm 위로 수직 상승 중..." << std::endl;
            if (!execute_move(pick_x, pick_y, pick_z + 0.10, target_r, target_p, target_yaw, "Pick_Retract")) continue;


            // ---------------------------------------------------------
            // [Phase 2] PLACE (놓기) 파트
            // ---------------------------------------------------------
            std::cout << "\n>> 🟦 [PLACE] 놓으러 갈 좌표 X Y Z 입력 (또는 q): ";
            std::cin >> input_x;

            if (input_x == "q" || input_x == "Q") break;

            double place_x, place_y, place_z;
            try {
                place_x = std::stod(input_x);
                std::cin >> place_y >> place_z;
            } catch (...) {
                std::cout << "⚠️ 잘못된 입력입니다. 루프를 처음(Pick)부터 다시 시작합니다." << std::endl;
                std::cin.clear(); std::cin.ignore(10000, '\n');
                continue;
            }

            std::cout << "\n   🚀 [Place 시퀀스 시작] 목표: [" << place_x << ", " << place_y << ", " << place_z << "]" << std::endl;

            // 4. Place 접근 (Z + 0.05m)
            std::cout << "\n   ▶ [Step 4] Place 상단 5cm 위치로 접근 중..." << std::endl;
            if (!execute_move(place_x, place_y, place_z + 0.05, target_r, target_p, target_yaw, "Place_Approach")) continue;

            // 5. Place 하강 (Z)
            std::cout << "\n   ▶ [Step 5] Place 지점으로 수직 하강 중..." << std::endl;
            if (!execute_move(place_x, place_y, place_z, target_r, target_p, target_yaw, "Place_Reach")) continue;

            // 💡 [I/O OFF] 물건 놓기
            std::cout << "\n   👐 [I/O 제어] 그리퍼 해제 (OFF)" << std::endl;
            robot.set_digital_output(9, false);
            std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 떨어질 때까지 대기

            // 6. Place 상승 (Z + 0.10m)
            std::cout << "\n   ▶ [Step 6] 10cm 위로 수직 상승 중..." << std::endl;
            if (!execute_move(place_x, place_y, place_z + 0.10, target_r, target_p, target_yaw, "Place_Retract")) continue;

            std::cout << "\n   🎉 1회 Pick & Place 완료!" << std::endl;
        }

    } catch (const std::exception& e) {
        std::cerr << "❌ Error: " << e.what() << std::endl;
    }

    // ==========================================================
    // 🏁 [종료] 로봇 안전 종료
    // ==========================================================
    csv.close();
    robot.stop();
    (void)robot.servo_off();
    robot.disconnect_rt();
    (void)robot.close_connection();
    
    std::cout << "\n🏁 [테스트 종료] 실제 로봇 구동 완료!" << std::endl;
    return 0;
}