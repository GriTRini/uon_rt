#include <iostream>
#include <vector>
#include <iomanip>
#include <fstream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <string>
#include "../../../rt_control/dsr/dsr_robot.hpp" 

using namespace dsr; // 네임스페이스 dsr로 맞춤

std::atomic<bool> keep_running(true);
void sigint_handler(int signum) { keep_running = false; }

int main() {
    std::signal(SIGINT, sigint_handler);

    // [설정] 로봇 모델 및 IP
    Robot<0> robot(std::chrono::milliseconds(1));
    const std::string robot_ip = "192.168.1.30";
    
    // [설정] TCP 오프셋 (길고 복잡하게 꺾인 툴)
    const double tx = -0.1219, ty = -0.1219, tz = 0.26611;
    const double tr = 60.0, tp = 0.0, tyw = -45.0;

    // [파일] 데이터 로깅용 CSV
    std::ofstream csv("tcp_interactive_test_log.csv");
    csv << "Time,Step,J1,J2,J3,J4,J5,J6,TCP_X,TCP_Y,TCP_Z\n";

    // 로봇 연결 및 서보 온
    if (robot.open_connection(robot_ip, 12345) != OpenConnError::NO_ERROR) return -1;
    if (!robot.connect_rt(robot_ip, 12347)) return -1;
    if (robot.servo_on(std::chrono::milliseconds(5000)) != ServoOnError::NO_ERROR) return -1;

    double total_time = 0.0;

    // =========================================================================
    // 🌟 모듈 1: 목표 도달 대기 및 에러 감지 (wait_goal)
    // =========================================================================
    auto wait_goal = [&](const std::string& info, 
                         double p_th = 0.02,  // 위치 허용 오차: 2cm
                         double r_th = 1.0,   // 회전 허용 오차: 1도
                         double timeout_sec = 100.0) -> bool { 
        int loop_count = 0;
        double start_time = total_time;
        
        while (keep_running) {
            // 정지(충돌/에러) 감지 시 즉각 탈출
            if (robot.is_paused()) {
                std::cout << "\n🚨 [Safety Stop 감지] 로봇이 정지되었습니다. (Step: " << info << ")" << std::endl;
                return false; 
            }

            bool reached = robot.get_goal_reached(std::nullopt, p_th, r_th);
            auto cur_q = robot.get_current_angles();
            auto cur_tcp_opt = robot.get_current_tmat(); 

            if (cur_q && cur_tcp_opt) {
                Eigen::Isometry3d cur_tcp = *cur_tcp_opt;
                csv << std::fixed << std::setprecision(5) << total_time << "," << info << ",";
                for(int i=0; i<6; ++i) csv << (*cur_q)(i) << ",";
                csv << cur_tcp.translation().x() << "," << cur_tcp.translation().y() << "," << cur_tcp.translation().z() << "\n";
            }

            if (loop_count % 500 == 0) {
                Robot<0>::angles_t print_q = cur_q ? *cur_q : md::zeros_like(Robot<0>::angles_t{}); 
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
    // 🌟 모듈 2: 에러 대응 전용 함수 (handle_recovery)
    // =========================================================================
    auto handle_recovery = [&]() -> bool {
        std::cout << "\n🚨 티칭 펜던트 조치 후 옵션을 선택하세요." << std::endl;
        
        while (keep_running) {
            std::cout << "   [1] Resume: 이어서 가기 | [2] Cancel: 취소 후 Home으로 복귀\n   입력: ";
            std::string recovery_input;
            std::cin >> recovery_input;

            if (recovery_input == "1") {
                if (robot.resume_trajectory()) {
                    std::cout << "   -> Resume 실행 (다시 목적지로 출발)..." << std::endl;
                    return true; 
                }
            } 
            else if (recovery_input == "2") {
                if (robot.cancel_trajectory()) {
                    std::cout << "   -> 명령 취소. Home 위치로 복귀합니다." << std::endl;
                    Robot<0>::angles_t home_q = md::zeros_like(Robot<0>::angles_t{}); 
                    home_q(2) = -90.0; home_q(4) = -90.0;
                    if (robot.trapj(home_q)) wait_goal("Safety_Home_Return", 0.01, 1.0, 100.0);
                    return false; 
                }
            } 
            else {
                std::cout << "   ⚠️ 1 또는 2를 입력하세요." << std::endl;
            }
        }
        return false;
    };

    // =========================================================================
    // 🌟 모듈 3: 동작 실행 및 에러 처리 통합 래퍼 (execute_move)
    // =========================================================================
    auto execute_move = [&](const Eigen::Isometry3d& target_pose, const std::string& step_info) -> bool {
        if (!robot.attrl(target_pose, 150.0)) return false;
        
        while (keep_running) {
            bool success = wait_goal(step_info, 0.01, 1.0, 100.0);
            if (success) return true; // 정상 도달 
            
            if (robot.is_paused()) {
                if (handle_recovery()) continue; // Resume 시 다시 wait_goal 진입
                else return false;               // Cancel 시 이 동작 취소
            } else {
                return false; // 타임아웃 등 기타 실패
            }
        }
        return false;
    };

    auto wait_for_apply = [&]() {
        for(int i=0; i<300; ++i) { 
            std::this_thread::sleep_for(std::chrono::milliseconds(1)); 
            total_time += 0.001; 
        }
    };

    try {
        // ==========================================================
        // 1️⃣ [Step 1] TCP 설정
        // ==========================================================
        std::cout << "\n1️⃣ [Set TCP] 툴 정보를 로봇에 입력합니다." << std::endl;
        robot.set_tcp(tx, ty, tz, tr, tp, tyw);
        wait_for_apply(); 
        
        // ==========================================================
        // 2️⃣ [Step 2] J3, J5 -90도로 이동 (TrapJ)
        // ==========================================================
        std::cout << "\n2️⃣ [TrapJ] 초기 자세(J3, J5 -90도)로 이동합니다..." << std::endl;
        Robot<0>::angles_t q_pose = md::zeros_like(Robot<0>::angles_t{}); 
        q_pose(2) = -90.0; // J3
        q_pose(4) = -90.0; // J5
        
        if (robot.trapj(q_pose)) wait_goal("2_Setup_TrapJ");

        // ==========================================================
        // 3️⃣ [Step 3] 손목 고정 상태로 수직 정렬
        // ==========================================================
        std::cout << "\n3️⃣ [Align] 툴 팁을 바닥(-Z) 방향으로 정렬합니다..." << std::endl;
        if (robot.align_tcp_to_floor(-90.0, 200.0)) {
            wait_goal("3_Align_Vertical_Fixed_Flange", 0.01, 0.5, 100.0);
        }

        // ==========================================================
        // 4️⃣ [Step 4] 대화형(Interactive) 시퀀스 제어
        // ==========================================================
        std::cout << "\n==========================================================" << std::endl;
        std::cout << "4️⃣ [Interactive Sequence] 터미널에서 목표 좌표(X, Y, Z)를 입력하세요." << std::endl;
        std::cout << "▶ 동작 순서: 목표 5cm 위로 접근 -> 하강 -> 10cm 상승 복귀" << std::endl;
        std::cout << "▶ 단위: 미터(m) / 입력 예시: -0.5 0.1 0.2" << std::endl;
        std::cout << "▶ 종료하려면 'q'를 입력하세요." << std::endl;
        std::cout << "==========================================================\n" << std::endl;
        
        while (keep_running) {
            std::cout << "\n>> 타겟 좌표 X Y Z 입력: ";
            std::string input_x;
            std::cin >> input_x;

            if (input_x == "q" || input_x == "Q") {
                std::cout << "좌표 입력을 종료하고 로봇을 정지합니다." << std::endl;
                break;
            }

            double target_x, target_y, target_z;
            try {
                target_x = std::stod(input_x);
                std::cin >> target_y >> target_z;
            } catch (const std::exception& e) {
                std::cout << "⚠️ 잘못된 입력입니다. 숫자 3개를 띄어쓰기로 구분해 입력하세요." << std::endl;
                std::cin.clear();
                std::cin.ignore(10000, '\n');
                continue;
            }

            std::cout << "\n   🚀 시퀀스 시작: [" << target_x << ", " << target_y << ", " << target_z << "]" << std::endl;

            // 🌟 실제 현재 위치(또는 명령 위치)를 베이스로 타겟 설정
            Eigen::Isometry3d base_target;
            auto actual_pose_opt = robot.get_current_tmat();
            if (actual_pose_opt) base_target = *actual_pose_opt;
            else base_target = robot.get_desired_tmat();

            // [Step 1] 접근: Z + 0.05m
            Eigen::Isometry3d step1_target = base_target;
            step1_target.translation() << target_x, target_y, target_z + 0.05;
            std::cout << "\n   ▶ [Step 1] 목표 상단 5cm 위치로 접근 중..." << std::endl;
            if (!execute_move(step1_target, "Seq_1_Approach")) continue;

            // [Step 2] 하강: Z
            Eigen::Isometry3d step2_target = base_target;
            step2_target.translation() << target_x, target_y, target_z;
            std::cout << "\n   ▶ [Step 2] 목표 지점으로 수직 하강 중..." << std::endl;
            if (!execute_move(step2_target, "Seq_2_Reach")) continue;

            // [Step 3] 상승 복귀: Z + 0.10m
            Eigen::Isometry3d step3_target = base_target;
            step3_target.translation() << target_x, target_y, target_z + 0.10;
            std::cout << "\n   ▶ [Step 3] 10cm 위로 수직 상승 중..." << std::endl;
            if (!execute_move(step3_target, "Seq_3_Retract")) continue;

            std::cout << "\n   🎉 시퀀스 이동 성공!" << std::endl;
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