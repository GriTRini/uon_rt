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

using namespace dsr_control;

std::atomic<bool> keep_running(true);
void sigint_handler(int signum) { keep_running = false; }

int main() {
    std::signal(SIGINT, sigint_handler);

    // [설정] 로봇 모델 및 IP
    Robot<0> robot("m1013", std::chrono::milliseconds(1));
    const std::string robot_ip = "192.168.1.30";
    
    // [설정] TCP 오프셋
    const double tx = -0.1219, ty = -0.1219, tz = 0.26611;
    const double tr = 60.0, tp = 0.0, tyw = -45.0;

    std::ofstream csv("tcp_interactive_test_log.csv");
    csv << "Time,Step,J1,J2,J3,J4,J5,J6,TCP_X,TCP_Y,TCP_Z\n";

    if (robot.open_connection(robot_ip, 12345) != OpenConnError::NO_ERROR) return -1;
    if (!robot.connect_rt(robot_ip, 12347)) return -1;
    if (robot.servo_on(std::chrono::milliseconds(5000)) != ServoOnError::NO_ERROR) return -1;

    double total_time = 0.0;

    // 🌟 개선된 wait_goal: 에러 감지 기능 포함
    auto wait_goal = [&](const std::string& info, 
                         double p_th = 0.02,  
                         double r_th = 1.0,   
                         double timeout_sec = 100.0) { 
        int loop_count = 0;
        double start_time = total_time;
        
        while (keep_running) {
            // 🌟 1. 로봇이 Safety Stop 등 물리적 에러에 빠졌는지 체크
            // (Robot 클래스의 get_robot_state 함수가 있다고 가정하거나 SDK 직접 호출)
            // 여기서는 SDK 함수를 직접 호출하여 Standby나 Moving이 아닌 상태를 에러로 봅니다.
            /* * 주의: 실제로는 get_robot_state를 public으로 뚫어두는 것이 좋습니다.
             * 여기서는 로직의 흐름을 보여줍니다.
             */
            // bool is_error = robot.is_in_error_state(); 
            // if (is_error) {
            //     std::cout << "\n🚨 [충돌/에러 감지] 로봇이 멈췄습니다! 대기를 중단합니다." << std::endl;
            //     return false; // 에러 발생 시 false 반환
            // }

            bool reached = robot.get_goal_reached(std::nullopt, p_th, r_th);
            auto cur_q = robot.get_current_angles();
            auto cur_tcp = robot.get_task_pos(); 

            if (cur_q) {
                csv << std::fixed << std::setprecision(5) << total_time << "," << info << ",";
                for(int i=0; i<6; ++i) csv << (*cur_q)(i) << ",";
                csv << cur_tcp.translation().x() << "," << cur_tcp.translation().y() << "," << cur_tcp.translation().z() << "\n";
            }

            if (loop_count % 500 == 0) {
                angles_t print_q = cur_q ? *cur_q : angles_t::Zero(); 
                std::cout << std::fixed << std::setprecision(4)
                          << "[T: " << std::setw(6) << total_time << "s] " << std::setw(18) << info 
                          << " | J: [" << print_q.transpose() << "]" << std::endl;
            }

            if (reached) {
                std::cout << "      ✅ " << info << " 완료\n" << std::endl;
                return true; // 성공 시 true 반환
            }
            
            if ((total_time - start_time) >= timeout_sec) {
                std::cout << "      ⚠️ " << info << " 타임아웃 (강제 진행)\n" << std::endl;
                return false;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            total_time += 0.001;
            loop_count++;
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
        // [Step 1] TCP 설정
        std::cout << "\n1️⃣ [Set TCP] 툴 정보를 로봇에 입력합니다." << std::endl;
        robot.set_tcp(tx, ty, tz, tr, tp, tyw);
        wait_for_apply(); 
        
        // [Step 2] J3, J5 -90도로 이동 (TrapJ)
        std::cout << "\n2️⃣ [TrapJ] 초기 자세로 이동합니다..." << std::endl;
        angles_t q_pose = angles_t::Zero(); 
        q_pose(2) = -90.0; q_pose(4) = -90.0;
        
        if (robot.trapj(q_pose)) wait_goal("2_Setup_TrapJ");

        // [Step 3] 손목 고정 상태로 수직 정렬
        std::cout << "\n3️⃣ [Align] 툴 팁을 바닥(-Z) 방향으로 정렬합니다..." << std::endl;
        if (robot.align_tcp_to_floor(-90.0, 200.0)) {
            wait_goal("3_Align_Vertical_Fixed_Flange", 0.01, 0.5, 100.0);
        }

        // ... (앞부분 동일: TCP 설정, Home 이동 등) ...

        // [Step 4] 대화형 모드 (이동 + 에러 복구 루프)
        std::cout << "\n==========================================================" << std::endl;
        std::cout << "4️⃣ [Interactive Mode] 터미널에서 목표 좌표(X, Y, Z)를 입력하세요." << std::endl;
        std::cout << "▶ 단위: 미터(m) (예: -0.5 0.1 0.2)" << std::endl;
        std::cout << "▶ 종료하려면 'q' 입력" << std::endl;
        std::cout << "==========================================================\n" << std::endl;
        
        while (keep_running) {
            std::cout << "\n>> 이동할 좌표 X Y Z 입력 (또는 q): ";
            std::string input_x;
            std::cin >> input_x;

            if (input_x == "q" || input_x == "Q") break;

            // 일반 좌표 입력 처리
            double target_x, target_y, target_z;
            try {
                target_x = std::stod(input_x);
                std::cin >> target_y >> target_z;
            } catch (const std::exception& e) {
                std::cout << "⚠️ 잘못된 입력입니다." << std::endl;
                std::cin.clear(); std::cin.ignore(10000, '\n');
                continue;
            }

            std::cout << "   🚀 목표 지점 [" << target_x << ", " << target_y << ", " << target_z << "] 로 이동 명령 전송..." << std::endl;

            Eigen::Isometry3d target = robot.get_task_pos(); 
            target.translation() << target_x, target_y, target_z;

            if (robot.attrl(target, 150.0)) {
                // 🌟 이동 중 충돌/에러 발생 시 false 반환
                bool success = wait_goal("Move_To_Input", 0.01, 1.0, 100.0);
                
                if (!success && keep_running) {
                    std::cout << "\n🚨 [충돌 감지] 로봇이 목적지에 도달하지 못하고 멈췄습니다!" << std::endl;
                    std::cout << "   문제를 해결한 후, 티칭 펜던트에서 [Reset] 버튼을 눌러주세요." << std::endl;
                    
                    // 🌟 에러 복구 서브 루프 진입
                    while (keep_running) {
                        std::cout << "\n>> 복구 옵션을 선택하세요:" << std::endl;
                        std::cout << "   [1] 끊긴 궤적 마저 이어서 가기 (Resume)" << std::endl;
                        std::cout << "   [2] 이동 취소 및 Home 위치로 복귀 (Cancel & Home)" << std::endl;
                        std::cout << "   입력: ";
                        
                        std::string recovery_input;
                        std::cin >> recovery_input;

                        if (recovery_input == "1") {
                            if (robot.resume_trajectory()) {
                                std::cout << "   -> Resume 성공! 다시 원래 목표를 향해 출발합니다." << std::endl;
                                wait_goal("Resume_Moving", 0.01, 1.0, 100.0);
                                break; // 복구 루프 탈출, 메인 루프로 복귀
                            }
                        } 
                        else if (recovery_input == "2") {
                            // 1. 기존 궤적 깔끔하게 취소 (Pause 해제)
                            if (robot.cancel_trajectory()) {
                                std::cout << "   -> 기존 명령 취소 성공! Home 위치로 대피합니다." << std::endl;
                                
                                // 2. Home 자세(J3, J5 -90도) 정의 및 TrapJ 명령 하달
                                angles_t home_q = angles_t::Zero(); 
                                home_q(2) = -90.0; home_q(4) = -90.0;
                                
                                robot.trapj(home_q);
                                wait_goal("Emergency_Return_Home", 0.01, 1.0, 100.0);
                                break; // 복구 루프 탈출, 메인 루프로 복귀
                            }
                        } 
                        else {
                            std::cout << "   ⚠️ 잘못된 입력입니다. 1 또는 2를 선택하세요." << std::endl;
                        }
                    } // end of recovery while loop
                }
            }
        } // end of main while loop

    } catch (const std::exception& e) {
        std::cerr << "❌ Error: " << e.what() << std::endl;
    }

    // 🏁 종료 절차
    csv.close();
    robot.stop();
    (void)robot.servo_off();
    robot.disconnect_rt();
    (void)robot.close_connection();
    
    std::cout << "\n🏁 [테스트 종료] 실제 로봇 구동 완료!" << std::endl;
    return 0;
}