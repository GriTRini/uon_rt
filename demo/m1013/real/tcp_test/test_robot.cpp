#include <iostream>
#include <vector>
#include <iomanip>
#include <fstream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include "../../../rt_control/dsr/dsr_robot.hpp" 

using namespace dsr_control;

std::atomic<bool> keep_running(true);
void sigint_handler(int signum) { keep_running = false; }

int main() {
    std::signal(SIGINT, sigint_handler);

    // [설정] 로봇 모델 및 IP
    Robot<0> robot("m1013", std::chrono::milliseconds(1));
    const std::string robot_ip = "192.168.1.30";
    
    // [설정] TCP 오프셋 (길고 복잡하게 꺾인 툴)
    const double tx = -0.22, ty = -0.22, tz = 0.3;
    const double tr = 60.0, tp = 0.0, tyw = -45.0;

    // [파일] 데이터 로깅용 CSV
    std::ofstream csv("tcp_test_log.csv");
    csv << "Time,Step,J1,J2,J3,J4,J5,J6,TCP_X,TCP_Y,TCP_Z\n";

    // 로봇 연결 및 서보 온
    if (robot.open_connection(robot_ip, 12345) != OpenConnError::NO_ERROR) return -1;
    if (!robot.connect_rt(robot_ip, 12347)) return -1;
    if (robot.servo_on(std::chrono::milliseconds(5000)) != ServoOnError::NO_ERROR) return -1;

    double total_time = 0.0;

    // 🌟 헬퍼 함수: goal_reached()를 사용하여 직관적으로 대기 및 로깅
    auto wait_goal = [&](const std::string& info, 
                         double p_th = 0.02,  // 위치 허용 오차: 2cm
                         double r_th = 1.0,   // 회전 허용 오차: 1도
                         double timeout_sec = 100.0) { 
        int loop_count = 0;
        double start_time = total_time;
        
        while (keep_running) {
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
                break;
            }
            
            if ((total_time - start_time) >= timeout_sec) {
                std::cout << "      ⚠️ " << info << " 타임아웃 (강제 진행)\n" << std::endl;
                break;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            total_time += 0.001;
            loop_count++;
        }
    };

    auto wait_for_apply = [&]() {
        for(int i=0; i<300; ++i) { 
            std::this_thread::sleep_for(std::chrono::milliseconds(1)); 
            total_time += 0.001; 
        }
    };

    try {
        // ==========================================================
        // 1️⃣ [Step 1] TCP 설정 (가장 먼저 실행!)
        // ==========================================================
        std::cout << "\n1️⃣ [Set TCP] 툴 정보를 로봇에 입력합니다." << std::endl;
        robot.set_tcp(tx, ty, tz, tr, tp, tyw);
        wait_for_apply(); 
        
        // ==========================================================
        // 2️⃣ [Step 2] J3, J5 -90도로 이동 (TrapJ)
        // ==========================================================
        std::cout << "\n2️⃣ [TrapJ] 초기 자세(J3, J5 -90도)로 이동합니다..." << std::endl;
        angles_t q_pose = angles_t::Zero(); 
        q_pose(2) = -90.0; // J3
        q_pose(4) = -90.0; // J5
        
        if (robot.trapj(q_pose)) wait_goal("2_Setup_TrapJ");

        // ==========================================================
        // 3️⃣ [Step 3] 손목(Flange) 고정 상태로 수직 정렬 (내장 함수 사용)
        // ==========================================================
        std::cout << "\n3️⃣ [Align] 손목 위치를 고정하고 툴 팁을 바닥(-Z) 방향으로 정렬합니다..." << std::endl;
        
        // 라이브러리에 내장시킨 함수 단 한 줄 호출! (Kp=80.0으로 부드럽게)
        if (robot.align_tcp_to_floor(-90.0, 200.0)) {
            // 위치가 살짝 흔들릴 수 있으니 오차 1cm(0.01), 회전 2도로 세팅
            wait_goal("3_Align_Vertical_Fixed_Flange", 0.01, 0.5, 100.0);
        }
// ==========================================================
        // 4️⃣ [Step 4] 수직 자세 유지하며 절대 좌표 사각형 궤적 그리기
        // ==========================================================
        std::cout << "\n4️⃣ [Square] 바닥을 바라본 상태로 절대 좌표 궤적을 그립니다..." << std::endl;
        
        Eigen::Isometry3d target = robot.get_task_pos(); 

        // 1번 지점: X = -50cm, Y = 0cm, Z = 0cm
        target.translation().x() = -0.8;
        target.translation().y() =  0.0;
        target.translation().z() =  0.0;
        if (robot.attrl(target, 150.0)) wait_goal("4_Pos_1");

        // 2번 지점: X = -50cm, Y = 10cm, Z = 0cm
        target.translation().x() = -0.8;
        target.translation().y() =  0.1;
        target.translation().z() =  0.0;
        if (robot.attrl(target, 150.0)) wait_goal("4_Pos_2");

        // 3번 지점: X = -55cm, Y = 10cm, Z = 0cm
        target.translation().x() = -0.85;
        target.translation().y() =  0.1;
        target.translation().z() =  0.0;
        if (robot.attrl(target, 150.0)) wait_goal("4_Pos_3");

        // 4번 지점: X = -55cm, Y = -10cm, Z = 0cm
        target.translation().x() = -0.85;
        target.translation().y() = -0.1;
        target.translation().z() =  0.0;
        if (robot.attrl(target, 150.0)) wait_goal("4_Pos_4");

        // 5번 지점(원위치 복귀): X = -50cm, Y = 0cm, Z = 0cm
        target.translation().x() = -0.5;
        target.translation().y() =  0.0;
        target.translation().z() =  0.0;
        if (robot.attrl(target, 150.0)) wait_goal("4_Pos_5_Return");

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