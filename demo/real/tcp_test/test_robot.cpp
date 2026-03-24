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
    
    // [설정] TCP 오프셋 (Roll 30도)
    const double tx = -0.1218, ty = -0.1218, tz = 0.2661;
    const double tr = 30.0, tp = 0.0, tyw = 0.0;

    // [파일] 데이터 로깅용 CSV
    std::ofstream csv("tcp_square_real_test_log.csv");
    csv << "Time,Step,J1,J2,J3,J4,J5,J6,TCP_X,TCP_Y,TCP_Z\n";

    // 로봇 연결 및 서보 온
    if (robot.open_connection(robot_ip, 12345) != OpenConnError::NO_ERROR) return -1;
    if (!robot.connect_rt(robot_ip, 12347)) return -1;
    if (robot.servo_on(std::chrono::milliseconds(5000)) != ServoOnError::NO_ERROR) return -1;

    double total_time = 0.0;

    // 🌟 헬퍼 함수: goal_reached()를 사용하여 직관적으로 대기 및 로깅
    auto wait_goal = [&](const std::string& info, 
                         double p_th = 0.02,  // 위치 허용 오차: 기본 5mm
                         double r_th = 1.0,    // 회전 허용 오차: 기본 3도
                         double timeout_sec = 100.0) { // 8초가 지나면 강제로 다음 진행
        int loop_count = 0;
        double start_time = total_time;
        
        while (keep_running) {
            // q_th(관절 오차)는 nullopt로 무시하고, 위치와 회전 오차만 검사
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

            // 🌟 조건 1: 허용 오차 내에 들어오면 성공
            if (reached) {
                std::cout << "      ✅ " << info << " 완료\n" << std::endl;
                break;
            }
            
            // 🌟 조건 2: 시간이 너무 오래 끌리면 강제 진행 (무한 루프 방지)
            if ((total_time - start_time) >= timeout_sec) {
                std::cout << "      ⚠️ " << info << " 타임아웃 (오차 허용치 도달 실패, 강제 진행)\n" << std::endl;
                break;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            total_time += 0.001;
            loop_count++;
        }
    };

    // TCP 적용 후 내부 갱신을 위한 짧은 대기
    auto wait_for_apply = [&]() {
        for(int i=0; i<300; ++i) { 
            std::this_thread::sleep_for(std::chrono::milliseconds(1)); 
            total_time += 0.001; 
        }
    };

    try {
        // ==========================================================
        // 1️⃣ [Step 1] J3, J5 90도로 이동 (TrapJ)
        // ==========================================================
        std::cout << "\n1️⃣ [TrapJ] J3, J5를 90도로 이동합니다..." << std::endl;
        angles_t q_pose = angles_t::Zero(); 
        q_pose(2) = 90.0; // J3
        q_pose(4) = 90.0; // J5
        
        if (robot.trapj(q_pose)) wait_goal("1_Setup_TrapJ");


        // ==========================================================
        // 2️⃣ [Step 2] TCP 설정
        // ==========================================================
        std::cout << "\n2️⃣ [Set TCP] 툴 정보를 로봇에 입력합니다." << std::endl;
        robot.set_tcp(tx, ty, tz, tr, tp, tyw);
        wait_for_apply(); 


        // ==========================================================
        // 3️⃣ [Step 3] TCP 팁을 바닥으로 수직 정렬
        // ==========================================================
        std::cout << "\n3️⃣ [Align] 툴 팁을 바닥(-Z) 방향으로 정렬합니다..." << std::endl;
        if (robot.align_tcp_to_floor(200.0)) wait_goal("3_Align_Vertical");


        // ==========================================================
        // 4️⃣ [Step 4] 사각형 궤적 이동 (수직 자세 유지)
        // ==========================================================
        std::cout << "\n4️⃣ [Square] 바닥을 바라본 상태로 사각형 궤적을 그립니다..." << std::endl;
        double step_size = 0.2; // 20cm 이동
        
        // 🌟 에러 수정 2: tmat_t 타입 대신 전역 범위에서 접근 가능한 Eigen::Isometry3d 사용
        Eigen::Isometry3d target; 

        // Side 1: X축 이동
        target = robot.get_task_pos();
        target.translation().x() += step_size; 
        if (robot.attrl(target, 200.0)) wait_goal("4_Side_1_X");

        // Side 2: Y축 이동
        target = robot.get_task_pos();
        target.translation().y() += step_size; 
        if (robot.attrl(target, 200.0)) wait_goal("4_Side_2_Y");

        // Side 3: X축 복귀
        target = robot.get_task_pos();
        target.translation().x() -= step_size; 
        if (robot.attrl(target, 200.0)) wait_goal("4_Side_3_X");

        // Side 4: Y축 복귀
        target = robot.get_task_pos();
        target.translation().y() -= step_size; 
        if (robot.attrl(target, 200.0)) wait_goal("4_Side_4_Y");


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