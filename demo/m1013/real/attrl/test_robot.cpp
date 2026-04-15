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

using namespace rt_control; // 네임스페이스 수정

std::atomic<bool> keep_running(true);
void sigint_handler(int signum) { keep_running = false; }

int main() {
    std::signal(SIGINT, sigint_handler);

    // 💡 수정됨: Robot<0> 대신 DsrRobot 클래스를 직접 사용
    DsrRobot robot("m1013", std::chrono::milliseconds(1));
    const std::string robot_ip = "192.168.1.30";
    
    // [설정] TCP 오프셋
    const double tx = -0.029, ty = 0.0, tz = 0.3819;
    const double tr = 0.0, tp = 0.0, tyw = 0.0;

    // 데이터 로깅용 CSV
    std::ofstream csv("sequence_test_log.csv");
    csv << "Time,Step,J1,J2,J3,J4,J5,J6,TCP_X,TCP_Y,TCP_Z\n";

    // 로봇 연결 및 서보 온
    std::cout << "로봇 연결을 시도합니다..." << std::endl;
    // (OpenConnError 같은 enum이 rt_control에 없는 경우 단순 int 캐스팅이나 제거가 필요할 수 있으나 기존 형태 유지)
    if (!robot.open_connection(robot_ip, 12345)) return -1;
    if (!robot.connect_rt(robot_ip, 12347)) return -1;
    if (!robot.servo_on()) return -1;

    double total_time = 0.0;

    // =========================================================================
    // 🌟 모니터링 함수
    // =========================================================================
    auto wait_goal = [&](const std::string& info, 
                         double p_th = 0.02,  
                         double r_th = 1.0,   
                         double timeout_sec = 100.0) -> bool { 
        int loop_count = 0;
        double start_time = total_time;
        
        while (keep_running) {
            if (robot.has_alarm()) {
                std::cerr << "❌ [알람 발생] 로봇 동작 중단: " << info << std::endl;
                return false; 
            }

            bool reached = robot.get_goal_reached(std::nullopt, p_th, r_th);
            auto cur_q = robot.get_current_angles();
            std::optional<Eigen::Isometry3d> cur_tcp_opt = std::nullopt;
            
            if (cur_q) cur_tcp_opt = robot.solve_forward(*cur_q);

            if (cur_q && cur_tcp_opt) {
                Eigen::Isometry3d cur_tcp = *cur_tcp_opt;
                csv << std::fixed << std::setprecision(5) << total_time << "," << info << ",";
                for(int i=0; i<6; ++i) csv << (*cur_q)(i) << ",";
                csv << cur_tcp.translation().x() << "," << cur_tcp.translation().y() << "," << cur_tcp.translation().z() << "\n";
            }

            if (loop_count % 500 == 0) {
                // 💡 수정됨: rt_control::angles_t 사용
                rt_control::angles_t print_q = cur_q ? *cur_q : rt_control::angles_t::Zero(); 
                std::cout << std::fixed << std::setprecision(4)
                          << "[T: " << std::setw(6) << total_time << "s] " << std::setw(18) << info 
                          << " | J: [" << print_q.transpose() << "]" << std::endl;
            }

            if (reached) {
                std::cout << "      ✅ [" << info << "] 도달 완료!\n" << std::endl;
                return true;
            }
            if ((total_time - start_time) >= timeout_sec) {
                std::cout << "      ⚠️ [" << info << "] 타임아웃 발생\n" << std::endl;
                return false;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            total_time += 0.001;
            loop_count++;
        }
        return false;
    };

    auto execute_move = [&](double x, double y, double z, double r, double p, double yaw, const std::string& step_info) -> bool {
        if (!robot.attrl(x, y, z, r, p, yaw, 50.0)) return false; 
        return wait_goal(step_info, 0.01, 1.0, 100.0);
    };

    // =========================================================================
    // 🚀 메인 자동 시퀀스 실행부
    // =========================================================================
    try {
        std::cout << "\n1️⃣ [Set TCP] 툴 정보를 로봇에 입력합니다." << std::endl;
        robot.set_tcp(tx, ty, tz, tr, tp, tyw);
        for(int i=0; i<300; ++i) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); total_time += 0.001; }
        
        // ---------------------------------------------------------
        // 2️⃣ 초기 홈 자세(Home Point)로 이동 (TrapJ)
        // ---------------------------------------------------------
        std::cout << "\n2️⃣ [홈 복귀] 초기 조인트 자세(-90, 0, -90, 0, -90, 0)로 이동합니다..." << std::endl;
        
        // 💡 수정됨: rt_control::angles_t 사용
        rt_control::angles_t home_pose;
        home_pose << -90.0, 0.0, -90.0, 0.0, -90.0, 0.0;
        
        if (robot.trapj(home_pose)) {
            if (!wait_goal("Move_To_Home")) throw std::runtime_error("홈 자세 이동 실패");
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        total_time += 1.0;

        // ---------------------------------------------------------
        // 3️⃣ 지정된 Task 위치로 이동 (attrl)
        // ---------------------------------------------------------
        double target_x = -0.500; 
        double target_y =  0.100;
        double target_z =  0.200;
        double target_r =  180.0; 
        double target_p =  0.0;
        double target_yaw = 90.0; 

        std::cout << "\n3️⃣ [목표 이동] 지정된 Task 좌표로 이동합니다 (attrl)..." << std::endl;
        
        if (!execute_move(target_x, target_y, target_z, target_r, target_p, target_yaw, "Move_To_Target")) {
            throw std::runtime_error("목표 좌표 이동 실패");
        }

        std::cout << "\n  ⏱️ 목적지 도착. 2초 대기 중..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        total_time += 2.0;

        // ---------------------------------------------------------
        // 4️⃣ 다시 홈 자세(Home Point)로 복귀 (TrapJ)
        // ---------------------------------------------------------
        std::cout << "\n4️⃣ [작업 완료] 다시 홈 자세로 복귀합니다..." << std::endl;
        if (robot.trapj(home_pose)) {
            if (!wait_goal("Return_To_Home")) throw std::runtime_error("홈 복귀 실패");
        }

    } catch (const std::exception& e) {
        std::cerr << "❌ Error: " << e.what() << std::endl;
    }

    csv.close();
    robot.stop();
    (void)robot.servo_off();
    robot.disconnect_rt();
    (void)robot.close_connection();
    
    std::cout << "\n🏁 [프로그램 정상 종료] 로봇 구동 완료!" << std::endl;
    return 0;
}