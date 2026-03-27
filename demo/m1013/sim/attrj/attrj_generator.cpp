#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>
#include <chrono>   // 시간 측정
#include <numeric>  // 통계
#include <algorithm>
#include "../../../../rt_control/model/model.hpp"
#include "../../../../rt_control/trajectory/trajectory_generator.hpp"

using namespace rt_control;
using namespace rt_control::trajectory;

int main() {
    // 1. 로봇 모델 및 타겟 설정
    rt_control::model::RobotModel model("m1013");
    
    angles_t q_start;  q_start << 0, 0, 90, 0, 90, 0;
    angles_t q_goal1; q_goal1 << 30, -20, 110, 45, 60, 90;
    angles_t q_goal2; q_goal2 << -30, 20, 70, -45, 120, -90;

    // 2. 제너레이터 초기화
    TrajGenerator traj_gen;
    traj_gen.initialize(model, q_start, angles_t::Zero(), angles_t::Zero());

    // 3. 디버깅용 CSV 설정
    std::ofstream csv("attrj_bench_report.csv");
    csv << "Time,Step,Q_Err,DQ_Norm,DDQ_Norm,Exec_us,Reached\n";

    double dt = 0.001; // 목표 주기: 1ms (1000Hz)
    double current_time = 0.0;
    std::vector<angles_t> targets = {q_goal1, q_goal2, q_start};
    std::vector<std::string> step_names = {"MOVE_1", "MOVE_2", "RETURN"};

    // 제어 게인 설정
    double active_kp = 10.0; 

    // 성능 측정용 버퍼
    std::vector<double> exec_times;

    std::cout << "===== 🚀 AttrJ 1000Hz Benchmarking & Goal Reach Start =====" << std::endl;
    std::cout << "Target Frequency: 1000 Hz | Kp: " << active_kp << std::endl;

    for (size_t i = 0; i < targets.size(); ++i) {
        std::cout << "\n[Step " << i + 1 << "] " << step_names[i] << " 기동 시작" << std::endl;
        
        // AttrJ 실행
        (void)traj_gen.attrj(targets[i], active_kp);

        double step_start_time = current_time;
        int loop_count = 0;
        
        // 해당 Step에서의 최대 물리량 측정용
        double max_v_seen = 0.0;
        double max_a_seen = 0.0;

        // --- 메인 제어 루프 ---
        while (true) {
            // 연산 시간 측정 시작
            auto start_tick = std::chrono::high_resolution_clock::now();

            traj_gen.update(dt); // 핵심 연산

            auto end_tick = std::chrono::high_resolution_clock::now();
            double elapsed_us = std::chrono::duration<double, std::micro>(end_tick - start_tick).count();
            
            exec_times.push_back(elapsed_us);

            // 실시간 물리량 및 오차 확인
            double q_err = (targets[i] - traj_gen.angles()).norm();
            double dq_norm = traj_gen.angvels().norm();
            double ddq_norm = traj_gen.angaccs().norm();

            max_v_seen = std::max(max_v_seen, dq_norm);
            max_a_seen = std::max(max_a_seen, ddq_norm);

            // 도착 판정 (임계값: 각도 0.1도, 속도 0.5deg/s)
            bool is_reached = traj_gen.goal_reached(std::nullopt, std::nullopt, 0.1, 0.5);

            // CSV 기록
            csv << std::fixed << std::setprecision(6)
                << current_time << "," << i + 1 << "," 
                << q_err << "," << dq_norm << "," << ddq_norm << "," 
                << elapsed_us << "," << (is_reached ? 1 : 0) << "\n";

            // 500ms 마다 모니터링 출력
            if (loop_count % 500 == 0) {
                std::cout << "[RUN] T: " << std::fixed << std::setprecision(3) << current_time 
                          << "s | Err: " << std::setprecision(3) << q_err 
                          << " | Exec: " << std::setw(6) << std::setprecision(1) << elapsed_us << " us" << std::endl;
            }

            if (is_reached && loop_count > 100) {
                std::cout << "✅ SUCCESS: 목표 지점 도달 완료!" << std::endl;
                break; 
            }

            current_time += dt;
            loop_count++;

            // 안전장치 (타임아웃 5초)
            if (current_time - step_start_time > 5.0) {
                std::cout << "⚠️ TIMEOUT: 도달 전 다음 단계 강제 전환" << std::endl;
                break;
            }
        }

        // --- 스텝 종료 후 물리량 및 성능 통계 출력 ---
        if (!exec_times.empty()) {
            double sum = std::accumulate(exec_times.begin(), exec_times.end(), 0.0);
            double avg = sum / exec_times.size();
            double max_exec = *std::max_element(exec_times.begin(), exec_times.end());
            
            std::cout << "------------------------------------------------------" << std::endl;
            std::cout << "📊 Step " << i + 1 << " Summary:" << std::endl;
            std::cout << "   - Actual Duration  : " << current_time - step_start_time << " s" << std::endl;
            std::cout << "   - Peak Velocity    : " << max_v_seen << " deg/s (Norm)" << std::endl;
            std::cout << "   - Peak Acceleration: " << max_a_seen << " deg/s^2 (Norm)" << std::endl;
            std::cout << "   - Avg Exec Time    : " << std::fixed << std::setprecision(2) << avg << " us" << std::endl;
            std::cout << "   - Worst Exec (Max) : " << max_exec << " us" << std::endl;
            std::cout << "   - CPU Margin (1ms) : " << (1.0 - avg/1000.0)*100.0 << " %" << std::endl;
            std::cout << "------------------------------------------------------" << std::endl;
        }
        exec_times.clear();
        
        // 안정화 구간
        for(int j=0; j<100; ++j) { traj_gen.update(dt); current_time += dt; }
    }

    csv.close();
    std::cout << "\n🏁 AttrJ 벤치마킹 종료. 'attrj_bench_report.csv' 저장됨." << std::endl;
    return 0;
}