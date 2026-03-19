#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>
#include <chrono>   // 시간 측정용
#include <numeric>  // 통계 계산용
#include <algorithm> // max_element용
#include "../../../rt_control/model/model.hpp"
#include "../../../rt_control/trajectory/trajectory_generator.hpp"

using namespace rt_control;
using namespace rt_control::trajectory;

int main() {
    // 1. 로봇 모델 및 타겟 설정
    rt_control::model::RobotModel model("m1013");
    
    angles_t q_start;   q_start   << 0, 0, 0, 0, 0, 0;
    angles_t q_target1; q_target1 << 90, 15, -60, 30, -45, 180;
    angles_t q_target2; q_target2 << -90, -20, 80, -90, 90, -170;
    angles_t q_target3; q_target3 << 10, 45, 10, 0, 30, 45;

    // 2. 제너레이터 초기화
    TrajGenerator traj_gen;
    traj_gen.initialize(model, q_start, angles_t::Zero(), angles_t::Zero());

    // 3. 디버깅용 CSV 설정
    std::ofstream csv("trapj_bench_data.csv");
    csv << "Time,Step,Q_Error,Exec_us,Reached\n";

    double dt = 0.001; // 목표 제어 주기: 1ms (1000Hz)
    double current_time = 0.0;
    std::vector<angles_t> test_goals = {q_target1, q_target2, q_target3};
    
    // 성능 측정을 위한 버퍼
    std::vector<double> exec_times; 
    
    std::cout << "===== 🚀 1000Hz Loop & Goal Reach Benchmarking =====" << std::endl;
    std::cout << "Target Frequency: 1000 Hz (Cycle: 1000 us)" << std::endl;

    for (size_t i = 0; i < test_goals.size(); ++i) {
        (void)traj_gen.trapj(test_goals[i]);

        std::cout << "\n[Step " << i + 1 << "] 기동 시작" << std::endl;
        
        double step_start_time = current_time;
        int loop_count = 0;

        // --- 메인 제어 루프 ---
        while (!traj_gen.goal_reached(0.05, std::nullopt, std::nullopt, 0.1)) {
            // 연산 시간 측정 시작
            auto start_tick = std::chrono::high_resolution_clock::now();

            // 핵심 연산: 궤적 업데이트
            traj_gen.update(dt); 

            // 연산 시간 측정 종료
            auto end_tick = std::chrono::high_resolution_clock::now();
            double elapsed_us = std::chrono::duration<double, std::micro>(end_tick - start_tick).count();
            
            exec_times.push_back(elapsed_us);
            double q_err = traj_gen.angles_enorm().value_or(0.0);

            // CSV 기록
            csv << std::fixed << std::setprecision(4) 
                << current_time << "," << i + 1 << "," 
                << q_err << "," << elapsed_us << ","
                << (traj_gen.goal_reached() ? 1 : 0) << "\n";
            
            // 500ms마다 실시간 주기 모니터링 출력
            if (loop_count % 500 == 0) {
                std::cout << "[RUN] T: " << std::fixed << std::setprecision(3) << current_time 
                          << "s | Err: " << std::setprecision(3) << q_err 
                          << " | Exec: " << std::setw(6) << std::setprecision(1) << elapsed_us << " us" << std::endl;
            }

            current_time += dt;
            loop_count++;
            
            // 타임아웃 10초
            if (current_time - step_start_time > 10.0) break;
        }

        // --- 스텝 종료 후 성능 통계 출력 ---
        if (!exec_times.empty()) {
            double sum = std::accumulate(exec_times.begin(), exec_times.end(), 0.0);
            double avg = sum / exec_times.size();
            double max_t = *std::max_element(exec_times.begin(), exec_times.end());
            
            std::cout << "------------------------------------------------------" << std::endl;
            std::cout << "✅ Step " << i + 1 << " Done. (Duration: " << current_time - step_start_time << "s)" << std::endl;
            std::cout << "📊 Performance Report:" << std::endl;
            std::cout << "   - Average Execution: " << std::fixed << std::setprecision(2) << avg << " us" << std::endl;
            std::cout << "   - Worst Case (Max) : " << std::fixed << std::setprecision(2) << max_t << " us" << std::endl;
            std::cout << "   - CPU Margin (1ms) : " << std::fixed << std::setprecision(1) << (1.0 - avg/1000.0)*100.0 << " %" << std::endl;
            std::cout << "------------------------------------------------------" << std::endl;
        }
        exec_times.clear();
        
        // 안정화 대기 구간
        for(int j=0; j<200; ++j) { traj_gen.update(dt); current_time += dt; }
    }

    csv.close();
    std::cout << "\n🏁 Benchmarking Finished. Check 'trapj_bench_data.csv'" << std::endl;
    return 0;
}