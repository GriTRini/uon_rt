#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>
#include <chrono>
#include <numeric>
#include <algorithm>
#include "../../../../rt_control/model/model.hpp"
#include "../../../../rt_control/trajectory/trajectory_generator.hpp"

using namespace rt_control;
using namespace rt_control::trajectory;

int main() {
    // 1. 로봇 모델 및 타겟 설정
    rt_control::model::RobotModel model("m1013");
    
    angles_t q_start;   q_start   << 0, 0, 0, 0, 0, 0;
    angles_t q_target1; q_target1 << 0, 0, 90, 0, 0, 0; 
    angles_t q_target2; q_target2 << 0, 0, 0, 0, 0, 0; // J6 이동으로 수정 (설명 기반)
    angles_t q_target3; q_target3 << 0, 0, 90, 0, 0, 0;

    // 2. 제너레이터 초기화
    TrajGenerator traj_gen;
    traj_gen.initialize(model, q_start, angles_t::Zero(), angles_t::Zero());

    // 3. 디버깅용 CSV 설정 (모든 조인트 컬럼 추가)
    std::ofstream csv("trapj_bench_data.csv");
    csv << "Time,Step,J1,J2,J3,J4,J5,J6,Q_Error,Exec_us,Reached\n";

    double dt = 0.001; 
    double current_time = 0.0;
    std::vector<angles_t> test_goals = {q_target1, q_target2, q_target3};
    
    std::vector<double> exec_times; 
    
    std::cout << "===== 🚀 1000Hz Loop & All-Joint Monitoring Benchmarking =====" << std::endl;
    std::cout << "Target Frequency: 1000 Hz (Cycle: 1000 us)" << std::endl;

    for (size_t i = 0; i < test_goals.size(); ++i) {
        (void)traj_gen.trapj(test_goals[i]);

        std::cout << "\n[Step " << i + 1 << "] 기동 시작 -> Target: " << test_goals[i].transpose() << std::endl;
        
        double step_start_time = current_time;
        int loop_count = 0;

        while (!traj_gen.goal_reached(0.05, std::nullopt, std::nullopt, 0.1)) {
            auto start_tick = std::chrono::high_resolution_clock::now();

            traj_gen.update(dt); 

            auto end_tick = std::chrono::high_resolution_clock::now();
            double elapsed_us = std::chrono::duration<double, std::micro>(end_tick - start_tick).count();
            
            exec_times.push_back(elapsed_us);
            
            // 데이터 확보
            angles_t cur_q = traj_gen.angles();
            double q_err = traj_gen.angles_enorm().value_or(0.0);

            // CSV 기록 (모든 조인트 각도 기록)
            csv << std::fixed << std::setprecision(4) << current_time << "," << i + 1 << ",";
            for(int j=0; j<6; ++j) csv << cur_q(j) << ",";
            csv << q_err << "," << elapsed_us << "," << (traj_gen.goal_reached() ? 1 : 0) << "\n";
            
            // 500ms마다 실시간 모니터링 출력
            if (loop_count % 200 == 0) {
                std::cout << "[RUN] T: " << std::fixed << std::setprecision(3) << current_time << "s | ";
                std::cout << "Q: [";
                for(int j=0; j<6; ++j) {
                    std::cout << std::setw(7) << std::fixed << std::setprecision(2) << cur_q(j) << (j==5 ? "" : ", ");
                }
                std::cout << "] | Err: " << std::setprecision(3) << q_err 
                          << " | Exec: " << std::setw(5) << (int)elapsed_us << " us" << std::endl;
            }

            current_time += dt;
            loop_count++;
            
            if (current_time - step_start_time > 10.0) break;
        }

        // 성능 통계
        if (!exec_times.empty()) {
            double sum = std::accumulate(exec_times.begin(), exec_times.end(), 0.0);
            double avg = sum / exec_times.size();
            double max_t = *std::max_element(exec_times.begin(), exec_times.end());
            
            std::cout << "--------------------------------------------------------------------------------" << std::endl;
            std::cout << "✅ Step " << i + 1 << " Done. (Final Q: " << traj_gen.angles().transpose() << ")" << std::endl;
            std::cout << "📊 Stats: Avg Exec: " << std::fixed << std::setprecision(2) << avg 
                      << " us | Max: " << max_t << " us | Margin: " << (1.0 - avg/1000.0)*100.0 << " %" << std::endl;
            std::cout << "--------------------------------------------------------------------------------" << std::endl;
        }
        exec_times.clear();
        
        for(int j=0; j<200; ++j) { traj_gen.update(dt); current_time += dt; }
    }

    csv.close();
    std::cout << "\n🏁 Benchmarking Finished. Data saved to 'trapj_bench_data.csv'" << std::endl;
    return 0;
}