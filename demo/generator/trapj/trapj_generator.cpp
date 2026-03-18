#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include "../../../rt_control/model/model.hpp"
#include "../../../rt_control/trajectory/trajectory_generator.hpp"

using namespace rt_control;
using namespace rt_control::trajectory;

int main() {
    rt_control::model::RobotModel model("m1013");
    
    angles_t q_start;   q_start   << 0, 0, 0, 0, 0, 0;
    angles_t q_target1; q_target1 << 90, 15, -60, 30, -45, 180;
    angles_t q_target2; q_target2 << -90, -20, 80, -90, 90, -170;
    angles_t q_target3; q_target3 << 10, 45, 10, 0, 30, 45;

    TrajGenerator traj_gen;
    traj_gen.initialize(model, q_start, angles_t::Zero(), angles_t::Zero());

    std::ofstream csv("trapj_debug_data.csv");
    csv << "Time,Step,J1_Pos,J2_Pos,J3_Pos,J4_Pos,J5_Pos,J6_Pos,Reached\n";

    double dt = 0.001; 
    double current_time = 0.0;
    std::vector<angles_t> test_goals = {q_target1, q_target2, q_target3};
    
    std::cout << "===== 🛠️ Complex TrapJ Sequence Start =====" << std::endl;

    for (size_t i = 0; i < test_goals.size(); ++i) {
        std::cout << "\n🚀 [Step " << i + 1 << "] 기동: " << test_goals[i].transpose() << std::endl;
        
        traj_gen.trapj(test_goals[i]);

        double step_start_time = current_time;
        int loop_count = 0;

        while (!traj_gen.goal_reached()) {
            traj_gen.update(dt);
            loop_count++;
            
            // CSV 기록
            csv << std::fixed << std::setprecision(4) 
                << current_time << "," << i + 1 << "," 
                << traj_gen.angles()(0) << "," << traj_gen.angles()(1) << ","
                << traj_gen.angles()(2) << "," << traj_gen.angles()(3) << ","
                << traj_gen.angles()(4) << "," << traj_gen.angles()(5) << ","
                << (traj_gen.goal_reached() ? 1 : 0) << "\n";
            
            // 200ms마다 J1 ~ J6 전체 관절 모니터링 출력
            if (loop_count % 200 == 0) {
                std::cout << "[RUN] T: " << std::fixed << std::setprecision(3) << current_time << "s | "
                        << "J1:" << std::setw(7) << std::setprecision(2) << traj_gen.angles()(0) << " | "
                        << "J2:" << std::setw(7) << traj_gen.angles()(1) << " | "
                        << "J3:" << std::setw(7) << traj_gen.angles()(2) << " | "
                        << "J4:" << std::setw(7) << traj_gen.angles()(3) << " | "
                        << "J5:" << std::setw(7) << traj_gen.angles()(4) << " | "
                        << "J6:" << std::setw(7) << traj_gen.angles()(5) << std::endl;
            }

            current_time += dt;
            if (current_time - step_start_time > 8.0) break; // 복잡한 기동이므로 타임아웃 8초 확대
        }
        std::cout << "✅ Step " << i + 1 << " Done. (Duration: " << current_time - step_start_time << "s)" << std::endl;
        
        // 안정화 대기
        for(int j=0; j<200; ++j) { traj_gen.update(dt); current_time += dt; }
    }

    csv.close();
    return 0;
}