#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include "../../../../rt_control/model/model.hpp"
#include "../../../../rt_control/trajectory/trajectory_generator.hpp"

using namespace rt_control;
using namespace rt_control::trajectory;

// --- 공통 로깅 함수 ---
void log_data(std::ofstream& csv, double time, int mode, const TrajGenerator& traj_gen) {
    // TCP와 Flange의 병진(Translation) 좌표 추출
    auto tcp_pos = traj_gen.tmat().translation();
    auto flange_pos = traj_gen.flange_tmat().translation();

    csv << time << "," << mode << ","
        << tcp_pos.x() << "," << tcp_pos.y() << "," << tcp_pos.z() << ","
        << flange_pos.x() << "," << flange_pos.y() << "," << flange_pos.z();
    
    for (int j = 0; j < 6; ++j) {
        csv << "," << traj_gen.angles()(j);
    }
    csv << "\n";
}

int main() {
    rt_control::model::RobotModel model("hcr14");
    TrajGenerator traj_gen;
    
    using tmat_t = Eigen::Isometry3d;
    using angles_t = Eigen::Matrix<double, 6, 1>;

    // ==========================================================
    // [초기 자세 설정]
    // ==========================================================
    angles_t q_home; 
    q_home << 97.64, -101.78, -91.5, -73.42, 88.9, 8.74;
    traj_gen.initialize(model, q_home, angles_t::Zero(), angles_t::Zero());

    // 🌟 [핵심 추가 사항] TCP 설정: Z축으로 0.25m(25cm) 추가
    traj_gen.set_tcp(0.0, 0.0, 0.25, 0.0, 0.0, 0.0);
    std::cout << "✅ TCP 설정 완료: Z축 +0.25m 오프셋 적용" << std::endl;

    std::ofstream csv("reach_and_lift_test.csv");
    csv << std::fixed << std::setprecision(6);
    // CSV 헤더에 Flange_X, Flange_Y, Flange_Z 추가
    csv << "Time,Mode,TCP_X,TCP_Y,TCP_Z,Flange_X,Flange_Y,Flange_Z,J1,J2,J3,J4,J5,J6\n";

    double dt = 0.001;
    double current_time = 0.0;
    double test_duration;
    double start_time;

    // ==========================================================
    // [Step 1] 지정된 위치로 이동
    // ==========================================================
    std::cout << "\nStep 1: 초기 설정된 위치로 이동" << std::endl;
    traj_gen.trapj(q_home);
    while (!traj_gen.goal_reached(0.1, 0.002)) { 
        traj_gen.update(dt); 
        log_data(csv, current_time, 0, traj_gen);
        current_time += dt; 
        if(current_time > 10.0) break; 
    }
    std::cout << " -> Goal Reached at Step 1!" << std::endl;

    // ==========================================================
    // [Step 2] X=0.075, Y=0.85, Z=0.3 위치로 이동 (TCP 기준)
    // ==========================================================
    std::cout << "\nStep 2: Move to X=0.075, Y=0.85, Z=0.3" << std::endl;
    
    tmat_t target_tmat = traj_gen.tmat(); 
    target_tmat.translation() << 0.075, 0.85, 0.3; 
    
    traj_gen.attrl(target_tmat, 100.0);

    test_duration = 5.0; 
    start_time = current_time;
    while (current_time - start_time < test_duration) {
        traj_gen.update(dt);
        log_data(csv, current_time, 1, traj_gen);
        current_time += dt;
        
        if (traj_gen.goal_reached(0.01, 0.002)) { 
            std::cout << " -> Goal Reached at Step 2!" << std::endl;
            break;
        }
    }

    // ==========================================================
    // [Step 3] Z축을 0.0으로 하강 (TCP 기준)
    // ==========================================================
    std::cout << "\nStep 3: Move Z axis down to 0.0m" << std::endl;
    
    target_tmat = traj_gen.tmat();
    target_tmat.translation().z() = 0.0; 
    
    traj_gen.attrl(target_tmat, 100.0);

    test_duration = 5.0; 
    start_time = current_time;
    while (current_time - start_time < test_duration) {
        traj_gen.update(dt);
        log_data(csv, current_time, 1, traj_gen);
        current_time += dt;
        
        if (traj_gen.goal_reached(0.01, 0.002)) {
            std::cout << " -> Goal Reached at Step 3!" << std::endl;
            break;
        }
    }

    csv.close();
    std::cout << "\n✅ 시나리오 완료!" << std::endl;
    return 0;
}