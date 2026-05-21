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
    csv << time << "," << mode << ","
        << traj_gen.tmat().translation().x() << ","
        << traj_gen.tmat().translation().y() << ","
        << traj_gen.tmat().translation().z();
    
    for (int j = 0; j < 6; ++j) {
        csv << "," << traj_gen.angles()(j);
    }
    csv << "\n";
}

int main() {
    // 모델 초기화 (기존 코드 유지)
    rt_control::model::RobotModel model("m1013");
    TrajGenerator traj_gen;
    
    using tmat_t = Eigen::Isometry3d;
    using angles_t = Eigen::Matrix<double, 6, 1>;

    // ==========================================================
    // [초기 자세 설정] 요청하신 관절 각도 적용
    // ==========================================================
    angles_t q_home; 
    q_home << 97.64, -101.78, -91.5, -73.42, 88.9, 8.74;
    traj_gen.initialize(model, q_home, angles_t::Zero(), angles_t::Zero());

    std::ofstream csv("reach_and_lift_test.csv");
    csv << std::fixed << std::setprecision(6);
    csv << "Time,Mode,X,Y,Z,J1,J2,J3,J4,J5,J6\n";

    double dt = 0.001;
    double current_time = 0.0;
    double test_duration;
    double start_time;

    // ==========================================================
    // [Step 1] Home 위치로 이동
    // ==========================================================
    std::cout << "Step 1: 초기 설정된 Home 위치로 이동 (Joint Angles)" << std::endl;
    traj_gen.trapj(q_home);
    while (!traj_gen.goal_reached(0.1, 0.002)) { 
        traj_gen.update(dt); 
        log_data(csv, current_time, 0, traj_gen);
        current_time += dt; 
        if(current_time > 10.0) break; // 무한루프 방지
    }
    std::cout << " -> Goal Reached at Step 1!" << std::endl;

    // ==========================================================
    // [Step 2] X=0.075, Y=0.85, Z=0.3 위치로 이동
    // ==========================================================
    std::cout << "\nStep 2: Move to X=0.075, Y=0.85, Z=0.3" << std::endl;
    
    // 현재 TCP 회전 행렬 및 위치 복사 (Python의 self.robot.tmat.copy() 동일)
    tmat_t target_tmat = traj_gen.tmat(); 
    target_tmat.translation() << 0.075, 0.85, 0.3; // 목표 병진 좌표 설정
    
    // 이동 명령 및 예외 처리
    if (!traj_gen.attrl(target_tmat, 100.0)) {
        std::cout << "🚨 [FATAL] Step 2 목표 위치에 도달할 수 없습니다. 제어를 종료합니다." << std::endl;
        csv.close();
        return -1;
    }

    test_duration = 5.0; // 최대 5초 대기
    start_time = current_time;
    while (current_time - start_time < test_duration) {
        traj_gen.update(dt);
        
        if (std::isnan(traj_gen.angles()(0))) {
            std::cout << "🚨 [WARNING] IK Diverged at Step 2! Time: " << current_time << std::endl;
            break;
        }

        log_data(csv, current_time, 1, traj_gen);
        current_time += dt;
        
        if (traj_gen.goal_reached(0.01, 0.002)) { 
            std::cout << " -> Goal Reached at Step 2!" << std::endl;
            break;
        }
    }

    // ==========================================================
    // [Step 3] Z축을 0.0으로 하강 (Pick/Place 동작)
    // ==========================================================
    std::cout << "\nStep 3: Move Z axis down to 0.0m" << std::endl;
    
    // Step 2 완료 후의 현재 tmat을 다시 복사
    target_tmat = traj_gen.tmat();
    target_tmat.translation().z() = 0.0; // Z만 0.0으로 변경
    
    if (!traj_gen.attrl(target_tmat, 100.0)) {
         std::cout << "🚨 [WARNING] Step 3 하강 동작이 한계를 초과하여 스킵합니다." << std::endl;
    } else {
        test_duration = 5.0; 
        start_time = current_time;
        while (current_time - start_time < test_duration) {
            traj_gen.update(dt);
            
            if (std::isnan(traj_gen.angles()(0))) {
                std::cout << "🚨 [WARNING] IK Diverged at Step 3! Time: " << current_time << std::endl;
                break;
            }

            log_data(csv, current_time, 1, traj_gen);
            current_time += dt;
            
            if (traj_gen.goal_reached(0.01, 0.002)) {
                std::cout << " -> Goal Reached at Step 3!" << std::endl;
                break;
            }
        }
    }

    csv.close();
    std::cout << "\n✅ 로봇 타겟 이동 및 Z축 하강 시나리오 완료!" << std::endl;
    return 0;
}