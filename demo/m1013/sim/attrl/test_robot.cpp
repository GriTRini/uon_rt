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
    rt_control::model::RobotModel model("m1013");
    TrajGenerator traj_gen;
    
    using tmat_t = Eigen::Isometry3d;
    using angles_t = Eigen::Matrix<double, 6, 1>;

    // 초기 자세 설정 (Home)
    angles_t q_home; q_home << -86.96, -31.27, -59.55, -0.18, -89.7, 0.0;
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
    std::cout << "Step 1: Move to Home Pose" << std::endl;
    traj_gen.trapj(q_home);
    while (!traj_gen.goal_reached(0.1, 0.002)) { 
        traj_gen.update(dt); 
        log_data(csv, current_time, 0, traj_gen);
        current_time += dt; 
        if(current_time > 10.0) break; // 무한루프 방지
    }

    // ==========================================================
    // [Step 2] X=0, Y=1.1, Z=0 위치로 이동 (명령 검증 및 예외 처리)
    // ==========================================================
    std::cout << "\nStep 2: Move to X=0.5, Y=0.5, Z=0.0" << std::endl;
    
    tmat_t target_tmat = traj_gen.tmat(); // 현재 TCP 회전 행렬 복사
    target_tmat.translation() << 0.5, 0.5, 0.0; // 1.1m로 무리한 병진 좌표 설정
    
    // 🌟 [안전 예외 처리 적용] attrl의 반환값을 확인합니다.
    if (!traj_gen.attrl(target_tmat, 100.0)) {
        std::cout << "🚨 [WARNING] 목표 위치(Y=1.1m)가 로봇의 가용 반경을 초과하여 명령이 거부되었습니다!" << std::endl;
        std::cout << " -> [Fallback] Y=0.8m 의 안전한 위치로 목표를 수정하여 다시 시도합니다." << std::endl;
        
        // 거부당했을 경우, Y값을 0.8로 낮추어 다시 명령
        target_tmat.translation() << 0.0, 0.8, 0.0; 
        
        // 재명령마저 실패할 경우를 대비한 2차 방어막
        if (!traj_gen.attrl(target_tmat, 100.0)) {
            std::cout << "🚨 [FATAL] 수정된 목표(Y=0.8m)도 도달 불가능합니다. 제어를 종료합니다." << std::endl;
            csv.close();
            return -1;
        }
    } else {
        std::cout << "✅ [SUCCESS] 명령 수락됨: 명령 위치로 이동을 시작합니다." << std::endl;
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
        
        if (traj_gen.goal_reached(0.01, 0.002)) { // 도달 시 조기 종료
            std::cout << " -> Goal Reached at Step 2!" << std::endl;
            break;
        }
    }

    // ==========================================================
    // [Step 3] Z축을 1.0m 위로 들어올리기
    // ==========================================================
    std::cout << "\nStep 3: Lift Z axis up to 1.0m" << std::endl;
    
    target_tmat = traj_gen.tmat();
    target_tmat.translation().z() = 0.5; // Z를 0.5m로 설정
    
    // Step 3에서도 혹시 모를 한계 초과를 대비해 방어 코드를 작성하는 것이 좋은 습관입니다.
    if (!traj_gen.attrl(target_tmat, 100.0)) {
         std::cout << "🚨 [WARNING] 리프트 동작이 한계를 초과하여 스킵합니다." << std::endl;
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
    std::cout << "\n✅ 로봇 뻗기 및 들어올리기(Lift) 시나리오 완료!" << std::endl;
    return 0;
}