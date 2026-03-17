#include <iostream>
#include <iomanip>
#include <Eigen/Dense>

// 헤더 파일들은 기존 경로 유지
#include "../rt_control/model/model.hpp"
#include "../rt_control/trajectory/trajectory_attrl.hpp"

using namespace rt_control;

int main() {
    // 1. 모델 초기화 (m1013)
    model::RobotModel robot_model("m1013");
    
    // 2. 초기 상태 설정 (J3 = 90도 자세)
    angles_t q_start = angles_t::Zero();
    q_start(2) = 90.0;
    q_start(4) = 90.0;
    angles_t zero = angles_t::Zero();

    // 3. Attrl 생성 (Kp=50, Vel=0.5m/s, Acc=2.0m/s^2)
    // 생성자에서 부모(Base)에게 모델의 100% 성능을 넘기는 로직이 작동하는지 확인
    trajectory::TrajAttrL attrl(&robot_model, q_start, zero, zero, 0.5, M_PI, 2.0, M_PI*2);
    
    // 4. 목표 설정: 현재 위치에서 Z축 +100mm (0.1m) 상승
    Eigen::Isometry3d goal_pose = robot_model.forward_kinematics(q_start);
    double start_z = goal_pose.translation().z();
    goal_pose.translation().z() += 0.1; 
    
    attrl.set_goal_pose(goal_pose);
    attrl.set_kp_cartesian(500.0);

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "====================================================" << std::endl;
    std::cout << "       Attrl Logic Numerical Debugging              " << std::endl;
    std::cout << "====================================================" << std::endl;
    std::cout << "Start Z : " << start_z << " m" << std::endl;
    std::cout << "Target Z: " << goal_pose.translation().z() << " m" << std::endl;
    std::cout << "====================================================" << std::endl;

    double dt = 0.001; // 1ms 제어 주기
    for (int i = 0; i <= 200; ++i) { // 초기 0.2초간의 거동 관찰
        double z_before = attrl.current_pose().translation().z();
        
        // 🌟 핵심: Attrl의 내부 상태 업데이트
        attrl.update(dt);
        
        double z_after = attrl.current_pose().translation().z();
        double step_move_mm = (z_after - z_before) * 1000.0; // mm 단위

        // 20ms마다 상세 수치 출력
        if (i % 20 == 0) {
            double error_mm = (goal_pose.translation().z() - z_after) * 1000.0;
            std::cout << "[Step " << std::setw(3) << i << "ms] "
                      << "현재 Z: " << z_after 
                      << " | 남은 오차: " << std::setw(9) << error_mm << " mm"
                      << " | 이동량: " << std::setw(9) << step_move_mm << " mm/step" << std::endl;
            
            // 만약 이동량이 너무 작으면 원인 진단
            if (step_move_mm < 0.01) {
                std::cout << "  -> ⚠️ 경고: 이동량이 너무 작음! (가속도 제한 확인 필요)" << std::endl;
            }
        }
    }

    return 0;
}