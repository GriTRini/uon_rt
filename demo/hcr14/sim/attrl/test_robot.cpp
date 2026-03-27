#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>

#include "../../../../rt_control/model/model.hpp"
#include "../../../../rt_control/trajectory/trajectory_generator.hpp"

using namespace rt_control;

int main() {
    // 1. 모델 및 제너레이터 초기화 (hcr14 적용)
    model::RobotModel model("hcr14");
    trajectory::TrajGenerator traj_gen;
    
    // TCP 설정 (Z축 10cm, Roll 0도, Pitch 0도, Yaw 0도)
    double tx = 0.0, ty = 0.0, tz = 0.1, tr = 0.0, tp = 0.0, tyw = 0.0;
    traj_gen.set_tcp(tx, ty, tz, tr, tp, tyw);
    
    // 🌟 초기화 시점의 각도도 홈 포지션과 동일하게 맞춰주면 역학 계산이 더 안정적입니다.
    angles_t q_init;
    q_init << 0.0, -90.0, 90.0, 0.0, 0.0, 0.0;
    traj_gen.initialize(model, q_init, angles_t::Zero(), angles_t::Zero());

    double dt = 0.001; 
    
    // 출력 파일명 변경
    std::ofstream csv("hcr14_tcp_square_sim_data.csv");
    
    // 🌟 파이썬 코드 헤더에 완벽히 대응
    csv << "Scenario,Time,J1,J2,J3,J4,J5,J6,"
        << "Target_X,Target_Y,Target_Z,"
        << "TCP_X,TCP_Y,TCP_Z,TCP_R,TCP_P,TCP_Yw\n";

    double current_time = 0.0;
    Eigen::Isometry3d current_goal = Eigen::Isometry3d::Identity();

    // 데이터 기록용 람다 함수
    auto record_data = [&](const std::string& scenario) {
        const auto& q = traj_gen.angles();
        csv << scenario << "," << std::fixed << std::setprecision(6) << current_time << ",";
        for(int i=0; i<6; ++i) csv << q(i) << ",";
        // Target (빨간 별)
        csv << current_goal.translation().x() << "," << current_goal.translation().y() << "," << current_goal.translation().z() << ",";
        // TCP Offset (초록색 툴)
        csv << tx << "," << ty << "," << tz << "," << tr << "," << tp << "," << tyw << "\n";
    };

    // --- STEP 1: 시작 지점(Home)으로 이동 ---
    std::cout << "[Step 1] 지정된 홈 포지션으로 이동 중..." << std::endl;
    angles_t q_start_pos;
    
    // 🌟 HCR-14 전용 홈 포지션
    q_start_pos << 0.0, -90.0, 90.0, 0.0, 0.0, 0.0;
    current_goal = traj_gen.solve_forward(q_start_pos); // 목표 지점 설정

    if (traj_gen.trapj(q_start_pos)) {
        while (!traj_gen.goal_reached(0.1)) {
            traj_gen.update(dt);
            record_data("Setup_TrapJ");
            current_time += dt;
        }
        std::cout << "   -> 홈 포지션 도달 완료" << std::endl;
    }

    // --- STEP 2: 사각형 그리기 (attrl) ---
    Eigen::Isometry3d base_pose = traj_gen.tmat();
    
    // 20cm씩 이동하는 4개의 변
    std::vector<Eigen::Vector3d> offsets = {
        {0.2, 0.0, 0.0}, {0.0, 0.2, 0.0}, {-0.2, 0.0, 0.0}, {0.0, -0.2, 0.0}
    };

    std::cout << "[Step 2] 사각형 궤적 시작 (TCP 추종)..." << std::endl;
    Eigen::Isometry3d target_pose = base_pose;

    for (size_t i = 0; i < offsets.size(); ++i) {
        target_pose.translation() += offsets[i];
        current_goal = target_pose; // 타겟 위치 갱신
        
        std::string scenario_name = "Side_" + std::to_string(i + 1);
        
        // Kp 150.0으로 강하게 당김
        if (traj_gen.attrl(target_pose, 150.0)) { 
            // 허용 오차: 각도 0.1도, 위치 1cm(0.01m), 회전 2도
            while (!traj_gen.goal_reached(0.1, 0.01, 2.0)) { 
                traj_gen.update(dt);
                record_data(scenario_name);
                current_time += dt;
            }
            std::cout << "   -> Waypoint " << i + 1 << " 완료" << std::endl;
        }
    }

    csv.close();
    std::cout << "✅ HCR-14 시뮬레이션 완료: hcr14_tcp_square_sim_data.csv" << std::endl;
    return 0;
}