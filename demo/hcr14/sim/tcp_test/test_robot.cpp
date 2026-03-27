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
    
    // 🌟 1. 매우 길고 복잡하게 꺾인 툴 설정
    double tx = -0.022, ty = -0.022, tz = 0.03, tr = 60.0, tp = 0.0, tyw = -45.0;
    traj_gen.set_tcp(tx, ty, tz, tr, tp, tyw);
    
    // 2. HCR-14 홈 포지션
    angles_t q_init;
    q_init << 0.0, -90.0, 90.0, 0.0, 0.0, 0.0;
    traj_gen.initialize(model, q_init, angles_t::Zero(), angles_t::Zero());

    double dt = 0.001; 
    
    // 3. 출력 파일명
    std::ofstream csv("hcr14_tcp_square_sim_data.csv");
    csv << "Scenario,Time,J1,J2,J3,J4,J5,J6,"
        << "Target_X,Target_Y,Target_Z,"
        << "TCP_X,TCP_Y,TCP_Z,TCP_R,TCP_P,TCP_Yw\n";

    double current_time = 0.0;
    Eigen::Isometry3d current_goal = Eigen::Isometry3d::Identity();

    // 4. 데이터 기록용 람다 함수
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
    q_start_pos << 0.0, -90.0, 90.0, 0.0, 0.0, 0.0;
    current_goal = traj_gen.solve_forward(q_start_pos);

    if (traj_gen.trapj(q_start_pos)) {
        while (!traj_gen.goal_reached(0.1)) {
            traj_gen.update(dt);
            record_data("Setup_TrapJ");
            current_time += dt;
        }
        std::cout << "   -> 홈 포지션 도달 완료" << std::endl;
    }

    // --- 🌟 STEP 2: 툴 팁을 바닥으로 수직 정렬 (-90도 오프셋 반영) ---
    std::cout << "\n[Step 2] 툴 팁을 바닥(-Z) 방향으로 정렬합니다..." << std::endl;
    if (traj_gen.align_tcp_to_floor(-90.0, 200.0)) {
        current_goal = traj_gen.tmat(); // 정렬 목표
        int step = 0;
        int max_steps = 8000; // 8초 제한
        while (!traj_gen.goal_reached(0.1, 0.015, 3.0) && step < max_steps) {
            traj_gen.update(dt);
            record_data("Align_Vertical");
            current_time += dt;
            step++;
        }
        std::cout << "   -> 바닥 정렬 완료" << std::endl;
    }

    // --- STEP 3: 사각형 그리기 (attrl) ---
    Eigen::Isometry3d base_pose = traj_gen.tmat();
    
    // 20cm씩 이동하는 4개의 변
    std::vector<Eigen::Vector3d> offsets = {
        {0.2, 0.0, 0.0}, {0.0, 0.2, 0.0}, {-0.2, 0.0, 0.0}, {0.0, -0.2, 0.0}
    };

    std::cout << "\n[Step 3] 사각형 궤적 시작 (TCP 추종)..." << std::endl;
    Eigen::Isometry3d target_pose = base_pose;

    for (size_t i = 0; i < offsets.size(); ++i) {
        target_pose.translation() += offsets[i];
        current_goal = target_pose; 
        
        std::string scenario_name = "Side_" + std::to_string(i + 1);
        
        // Kp 150.0으로 강하게 당김
        if (traj_gen.attrl(target_pose, 150.0)) { 
            while (!traj_gen.goal_reached(0.1, 0.01, 2.0)) { 
                traj_gen.update(dt);
                record_data(scenario_name);
                current_time += dt;
            }
            std::cout << "   -> Waypoint " << i + 1 << " 완료" << std::endl;
        }
    }

    csv.close();
    std::cout << "\n✅ HCR-14 시뮬레이션 완료: hcr14_tcp_square_sim_data.csv" << std::endl;
    return 0;
}