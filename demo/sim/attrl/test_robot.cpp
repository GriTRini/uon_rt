#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>

#include "../../../rt_control/model/model.hpp"
#include "../../../rt_control/trajectory/trajectory_generator.hpp"

using namespace rt_control;

int main() {
    // 1. 모델 및 제너레이터 초기화
    model::RobotModel model("m1013");
    trajectory::TrajGenerator traj_gen;
    
    // TCP 설정 (Z축 10cm, Roll 45도 비틀림 예시)
    double tx = 0.0, ty = 0.0, tz = 0.1, tr = 0.0, tp = 0.0, tyw = 0.0;
    traj_gen.set_tcp(tx, ty, tz, tr, tp, tyw);
    
    angles_t q_init = angles_t::Zero();
    traj_gen.initialize(model, q_init, angles_t::Zero(), angles_t::Zero());

    double dt = 0.001; 
    std::ofstream csv("tcp_square_sim_data.csv");
    
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

    // --- STEP 1: 시작 지점으로 이동 ---
    angles_t q_start_pos;
    q_start_pos << 0.0, 0.0, -90.0, 0.0, -90.0, 0.0;
    current_goal = traj_gen.solve_forward(q_start_pos); // 목표 지점 설정

    if (traj_gen.trapj(q_start_pos)) {
        std::cout << "[Step 1] 시작 지점 이동 중..." << std::endl;
        while (!traj_gen.goal_reached(0.1)) {
            traj_gen.update(dt);
            record_data("Setup_TrapJ");
            current_time += dt;
        }
    }

    // --- STEP 2: 사각형 그리기 ---
    Eigen::Isometry3d base_pose = traj_gen.tmat();
    std::vector<Eigen::Vector3d> offsets = {
        {0.2, 0.0, 0.0}, {0.0, 0.2, 0.0}, {-0.2, 0.0, 0.0}, {0.0, -0.2, 0.0}
    };

    std::cout << "[Step 2] 사각형 궤적 시작..." << std::endl;
    Eigen::Isometry3d target_pose = base_pose;

    for (size_t i = 0; i < offsets.size(); ++i) {
        target_pose.translation() += offsets[i];
        current_goal = target_pose; // 타겟 별표 위치 갱신
        
        std::string scenario_name = "Side_" + std::to_string(i + 1);
        if (traj_gen.attrl(target_pose, 150.0)) { // 추종 강도 상향
            while (!traj_gen.goal_reached(0.1, 0.01, 2.0)) { // 도달 판정 완화 (1cm)
                traj_gen.update(dt);
                record_data(scenario_name);
                current_time += dt;
            }
            std::cout << "   -> Waypoint " << i + 1 << " 완료" << std::endl;
        }
    }

    csv.close();
    std::cout << "✅ 시뮬레이션 완료: tcp_square_sim_data.csv" << std::endl;
    return 0;
}