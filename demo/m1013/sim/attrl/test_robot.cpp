#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>

#include "../../../../rt_control/model/model.hpp"
#include "../../../../rt_control/trajectory/trajectory_generator.hpp"

using namespace rt_control;

int main() {
    // 1. 모델 및 제너레이터 초기화 (m1013)
    model::RobotModel model("m1013");
    trajectory::TrajGenerator traj_gen;
    
    // TCP 설정 제거 완료
    angles_t q_init = angles_t::Zero();
    traj_gen.initialize(model, q_init, angles_t::Zero(), angles_t::Zero());

    double dt = 0.001; 
    std::ofstream csv("flange_fig8_sim_data.csv");
    
    // CSV 헤더에서 TCP 오프셋 부분 제거
    csv << "Scenario,Time,J1,J2,J3,J4,J5,J6,"
        << "Target_X,Target_Y,Target_Z\n";

    double current_time = 0.0;
    Eigen::Isometry3d current_goal = Eigen::Isometry3d::Identity();

    // 데이터 기록용 람다 함수 (TCP 데이터 기록 부분 제거)
    auto record_data = [&](const std::string& scenario) {
        const auto& q = traj_gen.angles();
        csv << scenario << "," << std::fixed << std::setprecision(6) << current_time << ",";
        for(int i=0; i<6; ++i) csv << q(i) << ",";
        // Target (빨간 별)
        csv << current_goal.translation().x() << "," << current_goal.translation().y() << "," << current_goal.translation().z() << "\n";
    };

    // --- STEP 1: 시작 지점으로 이동 ---
    angles_t q_start_pos;
    q_start_pos << 0.0, 0.0, -90.0, 0.0, -90.0, 0.0;
    current_goal = traj_gen.solve_forward(q_start_pos);

    if (traj_gen.trapj(q_start_pos)) {
        std::cout << "[Step 1] 시작 지점 이동 중..." << std::endl;
        while (!traj_gen.goal_reached(0.1)) {
            traj_gen.update(dt);
            record_data("Setup_TrapJ");
            current_time += dt;
        }
    }

    // --- STEP 2: 극한/반전 궤적 (Step Response & Limit Test) ---
    std::cout << "[Step 2] 극한/반전 궤적 추종 시작 (목표 순간이동)..." << std::endl;
    
    struct Waypoint {
        Eigen::Vector3d pos;
        Eigen::Vector3d rpy_deg;
        std::string name;
    };

    // 로봇을 극한으로 몰아붙이는 4개의 극단적 웨이포인트
    std::vector<Waypoint> extreme_waypoints = {
        {{0.7, -0.5, 0.2}, {180.0, 0.0, 0.0}, "Far_Right_Down"},
        {{0.5,  0.6, 0.8}, {0.0, -45.0, 90.0}, "Far_Left_Up"},
        {{-0.5, 0.0, 0.5}, {0.0, 90.0, 180.0}, "Opposite_Behind"},
        {{0.1,  0.0, 0.95}, {0.0, 180.0, 0.0}, "High_Close_Inverted"}
    };

    for (const auto& wp : extreme_waypoints) {
        Eigen::Isometry3d target_pose = Eigen::Isometry3d::Identity();
        target_pose.translation() = wp.pos;
        
        Eigen::AngleAxisd roll(wp.rpy_deg.x() * M_PI / 180.0, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch(wp.rpy_deg.y() * M_PI / 180.0, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yaw(wp.rpy_deg.z() * M_PI / 180.0, Eigen::Vector3d::UnitZ());
        target_pose.linear() = (yaw * pitch * roll).matrix();
        
        current_goal = target_pose;
        std::cout << "   -> Target 순간이동: " << wp.name << std::endl;
        
        if (!traj_gen.attrl(target_pose, 100.0)) {
            std::cerr << "궤적 생성 실패!" << std::endl;
            continue;
        }
        
        // 4초간 최대 속도로 쫓아가는 모습 관찰
        double step_time = 0.0;
        while (step_time < 4.0) {
            traj_gen.update(dt);
            record_data(wp.name);
            current_time += dt;
            step_time += dt;
        }
    }

    csv.close();
    std::cout << "✅ 극한 시뮬레이션 완료: flange_fig8_sim_data.csv" << std::endl;
    return 0;
}