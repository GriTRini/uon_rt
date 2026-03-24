#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <Eigen/Dense>

#include "../../../rt_control/model/model.hpp"
#include "../../../rt_control/trajectory/trajectory_generator.hpp"

using namespace rt_control;

int main() {
    // 1. 모델 및 제너레이터 초기화 (m1013)
    model::RobotModel model("m1013");
    trajectory::TrajGenerator traj_gen;
    
    // 초기화: TCP 설정 없이 Flange 기준 (Identity)
    angles_t q_init = angles_t::Zero();
    traj_gen.initialize(model, q_init, angles_t::Zero(), angles_t::Zero());

    double dt = 0.001; // 1ms
    std::ofstream csv("robot_joint_data.csv");
    // 헤더: 시간, 조인트 1~6, 말단 X, Y, Z
    csv << "Time,J1,J2,J3,J4,J5,J6,X,Y,Z\n";

    // --- STEP 1: 사각형 시작 지점으로 이동 (trapj) ---
    angles_t q_start_pos;
    q_start_pos << 0.0, 0.0, -90.0, 0.0, -90.0, 0.0;
    
    std::cout << "[Step 1] 시작 지점으로 trapj 이동 중..." << std::endl;
    // 🌟 [[nodiscard]] 경고 해결: if 문으로 체크
    if (!traj_gen.trapj(q_start_pos)) {
        std::cerr << "Error: trapj 초기화 실패" << std::endl;
        return -1;
    }
    
    double current_time = 0.0;
    while (!traj_gen.goal_reached(0.1)) {
        traj_gen.update(dt);
        const auto& q = traj_gen.angles();
        const auto& p = traj_gen.tmat().translation();
        csv << std::fixed << std::setprecision(6) 
            << current_time << "," << q(0) << "," << q(1) << "," << q(2) << "," 
            << q(3) << "," << q(4) << "," << q(5) << "," << p.x() << "," << p.y() << "," << p.z() << "\n";
        current_time += dt;
    }

    // --- STEP 2: 네모 그리기 (attrl x 4) ---
    Eigen::Isometry3d current_pose = traj_gen.tmat();
    
    // 사각형 경로 설정 (상대 좌표: 0.2m x 0.2m)
    // 현재 자세를 유지하며 위치만 이동
    std::vector<Eigen::Vector3d> waypoints = {
        current_pose.translation() + Eigen::Vector3d(0.2, 0.0, 0.0),  // 우측
        current_pose.translation() + Eigen::Vector3d(0.2, 0.2, 0.0),  // 앞
        current_pose.translation() + Eigen::Vector3d(0.0, 0.2, 0.0),  // 좌측
        current_pose.translation()                                    // 복귀
    };

    std::cout << "[Step 2] attrl 사각형 궤적 생성 시작..." << std::endl;

    for (size_t i = 0; i < waypoints.size(); ++i) {
        Eigen::Isometry3d target_pose = current_pose;
        target_pose.translation() = waypoints[i];
        
        if (!traj_gen.attrl(target_pose, 40.0)) {
            std::cerr << "Error: attrl 변 " << i << " 생성 실패" << std::endl;
            break;
        }

        // 도달 판정 (위치 2mm, 회전 1도 이내)
        while (!traj_gen.goal_reached(0.1, 0.002, 1.0)) {
            traj_gen.update(dt);
            const auto& q = traj_gen.angles();
            const auto& p = traj_gen.tmat().translation();
            csv << std::fixed << std::setprecision(6) 
                << current_time << "," << q(0) << "," << q(1) << "," << q(2) << "," 
                << q(3) << "," << q(4) << "," << q(5) << "," << p.x() << "," << p.y() << "," << p.z() << "\n";
            current_time += dt;
        }
        std::cout << "   -> Waypoint " << i + 1 << " 도달 완료" << std::endl;
    }

    csv.close();
    std::cout << "✅ 데이터 저장 완료: robot_joint_data.csv" << std::endl;
    return 0;
}