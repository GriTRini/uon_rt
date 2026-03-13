#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <cmath>

#include "../rt_control/core/core.hpp"
#include "../rt_control/model/model.hpp"
#include "../rt_control/trajectory/trajectory_trapj.hpp"
#include "../rt_control/trajectory/trajectory_attrl.hpp"

using namespace rt_control;
using namespace rt_control::trajectory;
using namespace rt_control::model;

int main() {
    RobotModel my_robot("m1013");

    std::ofstream csv_file("fk_output.csv");
    csv_file << "time";
    for(int i=1; i<=6; ++i) csv_file << ",q" << i;
    for(int i=0; i<=6; ++i) csv_file << ",x" << i << ",y" << i << ",z" << i;
    csv_file << "\n";

    double dt = 0.001;
    double current_time = 0.0;

    // ==========================================================
    // Stage 1: TrapJ로 'Ready Pose' 만들기
    // ==========================================================
    std::cout << "--- 새롭게 설계된 AttrL 디버깅 시작 ---" << std::endl;
    std::cout << "[Stage 1] TrapJ를 사용하여 준비 자세로 이동..." << std::endl;
    
    TrajTrapJ::angles_t ready_q = TrajTrapJ::angles_t::Zero();
    ready_q(1) = -M_PI / 6; ready_q(2) = M_PI / 2; ready_q(3) = -M_PI / 3;     

    TrajTrapJ trapj(
        &my_robot, TrajTrapJ::angles_t::Zero(), TrajTrapJ::angles_t::Zero(), 
        ready_q, TrajTrapJ::angles_t::Zero(),
        my_robot.get_max_angvels(), my_robot.get_max_angaccs() // 정상 속도 주입
    );

    while (!trapj.goal_reached()) {
        auto q = trapj.angles();
        std::vector<Eigen::Isometry3d> poses = my_robot.forward_kinematics_all(q);

        csv_file << std::fixed << std::setprecision(4) << current_time;
        for(int i=0; i<6; ++i) csv_file << "," << std::setprecision(6) << q(i);
        for(int i=0; i<=6; ++i) csv_file << "," << poses[i].translation().x() << "," << poses[i].translation().y() << "," << poses[i].translation().z();
        csv_file << "\n";

        trapj.update(dt);
        current_time += dt;
    }
    
    while(current_time < 2.0) { current_time += dt; } // 2초까지 대기

    // ==========================================================
    // Stage 2: AttrL로 허공에 사각형 그리기
    // ==========================================================
    std::cout << "\n[Stage 2] AttrL 사각형 웨이포인트 추적 시작!" << std::endl;

    // 🌟 가혹 조건: 끝단 선속도를 0.1m/s 로 매우 느리게 제한해봅니다. (속도 클리핑 확인용)
    TrajAttrL attrl(&my_robot, trapj.angles(), TrajAttrL::angles_t::Zero(), TrajAttrL::angles_t::Zero(), 
                    0.1, M_PI, 10.0, M_PI*2); 
    
    Eigen::Isometry3d base_pose = my_robot.forward_kinematics(trapj.angles());
    std::vector<Eigen::Isometry3d> waypoints;
    
    Eigen::Isometry3d wp1 = base_pose; wp1.translation() += Eigen::Vector3d(0.2, 0.0, 0.2); waypoints.push_back(wp1);
    Eigen::Isometry3d wp2 = wp1;       wp2.translation() += Eigen::Vector3d(0.0, 0.3, 0.0); waypoints.push_back(wp2);
    Eigen::Isometry3d wp3 = wp2;       wp3.translation() += Eigen::Vector3d(0.0, 0.0, -0.2); waypoints.push_back(wp3);
    waypoints.push_back(base_pose);

    attrl.set_kp_cartesian(20.0); 

    for (size_t i = 0; i < waypoints.size(); ++i) {
        attrl.set_goal_pose(waypoints[i]);
        double target_time = current_time + 1.5; // 각 구간당 1.5초 할당
        
        while (current_time <= target_time) {
            attrl.update(dt);
            auto q = attrl.angles();
            std::vector<Eigen::Isometry3d> poses = my_robot.forward_kinematics_all(q);

            csv_file << std::fixed << std::setprecision(4) << current_time;
            for(int j=0; j<6; ++j) csv_file << "," << std::setprecision(6) << q(j);
            for(int j=0; j<=6; ++j) csv_file << "," << poses[j].translation().x() << "," << poses[j].translation().y() << "," << poses[j].translation().z();
            csv_file << "\n";

            current_time += dt;
        }
    }

    csv_file.close();
    std::cout << "완료! 파이썬 스크립트로 확인하세요." << std::endl;
    return 0;
}