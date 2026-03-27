#include <iostream>
#include <fstream>
#include <iomanip>
#include "../../../../rt_control/model/model.hpp"
#include "../../../../rt_control/trajectory/trajectory_generator.hpp"

using namespace rt_control::trajectory::planning;

int main() {
    CartesianAttractor ghost;

    // 1. 초기화 (시작점: 원점)
    Eigen::Isometry3d start_pose = Eigen::Isometry3d::Identity();
    
    // 파라미터 주입 (Kp=50, Vel=0.5, AngVel=180deg, Acc=10, AngAcc=10)
    ghost.init(start_pose, 50.0, 0.5, 3.14159, 10.0, 10.0);

    // 2. 목표 설정 (대각선 10cm, 20cm, 30cm 이동 + 45도 회전)
    Eigen::Isometry3d goal_pose = Eigen::Isometry3d::Identity();
    goal_pose.translation() << 0.1, -1.0, 0.4;
    goal_pose.linear() = Eigen::AngleAxisd(M_PI/4.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    
    ghost.goal_pose = goal_pose;

    // 3. 시뮬레이션 및 데이터 기록
    std::ofstream csv("ghost_debug.csv");
    csv << "Time,X,Y,Z,Vel_X,Vel_Y,Vel_Z,Err_Norm\n";

    double dt = 0.001; // 1ms
    for (int i = 0; i < 2000; ++i) { // 2초간 시뮬레이션
        ghost.update(dt);

        double err = (ghost.goal_pose.translation() - ghost.pose.translation()).norm();

        csv << std::fixed << std::setprecision(6)
            << i * dt << ","
            << ghost.pose.translation().x() << ","
            << ghost.pose.translation().y() << ","
            << ghost.pose.translation().z() << ","
            << ghost.vel.head<3>().x() << ","
            << ghost.vel.head<3>().y() << ","
            << ghost.vel.head<3>().z() << ","
            << err << "\n";

        if (i % 200 == 0) {
            std::cout << "T: " << i*dt << "s | Pos: " << ghost.pose.translation().transpose() 
                      << " | Err: " << err << std::endl;
        }
    }

    csv.close();
    std::cout << "Ghost debug finished. Check 'ghost_debug.csv'" << std::endl;
    return 0;
}