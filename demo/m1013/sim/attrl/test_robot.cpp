#include <iostream>
#include <fstream>
#include <iomanip>
#include "../../../../rt_control/model/model.hpp"
#include "../../../../rt_control/trajectory/trajectory_generator.hpp"

using namespace rt_control;
using namespace rt_control::trajectory;

int main() {
    rt_control::model::RobotModel model("m1013");
    TrajGenerator traj_gen;
    
    using tmat_t = TrajGenerator::tmat_t;
    using angles_t = TrajGenerator::angles_t;

    // 초기화 (홈 포인트)
    angles_t q_home; q_home << -90, 0, -90, 0, -90, 0;
    traj_gen.initialize(model, q_home, angles_t::Zero(), angles_t::Zero());

    std::ofstream csv("robot_dynamics_data.csv");
    // 헤더 생성 (Time, Mode, J1_p, J1_v, J1_a, ..., J6_a, X, Y, Z)
    csv << "Time,Mode";
    for(int i=1; i<=6; ++i) csv << ",J" << i << "_p,J" << i << "_v,J" << i << "_a";
    csv << ",X,Y,Z\n";

    double dt = 0.001;
    double current_time = 0.0;

    // --- STEP 1: AttrL 이동 ---
    tmat_t target_pose = traj_gen.tmat();
    target_pose.translation() << 0.0, 1.0, 0.045;
    (void)traj_gen.attrl(target_pose, 60.0);

    while (!traj_gen.goal_reached(std::nullopt, 0.002, 1.0)) {
        traj_gen.update(dt);
        csv << current_time << ",1"; // Mode 1: AttrL
        for(int j=0; j<6; ++j) csv << "," << traj_gen.angles()(j) << "," << traj_gen.angvels()(j) << "," << traj_gen.angaccs()(j);
        csv << "," << traj_gen.tmat().translation().transpose().format(Eigen::IOFormat(5, 0, ",", ",", "", "", "", "")) << "\n";
        current_time += dt;
    }

    // --- STEP 2: 안정화 (E-Stop 방지) ---
    traj_gen.stop();
    for(int i=0; i<500; ++i) {
        traj_gen.update(dt);
        csv << current_time << ",2"; // Mode 2: Stop/Wait
        for(int j=0; j<6; ++j) csv << "," << traj_gen.angles()(j) << "," << traj_gen.angvels()(j) << "," << traj_gen.angaccs()(j);
        csv << "," << traj_gen.tmat().translation().transpose().format(Eigen::IOFormat(5, 0, ",", ",", "", "", "", "")) << "\n";
        current_time += dt;
    }

    // --- STEP 3: TrapJ 복귀 ---
    (void)traj_gen.trapj(q_home);
    while (!traj_gen.goal_reached(0.05, std::nullopt, std::nullopt, 0.1)) {
        traj_gen.update(dt);
        csv << current_time << ",3"; // Mode 3: TrapJ
        for(int j=0; j<6; ++j) csv << "," << traj_gen.angles()(j) << "," << traj_gen.angvels()(j) << "," << traj_gen.angaccs()(j);
        csv << "," << traj_gen.tmat().translation().transpose().format(Eigen::IOFormat(5, 0, ",", ",", "", "", "", "")) << "\n";
        current_time += dt;
    }

    csv.close();
    return 0;
}