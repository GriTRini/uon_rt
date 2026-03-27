#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "../../../../rt_control/model/model.hpp"
#include "../../../../rt_control/trajectory/trajectory_generator.hpp"

using namespace rt_control;

void print_pose(const std::string& label, const Eigen::Isometry3d& pose) {
    auto t = pose.translation();
    std::cout << "[" << label << "] X: " << t.x() << ", Y: " << t.y() << ", Z: " << t.z() << std::endl;
}

int main() {
    model::RobotModel model("m1013");
    trajectory::TrajGenerator traj_gen;
    traj_gen.initialize(model, angles_t::Zero(), angles_t::Zero(), angles_t::Zero());

    // --- TEST 1: All Zero ---
    angles_t q_zero = angles_t::Zero();
    std::cout << "1. Testing All Zero Pose..." << std::endl;
    print_pose("Flange", traj_gen.solve_forward(q_zero));

    // --- TEST 2: Joint 3 at 90 deg ---
    angles_t q_j3_90 = angles_t::Zero();
    q_j3_90(2) = 90.0; // Index 2 is Joint 3
    std::cout << "\n2. Testing Joint 3 at 90 deg..." << std::endl;
    print_pose("Flange", traj_gen.solve_forward(q_j3_90));

    // --- TEST 3: User TCP Applied ---
    const double tx = -0.121, ty = -0.121, tz = 0.266;
    const double tr = 60.0, tp = 37.76, tyw = -135.0;
    traj_gen.set_tcp(tx, ty, tz, tr, tp, tyw);
    
    std::cout << "\n3. Testing All Zero with TCP..." << std::endl;
    print_pose("TCP", traj_gen.solve_forward(q_zero));

    return 0;
}