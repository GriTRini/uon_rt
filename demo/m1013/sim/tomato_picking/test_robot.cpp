#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include "../../../../rt_control/model/model.hpp"
#include "../../../../rt_control/trajectory/trajectory_generator.hpp"

using namespace rt_control;
using namespace rt_control::trajectory;

// --- 공통 로깅 함수 ---
// 현재 시간, 모드(0:TrapJ, 1:Linear, 2:Wait), 포즈(X,Y,Z), 관절 각도(J1~J6)를 CSV에 기록
void log_data(std::ofstream& csv, double time, int mode, const TrajGenerator& traj_gen) {
    csv << time << "," << mode << ","
        << traj_gen.tmat().translation().x() << ","
        << traj_gen.tmat().translation().y() << ","
        << traj_gen.tmat().translation().z();
    
    for (int j = 0; j < 6; ++j) {
        csv << "," << traj_gen.angles()(j);
    }
    csv << "\n";
}

// --- 직선 보간 유틸리티 ---
void move_linear_interp(TrajGenerator& traj_gen, const Eigen::Isometry3d& target_pose, 
                        double duration, double dt, std::ofstream& csv, double& current_time) {
    Eigen::Isometry3d start_pose = traj_gen.tmat();
    int total_steps = static_cast<int>(duration / dt);

    for (int i = 0; i <= total_steps; ++i) {
        double s = static_cast<double>(i) / total_steps;
        double s_smooth = s * s * (3.0 - 2.0 * s); 

        Eigen::Isometry3d interp_pose = start_pose;
        interp_pose.translation() = start_pose.translation() * (1.0 - s_smooth) + target_pose.translation() * s_smooth;
        
        Eigen::Quaterniond q_start(start_pose.linear());
        Eigen::Quaterniond q_end(target_pose.linear());
        interp_pose.linear() = q_start.slerp(s_smooth, q_end).toRotationMatrix();

        traj_gen.attrl(interp_pose, 500.0);
        traj_gen.update(dt);

        // 로깅 (Mode 1: Linear Interpolation)
        log_data(csv, current_time, 1, traj_gen);
        current_time += dt;
    }
}

int main() {
    rt_control::model::RobotModel model("m1013");
    TrajGenerator traj_gen;
    
    using tmat_t = Eigen::Isometry3d;
    using angles_t = Eigen::Matrix<double, 6, 1>;

    // 초기 자세 설정
    angles_t q_home; q_home << -86.96, -31.27, -59.55, -0.18, -89.7, 0.0;
    traj_gen.initialize(model, q_home, angles_t::Zero(), angles_t::Zero());

    // CSV 정밀도 설정
    std::ofstream csv("apple_pick_place_test.csv");
    csv << std::fixed << std::setprecision(6);
    csv << "Time,Mode,X,Y,Z,J1,J2,J3,J4,J5,J6\n";

    double dt = 0.001;
    double current_time = 0.0;

    // 타겟 및 포인트 설정
    tmat_t best_target_mat = traj_gen.tmat();
    best_target_mat.translation() << 0.1, 0.6, 0.05;

    angles_t q_zig_1; q_zig_1 << -259.35, -26.67, -67.6, 4.74, -85.36, 0.0;
    angles_t q_zig_90deg; q_zig_90deg << -246.74, -21.1, -139.97, 21.15, 74.06, -90.0;

    // --- 시나리오 시작 ---

    // [Step 1 ~ 5.1] 기존 코드 유지
    std::cout << "Step 1: Home" << std::endl;
    traj_gen.trapj(q_home);
    while (!traj_gen.goal_reached(0.1, 0.002)) { 
        traj_gen.update(dt); 
        log_data(csv, current_time, 0, traj_gen);
        current_time += dt; 
    }

    std::cout << "Step 2: Approach" << std::endl;
    tmat_t offset_mat = tmat_t::Identity();
    offset_mat.translation() << 0, 0, -0.05;
    move_linear_interp(traj_gen, best_target_mat * offset_mat, 0.8, dt, csv, current_time);

    std::cout << "Step 2.1: Down" << std::endl;
    move_linear_interp(traj_gen, best_target_mat, 0.5, dt, csv, current_time);

    std::cout << "Step 3: Lift" << std::endl;
    traj_gen.trapj(q_home);
    while (!traj_gen.goal_reached(0.1, 0.002)) { 
        traj_gen.update(dt); log_data(csv, current_time, 0, traj_gen); current_time += dt; 
    }

    std::cout << "Step 4: Move to Zig" << std::endl;
    traj_gen.trapj(q_zig_1);
    while (!traj_gen.goal_reached(0.1, 0.002)) { 
        traj_gen.update(dt); log_data(csv, current_time, 0, traj_gen); current_time += dt; 
    }

    std::cout << "Step 4.1: Zig Down" << std::endl;
    tmat_t zig_down = traj_gen.tmat();
    zig_down.translation().z() -= 0.07;
    move_linear_interp(traj_gen, zig_down, 0.5, dt, csv, current_time);

    std::cout << "Step 4.2: Wait" << std::endl;
    for(int i=0; i<300; ++i) { 
        traj_gen.update(dt); log_data(csv, current_time, 2, traj_gen); current_time += dt; 
    }

    std::cout << "Step 5: Lift Up" << std::endl;
    tmat_t lift_up = traj_gen.tmat();
    lift_up.translation().z() += 0.1;
    move_linear_interp(traj_gen, lift_up, 0.6, dt, csv, current_time);

    std::cout << "Step 5.11: Retreat Y" << std::endl;
    tmat_t retreat_y = traj_gen.tmat();
    retreat_y.translation().y() += 0.4;
    move_linear_interp(traj_gen, retreat_y, 1.2, dt, csv, current_time);

    std::cout << "Step 5.1: 90deg Approach" << std::endl;
    traj_gen.trapj(q_zig_90deg);
    while (!traj_gen.goal_reached(0.1, 0.002)) { 
        traj_gen.update(dt); log_data(csv, current_time, 0, traj_gen); current_time += dt; 
    }

    // --- 추가된 구간 (Python 5.2 ~ 5.8) ---

    // [Step 5.2] 90도 회전 잡으러 가기 (Y -0.04)
    std::cout << "Step 5.2: 90도 회전 잡으러 가기 (Y -0.04)" << std::endl;
    tmat_t target_tmat = traj_gen.tmat();
    target_tmat.translation().y() -= 0.04;
    traj_gen.attrl(target_tmat, 500.0);
    while (!traj_gen.goal_reached(0.1, 0.002)) {
        traj_gen.update(dt);
        log_data(csv, current_time, 1, traj_gen);
        current_time += dt;
    }

    // [Step 5.3] 90도 회전 후 잡기 (Wait 구간으로 대체)
    std::cout << "Step 5.3: 90도 회전 후 잡기 (그리퍼 동작)" << std::endl;
    for(int i=0; i<500; ++i) { // 그리퍼 작동 시간 대기 (0.5초)
        traj_gen.update(dt);
        log_data(csv, current_time, 2, traj_gen);
        current_time += dt;
    }

    // [Step 5.4] 잡은 후 상승 (Z +0.1)
    std::cout << "Step 5.4: 잡은 후 상승 (Z +0.1)" << std::endl;
    target_tmat = traj_gen.tmat();
    target_tmat.translation().z() += 0.1;
    traj_gen.attrl(target_tmat, 500.0);
    while (!traj_gen.goal_reached(0.1, 0.002)) {
        traj_gen.update(dt);
        log_data(csv, current_time, 1, traj_gen);
        current_time += dt;
    }

    // [Step 5.5] X축 후퇴 및 하강 (X -0.3, Z -0.1)
    std::cout << "Step 5.5: X -0.3, Z -0.1 이동" << std::endl;
    target_tmat = traj_gen.tmat();
    target_tmat.translation().x() -= 0.3;
    target_tmat.translation().z() -= 0.1;
    traj_gen.attrl(target_tmat, 500.0);
    while (!traj_gen.goal_reached(0.1, 0.002)) {
        traj_gen.update(dt);
        log_data(csv, current_time, 1, traj_gen);
        current_time += dt;
    }

    // [Step 5.6] J6 90도 회전
    std::cout << "Step 5.6: J6 90도 추가 회전" << std::endl;
    angles_t target_angles = traj_gen.angles();
    target_angles(5) += 90.0; // degree 기준 (필요시 라디안 변환)
    traj_gen.trapj(target_angles);
    while (!traj_gen.goal_reached(0.1, 0.002)) {
        traj_gen.update(dt);
        log_data(csv, current_time, 0, traj_gen);
        current_time += dt;
    }

    // [Step 5.7] Y축 이동 (Y -0.2)
    std::cout << "Step 5.7: Y -0.2 이동" << std::endl;
    target_tmat = traj_gen.tmat();
    target_tmat.translation().y() -= 0.2;
    traj_gen.attrl(target_tmat, 500.0);
    while (!traj_gen.goal_reached(0.1, 0.002)) {
        traj_gen.update(dt);
        log_data(csv, current_time, 1, traj_gen);
        current_time += dt;
    }

    // [Step 5.8] 최종 상승 (Z +0.4) 및 종료
    std::cout << "Step 5.8: 최종 상승 (Z +0.4)" << std::endl;
    target_tmat = traj_gen.tmat();
    target_tmat.translation().z() += 0.4;
    traj_gen.attrl(target_tmat, 500.0);
    while (!traj_gen.goal_reached(0.1, 0.002)) {
        traj_gen.update(dt);
        log_data(csv, current_time, 1, traj_gen);
        current_time += dt;
    }

    csv.close();
    std::cout << "✅ 모든 시나리오(5.8까지) 완료 및 데이터 저장 성공!" << std::endl;
    return 0;
}