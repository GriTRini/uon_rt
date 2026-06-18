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
void log_data(std::ofstream& csv, double time, int mode, const TrajGenerator& traj_gen) {
    auto tcp_pos = traj_gen.tmat().translation();
    auto flange_pos = traj_gen.flange_tmat().translation();

    csv << time << "," << mode << ","
        << tcp_pos.x() << "," << tcp_pos.y() << "," << tcp_pos.z() << ","
        << flange_pos.x() << "," << flange_pos.y() << "," << flange_pos.z();
    
    for (int j = 0; j < 6; ++j) {
        csv << "," << traj_gen.angles()(j);
    }
    csv << "\n";
}

int main() {
    rt_control::model::RobotModel model("hcr14");
    TrajGenerator traj_gen;
    
    using tmat_t = Eigen::Isometry3d;
    using angles_t = Eigen::Matrix<double, 6, 1>;

    // ==========================================================
    // [초기 자세 및 TCP 설정]
    // ==========================================================
    angles_t q_home; 
    q_home << 97.64, -101.78, -91.5, -73.42, 88.9, 8.74;
    traj_gen.initialize(model, q_home, angles_t::Zero(), angles_t::Zero());

    // TCP 오프셋 설정 (Z축 +0.25m)
    traj_gen.set_tcp(0.0, 0.0, 0.25, 0.0, 0.0, 0.0);
    std::cout << "✅ 로봇 초기화 및 TCP 설정 완료 (Z축 +0.25m)" << std::endl;

    // ==========================================================
    // [웨이포인트 정의] - CSV 기록을 위해 먼저 정의
    // ==========================================================
    std::vector<WaypointJ> waypoints;
    angles_t q_wp1, q_wp2, q_wp3;
    q_wp1 << 97.64, -90.0, -80.0, -73.42, 88.9, 8.74;
    q_wp2 << 45.0, -95.0, -85.0, -60.0, 90.0, 0.0;
    q_wp3 << 0.0, -100.0, -90.0, -50.0, 90.0, 0.0;

    waypoints.push_back({q_wp1, 0.15});
    waypoints.push_back({q_wp2, 0.20});
    waypoints.push_back({q_wp3, 0.005});

    // ==========================================================
    // [CSV 로깅 파일 오픈 및 헤더 기록]
    // ==========================================================
    std::ofstream csv("playj_test_log.csv");
    csv << std::fixed << std::setprecision(6);
    
    // 🌟 웨이포인트를 CSV 파일 상단에 메타데이터로 저장
    csv << "# [WAYPOINTS]\n";
    csv << "# WP_Index,J1,J2,J3,J4,J5,J6,AttrL\n";
    csv << "# 1," << q_wp1(0) << "," << q_wp1(1) << "," << q_wp1(2) << "," << q_wp1(3) << "," << q_wp1(4) << "," << q_wp1(5) << ",0.15\n";
    csv << "# 2," << q_wp2(0) << "," << q_wp2(1) << "," << q_wp2(2) << "," << q_wp2(3) << "," << q_wp2(4) << "," << q_wp2(5) << ",0.20\n";
    csv << "# 3," << q_wp3(0) << "," << q_wp3(1) << "," << q_wp3(2) << "," << q_wp3(3) << "," << q_wp3(4) << "," << q_wp3(5) << ",0.005\n";
    
    // 🌟 실제 데이터 시작 지점 마커
    csv << "# [LOG_DATA]\n";
    csv << "Time,Mode,TCP_X,TCP_Y,TCP_Z,Flange_X,Flange_Y,Flange_Z,J1,J2,J3,J4,J5,J6\n";

    double dt = 0.001;
    double current_time = 0.0;
    double test_duration;
    double start_time;

    // ==========================================================
    // [Step 1] 확실한 시작을 위해 Home 위치로 TRAPJ 이동
    // ==========================================================
    std::cout << "\nStep 1: 초기 Home 위치로 이동 (TRAPJ)" << std::endl;
    traj_gen.trapj(q_home);
    while (!traj_gen.goal_reached(0.1, 0.002)) { 
        traj_gen.update(dt); 
        log_data(csv, current_time, 0, traj_gen);
        current_time += dt; 
        if(current_time > 5.0) break; 
    }
    std::cout << " -> Home 도달 완료!" << std::endl;

    // ==========================================================
    // [Step 2] 3개의 웨이포인트를 연속으로 부드럽게 지나는 PLAYJ 실행
    // ==========================================================
    std::cout << "\nStep 2: 다중 웨이포인트 연속 이동 (PLAYJ)" << std::endl;

    angles_t peak_vels = model.get_max_angvels() * 0.5;
    angles_t peak_accs = model.get_max_angaccs() * 0.5;

    traj_gen.playj(waypoints, peak_vels, peak_accs, 5.0);

    test_duration = 15.0; 
    start_time = current_time;

    while (current_time - start_time < test_duration) {
        traj_gen.update(dt);
        log_data(csv, current_time, 2, traj_gen); 
        current_time += dt;
        
        if (traj_gen.goal_reached()) {
            std::cout << " -> 최종 목적지 도달 및 정지 완료!" << std::endl;
            break;
        }
    }

    csv.close();
    std::cout << "\n✅ 테스트 시나리오 완료! 결과가 'playj_test_log.csv'에 저장되었습니다." << std::endl;
    return 0;
}