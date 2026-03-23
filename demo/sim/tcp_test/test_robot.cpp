#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>
#include <cmath>
#include "../../../rt_control/model/model.hpp"
#include "../../../rt_control/trajectory/trajectory_generator.hpp"

using namespace rt_control;
using namespace rt_control::trajectory;

int main() {
    // 1. 모델 및 제너레이터 초기화
    model::RobotModel model("m1013");
    TrajGenerator traj_gen;

    // CSV 파일 준비
    std::ofstream csv("tcp_multi_test_data.csv");
    if (!csv.is_open()) {
        std::cerr << "파일을 생성할 수 없습니다!" << std::endl;
        return -1;
    }
    csv << "Scenario,Time,J1,J2,J3,J4,J5,J6,Target_X,Target_Y,Target_Z,TCP_X,TCP_Y,TCP_Z,TCP_R,TCP_P,TCP_Yw\n";

    // 공통 목표 포즈 (Z축이 바닥을 향함)
    Eigen::Isometry3d goal = Eigen::Isometry3d::Identity();
    goal.translation() << 0.5, 0.1, 0.4;
    Eigen::Matrix3d R_goal;
    R_goal << 1, 0, 0, 0, -1, 0, 0, 0, -1; // Z축 [0,0,-1]
    goal.linear() = R_goal;

    // 테스트할 5가지 TCP 설정 (X, Y, Z, R, P, Yw)
    struct TCPConfig { std::string name; double x; double y; double z; double r; double p; double yw; };
    std::vector<TCPConfig> configs = {
        {"no_tcp",      0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {"current_tcp", 0.0, 0.0, 0.1, 0.0, 90.0, 0.0}
    };

    double dt = 0.001;

    for (const auto& conf : configs) {
        std::cout << "\n>>> [Scenario] " << conf.name << " 실행 중..." << std::endl;
        
        // TCP 설정 및 초기화
        traj_gen.set_tcp(conf.x, conf.y, conf.z, conf.r, conf.p, conf.yw);
        traj_gen.initialize(model, angles_t::Zero(), angles_t::Zero(), angles_t::Zero());
        
        double current_time = 0.0;
        int step = 0;

        // 목표 지점으로 이동
        if (traj_gen.attrl(goal, 40.0)) {
            while (!traj_gen.goal_reached(0.1, 0.002, 1.0)) {
                traj_gen.update(dt);
                
                // 데이터 기록
                csv << conf.name << "," << std::fixed << std::setprecision(5) << current_time << ",";
                for(int j=0; j<6; ++j) csv << traj_gen.angles()(j) << ",";
                csv << goal.translation().x() << "," << goal.translation().y() << "," << goal.translation().z() << ",";
                csv << conf.x << "," << conf.y << "," << conf.z << "," << conf.r << "," << conf.p << "," << conf.yw << "\n";
                
                current_time += dt;
                step++;
                if (step > 15000) break; // 최대 15초 타임아웃
            }
        }
        std::cout << "    ✅ 도달 완료 (Time: " << current_time << "s)" << std::endl;
        
        // 시나리오 간 정지 구간
        for(int j=0; j<200; ++j) { traj_gen.update(dt); }
    }

    csv.close();
    std::cout << "\n🏁 모든 시뮬레이션 완료. 'tcp_multi_test_data.csv'가 생성되었습니다." << std::endl;
    return 0;
}