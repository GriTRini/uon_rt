#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <Eigen/Dense>

#include "../../../rt_control/model/model.hpp"
#include "../../../rt_control/trajectory/trajectory_generator.hpp"

using namespace rt_control;
using namespace rt_control::trajectory;

int main() {
    // 1. 모델 로드 및 초기화
    model::RobotModel model("m1013");
    TrajGenerator traj_gen;

    // 초기 자세 설정
    angles_t q_init;
    q_init << 0.0, 0.0, 90.0, 0.0, 90.0, 0.0; 
    traj_gen.initialize(model, q_init, angles_t::Zero(), angles_t::Zero());
    traj_gen.set_tcp(0.0, 0.0, 0.1, 0.0, 0.0, 0.0);

    // 2. CSV 파일 오픈
    std::ofstream csv("robot_joint_data.csv");
    csv << "Time,X,Y,Z,J1,J2,J3,J4,J5,J6\n";

    // 3. 대각선 목표 설정
    Eigen::Isometry3d p1 = Eigen::Isometry3d::Identity();
    p1.translation() << 0.6, 0.4, 0.7;
    p1.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();

    Eigen::Isometry3d p2 = Eigen::Isometry3d::Identity();
    p2.translation() << -0.3, -0.4, 0.2;
    p2.linear() = p1.linear();

    std::vector<Eigen::Isometry3d> targets = {p1, p2};
    double dt = 0.001;
    double current_time = 0.0;

    std::cout << "[C++] 데이터 생성 시작..." << std::endl;

    for (const auto& target : targets) {
        // 🌟 [[nodiscard]] 경고 해결: 반환값을 체크합니다.
        if (!traj_gen.attrl(target, 120.0)) {
            std::cerr << "Error: attrl mode 시작 실패!" << std::endl;
            return -1;
        }

        int step = 0;
        // goal_reached 인자는 현재 헤더 정의에 맞게 조정 (위치 2mm, 회전 1도 등)
        while (!traj_gen.goal_reached(0.1, 0.002, 1.0, 0.5)) {
            traj_gen.update(dt);
            
            const auto& q = traj_gen.angles();
            const auto& t = traj_gen.tmat().translation();

            csv << std::fixed << std::setprecision(6) << current_time << ","
                << t.x() << "," << t.y() << "," << t.z() << ","
                << q(0) << "," << q(1) << "," << q(2) << "," << q(3) << "," << q(4) << "," << q(5) << "\n";

            current_time += dt;
            step++;
        }
    }

    csv.close();
    std::cout << "[C++] 저장 완료: robot_joint_data.csv" << std::endl;
    return 0;
}