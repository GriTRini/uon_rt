#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <Eigen/Dense>

// 경로에 맞게 헤더 파일 포함
#include "../rt_control/model/model.hpp"
#include "../rt_control/trajectory/trajectory_generator.hpp"

using namespace rt_control;
using namespace rt_control::trajectory;
using namespace rt_control::model;

int main() {
    // 1. 모델 및 TrajGenerator 초기화
    RobotModel model("m1013");
    
    angles_t q_start = angles_t::Zero();
    angles_t start_dq = angles_t::Zero();
    angles_t start_ddq = angles_t::Zero();

    TrajGenerator traj_gen(model, q_start, start_dq, start_ddq);

    // 🌟 변수 선언 위치 확인 (람다 함수보다 앞에 있어야 합니다)
    double t = 0.0;
    double dt = 0.001; // 1ms 제어 주기

    // 2. CSV 파일 열기
    std::string csv_path = "/home/uon/uon_rt/demo/trajectory_data.csv";
    std::ofstream out(csv_path);
    if (!out.is_open()) {
        std::cerr << "CSV 파일을 열 수 없습니다: " << csv_path << std::endl;
        return -1;
    }
    
    // CSV 헤더 작성 (X, Y, Z 포함)
    out << "Time,Mode,Q1,Q2,Q3,Q4,Q5,Q6,X,Y,Z\n";

    // 🌟 [&] 캡처를 사용하여 외부의 t, out 등을 람다 내부에서 쓸 수 있게 합니다.
    auto log_data = [&](const std::string& mode) {
        angles_t q = traj_gen.angles();
        Eigen::Vector3d pos = traj_gen.tmat().translation();
        
        out << t << "," << mode << ","
            << q(0) << "," << q(1) << "," << q(2) << ","
            << q(3) << "," << q(4) << "," << q(5) << ","
            << pos.x() << "," << pos.y() << "," << pos.z() << "\n";
    };

    // -----------------------------------------------------------
    // [1] TRAPJ 테스트
    // -----------------------------------------------------------
    std::cout << "[1] TRAPJ 시작 (목표: 90도)" << std::endl;
    angles_t q1; q1 << 90, 0, 0, 0, 0, 0;
    (void)traj_gen.trapj(q1);

    // 상태가 STOP이 될 때까지 루프
    while (traj_gen.traj_state() != TrajState::STOP && t < 10.0) {
        traj_gen.update(dt);
        log_data("TRAPJ");
        t += dt;
    }

    // -----------------------------------------------------------
    // [2] ATTRJ 테스트
    // -----------------------------------------------------------
    std::cout << "[2] ATTRJ 시작 (목표: 0도)" << std::endl;
    angles_t q2 = angles_t::Zero();
    (void)traj_gen.attrj(q2, 10.0); // Kp = 10.0

    double start_attrj = t;
    // 점근적 수렴이므로 3초간 강제 구동하거나 오차 체크
    while (t < start_attrj + 3.0) {
        traj_gen.update(dt);
        log_data("ATTRJ");
        t += dt;
        if ((traj_gen.angles() - q2).norm() < 0.1) break;
    }

    // -----------------------------------------------------------
    // [3] ATTRL 테스트
    // -----------------------------------------------------------
    std::cout << "[3] ATTRL 시작" << std::endl;
    Eigen::Isometry3d target_tmat = traj_gen.tmat();
    target_tmat.translation().z() -= 0.1; // 10cm 아래로
    
    (void)traj_gen.attrl(target_tmat, 15.0);

    double start_attrl = t;
    while (t < start_attrl + 3.0) {
        traj_gen.update(dt);
        log_data("ATTRL");
        t += dt;
    }

    out.close();
    std::cout << "모든 테스트 완료! 도달 시간: " << t << "s" << std::endl;

    return 0;
}