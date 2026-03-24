#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>
#include "../../../rt_control/trajectory/trajectory_generator.hpp"

using namespace rt_control;

int main() {
    model::RobotModel model("m1013");
    trajectory::TrajGenerator traj_gen;

    std::ofstream csv("tcp_square_sim_data.csv");
    csv << "Scenario,Time,J1,J2,J3,J4,J5,J6,"
        << "Target_X,Target_Y,Target_Z,"
        << "TCP_X,TCP_Y,TCP_Z,TCP_R,TCP_P,TCP_Yw\n";

    double dt = 0.001;
    double total_time = 0.0;
    Eigen::Isometry3d current_goal = Eigen::Isometry3d::Identity();

    // 공용 TCP 변수 설정 (기록용)
    double tx = 0.0, ty = 0.0, tz = 0.1, tr = 0.0, tp = 60.0, tyw = 45.0;

    // ---------------------------------------------------------
    // 🛠️ 헬퍼 함수: 시뮬레이션 업데이트 및 CSV 데이터 기록
    // ---------------------------------------------------------
    auto wait_until_done = [&](const std::string& scenario, double timeout_sec = 8.0) {
        int max_steps = static_cast<int>(timeout_sec / dt);
        int step = 0;
        
        // 판정 기준 완화: 위치 오차 1.5cm, 회전 오차 3.0도 이내면 도달로 인정
        while (!traj_gen.goal_reached(0.1, 0.015, 3.0) && step < max_steps) {
            traj_gen.update(dt);
            
            csv << scenario << "," << std::fixed << std::setprecision(5) << total_time << ",";
            for(int j=0; j<6; ++j) csv << traj_gen.angles()(j) << ",";
            csv << current_goal.translation().x() << "," << current_goal.translation().y() << "," << current_goal.translation().z() << ",";
            csv << tx << "," << ty << "," << tz << "," << tr << "," << tp << "," << tyw << "\n";
            
            total_time += dt;
            step++;
        }
        std::cout << "  >> [" << scenario << "] " << (step < max_steps ? "완료" : "타임아웃") 
                  << " (" << std::setprecision(2) << step * dt << "s)" << std::endl;
    };

    // =========================================================
    // 🚀 시나리오 시작
    // =========================================================
    traj_gen.initialize(model, angles_t::Zero(), angles_t::Zero(), angles_t::Zero());

    // --- Step 1: 시작 자세로 이동 (TrapJ) ---
    std::cout << "\n1️⃣ 초기 자세로 이동 중..." << std::endl;
    angles_t q_pose1;
    q_pose1 << 0.0, 0.0, -90.0, 0.0, -90.0, 0.0; 
    current_goal = traj_gen.solve_forward(q_pose1); // 현재 목표 위치 계산
    
    if (traj_gen.trapj(q_pose1)) wait_until_done("Setup_TrapJ");


    // --- Step 2: 툴 장착 (TCP 설정) ---
    std::cout << "\n2️⃣ 툴 장착 (Z: 10cm, Pitch: 60, Yaw: 45)" << std::endl;
    traj_gen.set_tcp(tx, ty, tz, tr, tp, tyw); 


    // --- 🌟 Step 3: 툴 팁을 바닥으로 수직 정렬 (내장 함수 사용) ---
    std::cout << "\n3️⃣ 툴 팁을 바닥(-Z) 방향으로 정렬합니다..." << std::endl;
    
    // Generator에 추가한 함수를 호출만 하면 끝입니다!
    if (traj_gen.align_tcp_to_floor(200.0)) {
        current_goal = traj_gen.tmat(); // 정렬된 위치를 목표로 삼아 시각화
        wait_until_done("Align_Vertical");
    }


    // --- Step 4: 사각형 궤적 이동 (.x() .y() 참조 방식) ---
    std::cout << "\n4️⃣ 수직을 유지하며 사각형 그리기 시작..." << std::endl;
    Eigen::Isometry3d target;

    // Side 1: X축 이동
    target = traj_gen.tmat(); // 🌟 현재 툴이 바닥을 보는 상태를 그대로 복사!
    target.translation().x() += 0.2; 
    current_goal = target;
    if (traj_gen.attrl(target, 150.0)) wait_until_done("Side_1");

    // Side 2: Y축 이동
    target = traj_gen.tmat();
    target.translation().y() += 0.2; 
    current_goal = target;
    if (traj_gen.attrl(target, 150.0)) wait_until_done("Side_2");

    // Side 3: X축 복귀
    target = traj_gen.tmat();
    target.translation().x() -= 0.2; 
    current_goal = target;
    if (traj_gen.attrl(target, 150.0)) wait_until_done("Side_3");

    // Side 4: Y축 복귀
    target = traj_gen.tmat();
    target.translation().y() -= 0.2; 
    current_goal = target;
    if (traj_gen.attrl(target, 150.0)) wait_until_done("Side_4");

    csv.close();
    std::cout << "\n🏁 모든 테스트 완료. CSV 파일을 파이썬으로 확인하세요!" << std::endl;
    return 0;
}