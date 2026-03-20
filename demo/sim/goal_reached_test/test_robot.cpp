#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>
#include <chrono>
#include <numeric>
#include <algorithm>
#include "../../../rt_control/model/model.hpp"
#include "../../../rt_control/trajectory/trajectory_generator.hpp"

using namespace rt_control;
using namespace rt_control::trajectory;

int main() {
    // 1. 모델 및 제너레이터 초기화
    rt_control::model::RobotModel model("m1013");
    TrajGenerator traj_gen;

    angles_t q_start = angles_t::Zero();
    angles_t q_home; 
    q_home << 0, -10, -100, 0, -70, 0;

    traj_gen.initialize(model, q_start, angles_t::Zero(), angles_t::Zero());

    // 2. CSV 파일 오픈
    std::ofstream csv("goal_reached_fail_test.csv");
    csv << "Time,Step,J1,J2,J3,J4,J5,J6,Q_Err,Pos_Err,Rot_Err,dQ_Err,GoalReached\n";

    double dt = 0.001; 
    double current_time = 0.0;
    
    // 3. 시뮬레이션 루프 람다 함수
    auto run_sim = [&](int step_id, const std::string& label) {
        std::cout << ">>> [Step " << step_id << ": " << label << "] 구동 시작..." << std::endl;
        int loop_count = 0;
        bool forced_break = false;

        // 정밀한 goal_reached 판정 루프
        while (!traj_gen.goal_reached(0.1, 0.002, 1.0, 0.5, 0.01, 0.02)) {
            traj_gen.update(dt); 

            // 변수들을 먼저 선언하여 스코프 문제 해결
            angles_t cur_q = traj_gen.angles();
            double q_err = traj_gen.angles_enorm().value_or(0.0);    // 관절 오차 (deg)
            double p_err = traj_gen.pos_enorm().value_or(0.0);       // 위치 오차 (m)
            double r_err = traj_gen.rot_enorm().value_or(0.0);       // 회전 오차 (deg)
            double dq_err = traj_gen.angvels_enorm().value_or(0.0);  // 속도 오차 (deg/s)

            // 데이터 기록
            csv << std::fixed << std::setprecision(5) << current_time << "," << step_id << ",";
            for(int j=0; j<6; ++j) csv << cur_q(j) << ",";
            csv << q_err << "," << p_err << "," << r_err << "," << dq_err << ",0\n";

            current_time += dt;
            loop_count++;

            // 1초(1000루프)마다 모니터링 출력
            if (loop_count % 1000 == 0) {
                std::cout << "    [T: " << current_time << "s] Pos_Err: " << p_err << " m (Target: 0.002)" << std::endl;
            }

            // 안전 타임아웃 (30초)
            if (loop_count > 30000) { 
                std::cout << "    ⚠️ [경고] 도달 실패! 타임아웃 발생 (강제 종료)" << std::endl;
                forced_break = true;
                break; 
            }
        }

        if (!forced_break) {
            std::cout << "    ✅ 도달 성공!" << std::endl;
            csv << current_time << "," << step_id << ",0,0,0,0,0,0,0,0,0,0,1\n";
        }

        // 안정화를 위한 짧은 휴지기
        for(int j=0; j<100; ++j) { traj_gen.update(dt); current_time += dt; }
    };

    // -------------------------------------------------------------------------
    // 4. 시나리오 실행
    // -------------------------------------------------------------------------

    // [Step 1] 정상 홈포인트 이동
    (void)traj_gen.trapj(q_home); 
    run_sim(1, "Move_to_Home");

    // [Step 2] 도달 불가 테스트: 지하 5미터로 하강 명령
    std::cout << "\n🔥 [도달 불가 테스트 시작]" << std::endl;
    Eigen::Isometry3d T_home = traj_gen.tmat();
    Eigen::Isometry3d T_impossible = T_home;
    T_impossible.translation().z() -= 5.0; // 로봇 작업 반경을 완전히 벗어남

    (void)traj_gen.attrl(T_impossible, 40.0); 
    run_sim(2, "Impossible_Z_Down"); 

    // [Step 3] 다시 원래 위치로 복귀
    std::cout << "\n🔄 [복귀 테스트]" << std::endl;
    (void)traj_gen.attrl(T_home, 40.0); 
    run_sim(3, "Return_to_Home");

    csv.close();
    std::cout << "\n🏁 테스트 완료. CSV 데이터를 분석하세요." << std::endl;

    return 0;
}