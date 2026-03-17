#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <Eigen/Dense>

#include "../rt_control/model/model.hpp"
#include "../rt_control/trajectory/trajectory_generator.hpp"
#include "../rt_control/timer.hpp"

using namespace rt_control;
using namespace rt_control::trajectory;
using namespace rt_control::model;

int main() {
    // 1. 모델 설정
    RobotModel model("m1013");
    angles_t q_start = angles_t::Zero();
    angles_t dq_start = angles_t::Zero();
    angles_t ddq_start = angles_t::Zero();

    // 2. 생성기 초기화
    TrajGenerator traj_gen(model, q_start, dq_start, ddq_start);

    // 3. 기록용 파일
    std::ofstream csv("/home/uon/uon_rt/demo/gen_only_debug.csv");
    csv << "Time,Desired_Q,Desired_DQ,Desired_DDQ\n";

    double current_time = 0.0;
    const double dt = 0.001; // 1ms 제어 주기

    // 4. 컨트롤 루프 (Timer)
    uon::timer::Timer<int64_t, std::milli> control_timer(
        std::chrono::milliseconds(1),
        [&]() {
            traj_gen.update(dt); // 🌟 여기서 내부적으로 m_gen_attrj.update(dt)가 호출되어야 함
            
            csv << current_time << ","
                << traj_gen.angles()(0) << ","
                << traj_gen.angvels()(0) << ","
                << traj_gen.angaccs()(0) << "\n";
            
            current_time += dt;
        }
    );

    // 5. 시나리오 실행
    std::cout << "--- AttrJ Simulation Start ---" << std::endl;
    
    angles_t q_goal = angles_t::Zero();
    q_goal(0) = 10.0; // 1번 조인트 10도 이동

    // 🌟 AttrJ 호출 시 Kp 값을 5.0 정도로 설정
    // 만약 TrajGenerator::attrj 인터페이스가 각도와 Kp만 받는다면 아래와 같이 호출
    if (!traj_gen.attrj(q_goal, 5.0)) {
        std::cerr << "Failed to initialize AttrJ" << std::endl;
        return -1;
    }

    control_timer.start();

    // 3초간 데이터 수집 (AttrJ는 수렴에 시간이 더 걸리므로 넉넉히 잡음)
    std::this_thread::sleep_for(std::chrono::seconds(3));

    control_timer.stop();
    csv.close();

    std::cout << "Simulation Finished. Check gen_only_debug.csv" << std::endl;

    return 0;
}