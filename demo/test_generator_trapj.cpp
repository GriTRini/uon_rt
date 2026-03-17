#include <iostream>
#include <fstream>
#include <iomanip>
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
    // 1. 모델 및 시작 상태 설정 (m1013)
    RobotModel model("m1013");
    angles_t q_start = angles_t::Zero();
    angles_t dq_start = angles_t::Zero();
    angles_t ddq_start = angles_t::Zero();

    // 2. 궤적 생성기 초기화
    TrajGenerator traj_gen(model, q_start, dq_start, ddq_start);

    // 3. 데이터 기록용 설정
    std::ofstream csv("/home/uon/uon_rt/demo/gen_only_trapj.csv");
    csv << "Time,Desired_Q,Desired_DQ,Desired_DDQ\n";

    double current_time = 0.0;
    double dt = 0.001; // 1ms

    // 4. 타이머 기반 루프 정의 (실제 Robot 클래스의 update와 동일한 로직)
    uon::timer::Timer<int64_t, std::milli> control_timer(
        std::chrono::milliseconds(1),
        [&]() {
            traj_gen.update(dt);
            
            // 데이터 기록 (1번 조인트 기준)
            csv << current_time << ","
                << traj_gen.angles()(0) << ","
                << traj_gen.angvels()(0) << ","
                << traj_gen.angaccs()(0) << "\n";
            
            current_time += dt;
        }
    );

    // 5. 시나리오 실행: 10도 이동 명령
    std::cout << "시뮬레이션 시작: 0도 -> 10도 이동 (TRAPJ)" << std::endl;
    
    // TRAPJ 명령
    angles_t q_goal = angles_t::Zero();
    q_goal(0) = 10.0;

    (void)traj_gen.trapj(q_goal);

    // 타이머 가동
    control_timer.start();

    // 2초간 데이터 수집 (충분히 도달할 시간)
    std::this_thread::sleep_for(std::chrono::seconds(2));

    control_timer.stop();
    csv.close();

    std::cout << "시뮬레이션 완료: gen_only_debug.csv 확인" << std::endl;

    return 0;
}