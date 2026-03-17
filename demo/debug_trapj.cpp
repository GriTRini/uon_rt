#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include "../rt_control/model/model.hpp"
#include "../rt_control/trajectory/trajectory_generator.hpp"

using namespace rt_control;
using namespace rt_control::trajectory;

int main() {
    rt_control::model::RobotModel model("m1013");
    angles_t zero_q = angles_t::Zero();
    TrajGenerator traj_gen(model, zero_q, zero_q, zero_q);

    double dt = 0.001;
    angles_t target_q;
    target_q << 45, 0, 0, 0, 0, 0; // 테스트를 위해 1축만 45도 이동

    // --- [Test 1] 현재 설정된 최대 가속도로 이동 ---
    std::cout << "\n[Test 1] 현재 설정값으로 이동 시작..." << std::endl;
    angles_t acc_normal = model.get_max_angaccs();
    std::cout << "사용 가속도(J1): " << acc_normal(0) << " deg/s^2" << std::endl;

    (void)traj_gen.trapj(target_q, std::nullopt, std::nullopt, acc_normal);
    
    double t1 = 0;
    while (!traj_gen.goal_reached() && t1 < 5.0) {
        traj_gen.update(dt);
        t1 += dt;
    }
    std::cout << ">> [Test 1] 완료 시간: " << t1 << "s" << std::endl;

    // 원점 복귀 (로그 생략)
    (void)traj_gen.trapj(zero_q);
    while(!traj_gen.goal_reached()) traj_gen.update(dt);

    // --- [Test 2] 가속도를 절반(0.5x)으로 낮춰서 이동 ---
    std::cout << "\n[Test 2] 가속도 50% 수준으로 이동 시작..." << std::endl;
    angles_t acc_half = acc_normal * 0.5;
    std::cout << "사용 가속도(J1): " << acc_half(0) << " deg/s^2" << std::endl;

    (void)traj_gen.trapj(target_q, std::nullopt, std::nullopt, acc_half);

    double t2 = 0;
    while (!traj_gen.goal_reached() && t2 < 5.0) {
        traj_gen.update(dt);
        t2 += dt;
    }
    std::cout << ">> [Test 2] 완료 시간: " << t2 << "s" << std::endl;

    if (t2 > t1) {
        std::cout << "\n결과: 가속도 조절이 정상 작동합니다. (시간 증가 확인)" << std::endl;
    } else {
        std::cout << "\n결과: 오류! 가속도를 낮췄는데 시간이 같습니다. 내부 로직 확인 필요." << std::endl;
    }

    return 0;
}