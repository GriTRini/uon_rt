#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <cmath>

#include "../rt_control/core/core.hpp"
#include "../rt_control/model/model.hpp"
#include "../rt_control/trajectory/trajectory_trapj.hpp"

using namespace rt_control;
using namespace rt_control::trajectory;
using namespace rt_control::model;

int main() {
    // 1. 로봇 모델 로드
    RobotModel my_robot("m1013");

    // 2. 시작 각도 및 목표 각도 설정 (단위: 라디안)
    TrajTrapJ::angles_t start_q = TrajTrapJ::angles_t::Zero();
    TrajTrapJ::angles_t goal_q = TrajTrapJ::angles_t::Zero();
    
    // 역동적인 확인을 위해 1, 2, 3번 축을 크게 움직여봅니다.
    goal_q(0) =  M_PI / 2;  // J1: 90도 회전
    goal_q(1) = -M_PI / 4;  // J2: -45도 꺾기
    goal_q(2) =  M_PI / 2;  // J3: 90도 꺾기

    // 🌟 3. 사용자 정의 한계값 (테스트용) 🌟
    // 고의로 로봇 스펙(약 2.09 rad/s)을 한참 초과하는 100.0 rad/s 를 넣어 경고 메시지를 유발합니다!
    TrajTrapJ::angles_t user_vel = TrajTrapJ::angles_t::Constant(100.0); 
    
    // 가속도는 5.0 rad/s^2 로 정상 범위 내의 값을 줘봅니다.
    TrajTrapJ::angles_t user_acc = TrajTrapJ::angles_t::Constant(5.0);   

    std::cout << "--- 새롭게 설계된 TrapJ 디버깅 시작 ---" << std::endl;
    
    // 4. TrapJ 생성 (새로운 생성자 시그니처 적용)
    TrajTrapJ trapj(
        &my_robot, 
        start_q, TrajTrapJ::angles_t::Zero(), 
        goal_q, TrajTrapJ::angles_t::Zero(),
        user_vel, user_acc
    );

    // 5. CSV 파일 헤더 작성 (plot_debug.py와 plot_robot_anim.py 모두 호환되도록 구성)
    std::ofstream csv_file("fk_output.csv");
    csv_file << "time";
    for(int i=1; i<=6; ++i) csv_file << ",q" << i;
    for(int i=0; i<=6; ++i) csv_file << ",x" << i << ",y" << i << ",z" << i;
    csv_file << "\n";

    double dt = 0.001; // 1ms
    double current_time = 0.0;

    // 6. 실시간 제어 루프
    while (!trapj.goal_reached()) {
        auto q = trapj.angles();
        std::vector<Eigen::Isometry3d> poses = my_robot.forward_kinematics_all(q);

        csv_file << std::fixed << std::setprecision(4) << current_time;
        for(int i=0; i<6; ++i) csv_file << "," << std::setprecision(6) << q(i);
        for(int i=0; i<=6; ++i) {
            csv_file << "," << poses[i].translation().x()
                     << "," << poses[i].translation().y()
                     << "," << poses[i].translation().z();
        }
        csv_file << "\n";

        trapj.update(dt);
        current_time += dt;
    }

    csv_file.close();
    std::cout << "완료! fk_output.csv 파일이 저장되었습니다." << std::endl;

    return 0;
}