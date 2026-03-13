#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <cmath>

#include "../rt_control/core/core.hpp"
#include "../rt_control/model/model.hpp"
#include "../rt_control/trajectory/trajectory_attrj.hpp"

using namespace rt_control;
using namespace rt_control::trajectory;
using namespace rt_control::model;

int main() {
    RobotModel my_robot("m1013");

    TrajAttrJ::angles_t start_q = TrajAttrJ::angles_t::Zero();
    
    // 1차 목표: J1(90도), J2(-45도), J3(90도)
    TrajAttrJ::angles_t goal1_q = TrajAttrJ::angles_t::Zero();
    goal1_q(0) =  M_PI / 2;
    goal1_q(1) = -M_PI / 4;
    goal1_q(2) =  M_PI / 2;

    // 2차 목표: J1(-90도), J2(45도), J3(-90도) - 완전히 반대편!
    TrajAttrJ::angles_t goal2_q = TrajAttrJ::angles_t::Zero();
    goal2_q(0) = -M_PI / 2;
    goal2_q(1) =  M_PI / 4;
    goal2_q(2) = -M_PI / 2;

    // 🌟 가혹 조건: 100 rad/s 속도와 500 rad/s^2 가속도 입력! (경고 메시지 유발) 🌟
    TrajAttrJ::angles_t user_vel = TrajAttrJ::angles_t::Constant(100.0);
    TrajAttrJ::angles_t user_acc = TrajAttrJ::angles_t::Constant(500.0);

    std::cout << "--- 새롭게 설계된 AttrJ 디버깅 시작 ---" << std::endl;

    TrajAttrJ attrj(
        &my_robot, 
        start_q, TrajAttrJ::angles_t::Zero(), TrajAttrJ::angles_t::Zero(),
        user_vel, user_acc
    );

    // Kp 게인을 15.0으로 설정하여 목표를 향해 꽤 강하게 당겨지도록 합니다.
    attrj.set_kp(TrajAttrJ::angles_t::Constant(15.0));
    attrj.set_goal_angles(goal1_q);

    std::ofstream csv_file("fk_output.csv");
    csv_file << "time";
    for(int i=1; i<=6; ++i) csv_file << ",q" << i;
    for(int i=0; i<=6; ++i) csv_file << ",x" << i << ",y" << i << ",z" << i;
    csv_file << "\n";

    double dt = 0.001;
    double current_time = 0.0;
    bool goal_changed = false;

    // 3초 동안 시뮬레이션
    while (current_time <= 3.0) {
        
        // 🌟 1.2초 시점에서 목표점을 완전히 반대로 바꿔버립니다!
        if (current_time > 1.2 && !goal_changed) {
            std::cout << "\n[1.2초 경과] 목표 지점을 반대 방향으로 급변경합니다!" << std::endl;
            attrj.set_goal_angles(goal2_q);
            goal_changed = true;
        }

        attrj.update(dt);
        
        auto q = attrj.angles();
        std::vector<Eigen::Isometry3d> poses = my_robot.forward_kinematics_all(q);

        csv_file << std::fixed << std::setprecision(4) << current_time;
        for(int i=0; i<6; ++i) csv_file << "," << std::setprecision(6) << q(i);
        for(int i=0; i<=6; ++i) {
            csv_file << "," << poses[i].translation().x() << "," 
                     << poses[i].translation().y() << "," << poses[i].translation().z();
        }
        csv_file << "\n";

        current_time += dt;
    }

    csv_file.close();
    std::cout << "완료! 파이썬 스크립트로 궤적을 확인해 보세요." << std::endl;

    return 0;
}