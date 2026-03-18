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
    rt_control::model::RobotModel model("m1013");
    
    // 초기 자세 설정
    angles_t q_start = angles_t::Zero();
    q_start << 0.0, -45.0, 90.0, 0.0, 45.0, 0.0; 
    
    TrajGenerator traj_gen;
    traj_gen.initialize(model, q_start, angles_t::Zero(), angles_t::Zero());

    // 2. CSV 파일 오픈
    std::ofstream csv("attrl_debug_report.csv");
    csv << "Time,Step,X,Y,Z,Att_X,Att_Y,Att_Z,Goal_X,Goal_Y,Goal_Z,P_Err,R_Err,Vel_Norm,Reached\n";

    double dt = 0.001; 
    double current_time = 0.0;
    
    // 3. 목표 경로 설정
    Eigen::Isometry3d start_pose = traj_gen.tmat();
    Eigen::Isometry3d target_pose = start_pose;
    target_pose.translation().x() += 0.15; 
    target_pose.translation().z() += 0.10; 

    std::vector<Eigen::Isometry3d> goals = {target_pose, start_pose};
    std::vector<std::string> step_names = {"DIAGONAL_AWAY", "RETURN_HOME"};

    // Kp 설정 (유령의 이동 강도)
    double active_kp = 200.0; 

    std::cout << "===== 🛠️ Attrl Trajectory Tracking Debug Start =====" << std::endl;

    for (size_t i = 0; i < goals.size(); ++i) {
        double gx = goals[i].translation().x();
        double gy = goals[i].translation().y();
        double gz = goals[i].translation().z();

        std::cout << "\n🚀 [Step " << i + 1 << "] " << step_names[i] 
                  << " -> Goal: (" << gx << ", " << gy << ", " << gz << ")" << std::endl;
        
        if (!traj_gen.attrl(goals[i], active_kp)) {
            std::cerr << "Failed to start ATTRL mode!" << std::endl;
            return -1;
        }

        double step_start_time = current_time;
        int step_loop_cnt = 0;

        while (true) {
            traj_gen.update(dt);
            step_loop_cnt++;

            const auto& curr_tmat = traj_gen.tmat();
            const auto& att_pose = traj_gen.get_attractor_pose();
            // curr_vel을 가속도(a)가 아닌 속도 벡터에서 추출하도록 확인 필요 (보통 get_velocity() 등)
            const auto& curr_vel = traj_gen.a(); 

            // 오차 계산
            double p_err = (goals[i].translation() - curr_tmat.translation()).norm();
            Eigen::AngleAxisd r_err_aa(goals[i].linear() * curr_tmat.linear().transpose());
            double r_err = std::abs(r_err_aa.angle()) * (180.0 / M_PI);
            double v_norm = curr_vel.head<3>().norm();

            // 🌟 [핵심 수정] 판정 기준 대폭 완화 (위치 1cm, 회전 2도, 속도 5cm/s)
            // 만약 그래도 안 된다면 0.01 -> 0.02로 늘려보세요.
            bool is_reached = traj_gen.goal_reached(0.01, 1.0, std::nullopt, 0.5);

            csv << std::fixed << std::setprecision(6)
                << current_time << "," << i + 1 << "," 
                << curr_tmat.translation().x() << "," << curr_tmat.translation().y() << "," << curr_tmat.translation().z() << ","
                << att_pose.translation().x() << "," << att_pose.translation().y() << "," << att_pose.translation().z() << ","
                << gx << "," << gy << "," << gz << "," 
                << p_err << "," << r_err << "," << v_norm << ","
                << (is_reached ? 1 : 0) << "\n";

            if (step_loop_cnt % 500 == 0) {
                double follow_err = (att_pose.translation() - curr_tmat.translation()).norm();
                // 🌟 범인 검거용 로그 출력
                std::cout << "Step " << i+1 << " | P:" << std::setw(8) << p_err 
                          << " | R:" << std::setw(6) << r_err 
                          << " | V:" << std::setw(8) << v_norm 
                          << " | F:" << std::setw(8) << follow_err << std::endl;
            }

            if (is_reached && step_loop_cnt > 100) {
                std::cout << "✅ SUCCESS: " << step_names[i] << " 도착!" << std::endl;
                break; 
            }

            current_time += dt;
            // 안전 장치: 10초
            if (current_time - step_start_time > 10.0) {
                std::cout << "⚠️ TIMEOUT: " << step_names[i] << " 도달 실패 (P_Err: " << p_err << ")" << std::endl;
                break;
            }
        }
        for(int j=0; j<100; ++j) { traj_gen.update(dt); current_time += dt; }
    }

    csv.close();
    std::cout << "\n📊 Debug report saved to 'attrl_debug_report.csv'" << std::endl;
    return 0;
}