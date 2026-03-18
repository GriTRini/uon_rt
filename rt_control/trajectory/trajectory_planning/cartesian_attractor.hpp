#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <algorithm>

namespace rt_control {
namespace trajectory {
namespace planning {

class CartesianAttractor {
public:
    using tmat_t = Eigen::Isometry3d;
    using a_t = Eigen::Matrix<double, 6, 1>;

    tmat_t pose;         // 현재 유령의 위치/자세
    a_t vel;             // 현재 유령의 속도 (Linear, Angular)
    tmat_t goal_pose;    // 최종 목적지
    a_t goal_vel;        // 목적지에서의 속도 (기본 0)
    
    double kp;           // 추종 게인

    // 하드웨어/사용자 제한치
    double peak_vel;
    double peak_angvel;
    double peak_acc;
    double peak_angacc;

public:
    CartesianAttractor() = default;

    /**
     * @brief TrajAttrL 생성자의 호출부와 인자 순서를 완벽히 일치시킴
     */
    void init(const tmat_t& start_pose, double init_kp, 
              double p_vel, double p_angvel, double p_acc, double p_angacc) 
    {
        pose = start_pose;
        goal_pose = start_pose; // 초기 목적지는 현재 위치와 동일하게 설정
        vel.setZero(); 
        goal_vel.setZero();
        
        kp = init_kp;
        peak_vel = p_vel;
        peak_angvel = p_angvel;
        peak_acc = p_acc;
        peak_angacc = p_angacc;
    }

    void update(double dt) {
        if (dt <= 0.0 || kp <= 0.0) return;

        // 임계 감쇠(Critical Damping) 조건 계산: 오버슈트 방지
        double kd = 2.0 * std::sqrt(kp);
        a_t acc = a_t::Zero();

        // 1. 선형(Translation) 업데이트
        Eigen::Vector3d p_err = goal_pose.translation() - pose.translation();
        Eigen::Vector3d v_err = goal_vel.head<3>() - vel.head<3>();
        acc.head<3>() = kp * p_err + kd * v_err;

        clip_norm(acc.head<3>(), peak_acc);      // 가속도 제한
        pose.translation() += vel.head<3>() * dt; // 위치 적분
        vel.head<3>() += acc.head<3>() * dt;      // 속도 적분
        clip_norm(vel.head<3>(), peak_vel);       // 속도 제한

        // 2. 회전(Rotation) 업데이트 (SO(3) Error)
        // 현재 자세에서 목표 자세로 가는 회전 행렬의 차이를 구함
        Eigen::AngleAxisd err_aa(goal_pose.linear() * pose.linear().transpose());
        Eigen::Vector3d r_err = err_aa.angle() * err_aa.axis();
        Eigen::Vector3d w_err = goal_vel.tail<3>() - vel.tail<3>();
        acc.tail<3>() = kp * r_err + kd * w_err;

        clip_norm(acc.tail<3>(), peak_angacc);    // 각가속도 제한
        
        // 지수 사상(Exponential Map)을 이용한 회전 업데이트
        Eigen::Vector3d rvec_prog = vel.tail<3>() * dt;
        if (rvec_prog.norm() > 1e-9) {
            pose.linear() = Eigen::AngleAxisd(rvec_prog.norm(), rvec_prog.normalized()) * pose.linear();
        }
        
        vel.tail<3>() += acc.tail<3>() * dt;      // 각속도 적분
        clip_norm(vel.tail<3>(), peak_angvel);    // 각속도 제한
    }

private:
    // 벡터의 크기가 제한치를 넘을 경우 방향은 유지하고 크기만 절삭
    void clip_norm(Eigen::Ref<Eigen::Vector3d> vec, double absmax) const noexcept {
        double norm = vec.norm();
        if (norm > absmax && norm > 1e-9) {
            vec *= (absmax / norm);
        }
    }
};

} // namespace planning
} // namespace trajectory
} // namespace rt_control