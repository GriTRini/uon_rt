#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

namespace rt_control {
    // 기본 실수 타입 정의 (double)
    using value_t = double;
    
    using angles_t = Eigen::Matrix<double, 6, 1, Eigen::ColMajor | Eigen::DontAlign>;
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW은 Eigen 타입이 멤버로 포함된 클래스에서 메모리 정렬 문제를 방지하기 위해 필요합니다.
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}