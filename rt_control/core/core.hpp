#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

namespace rt_control {
    // 기본 실수 타입 정의 (double)
    using value_t = double;
    
    // 6자유도 각도, 각속도 등을 담을 Eigen 벡터 정의 (6x1 Matrix)
    using angles_t = Eigen::Matrix<value_t, 6, 1>;

    // Degree to Radian 변환 헬퍼 함수
    inline constexpr value_t deg2rad(value_t deg) {
        return deg * M_PI / 180.0;
    }
}