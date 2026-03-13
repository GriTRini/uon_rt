#pragma once

// =================================================================
// RT Control Framework: Master Include Header
// =================================================================

// 1. 코어 및 데이터 타입
#include "core/core.hpp"
#include "rt_data.hpp"

// 2. 로봇 모델 및 기구학 엔진
#include "model/model.hpp"
#include "model/m1013.hpp"

// 3. 궤적 생성기 (수학 엔진)
#include "trajectory/trajectory_trapj.hpp"
#include "trajectory/trajectory_attrj.hpp"
#include "trajectory/trajectory_attrl.hpp"
#include "trajectory/trajectory_stop.hpp"
#include "trajectory/trajectory_playj.hpp"

// 4. 최상위 API 클래스 (파이썬 바인딩용)
// 🌟 루트로 빠져나온 파일들 반영 🌟
#include "sim_robot.hpp"
#include "real_robot.hpp"