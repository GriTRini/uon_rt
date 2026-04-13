#pragma once

#include "rtb_core.hpp"
// 엔진 코어 경로 참조
#include "../../rt_control/core/robot_base.hpp"
#include "../../rt_control/core/robot_factory.hpp"

namespace rtb {
namespace robot {

class RobotFactoryAdapter {
public:
    // Factory를 사용하여 모델명에 맞는 실제 로봇 인스턴스(DsrRobot 등)를 생성
    static std::unique_ptr<rt_control::RobotBase> create(const std::string& model_name) {
        auto robot = rt_control::RobotFactory::create(model_name);
        if (!robot) {
            throw std::runtime_error("[RobotFactory] 해당 모델 생성 실패: " + model_name);
        }
        return robot;
    }
};

} // namespace robot
} // namespace rtb