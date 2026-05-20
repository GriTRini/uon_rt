#pragma once

#include <memory>
#include <string>
#include <algorithm>
#include <iostream>

#include "robot_base.hpp"
#include "../dsr/dsr_robot.hpp"
#include "../hcr/hcr_robot.hpp"

namespace rt_control {

class RobotFactory {
public:
    static std::unique_ptr<RobotBase> create(const std::string& model_name) {
        // 소문자로 변환하여 비교 (예: "M1013" -> "m1013")
        std::string lower_model = model_name;
        std::transform(lower_model.begin(), lower_model.end(), lower_model.begin(), ::tolower);

        if (lower_model.find("m1013") == 0) {
            return std::make_unique<DsrRobot>(model_name);
        } 
        // 🌟 한화 시리즈 판단 정상화
        else if (lower_model.find("hcr14") == 0) {
            // 주석을 해제하고 실제 HcrRobot 객체를 생성하여 반환합니다.
            return std::make_unique<HcrRobot>(model_name);
        }

        std::cerr << "[Error] 지원하지 않거나 인식할 수 없는 로봇 모델입니다: " << model_name << std::endl;
        return nullptr;
    }
};

} // namespace rt_control