// /home/uon/uon_rt/rt_control/rt_data.hpp 내부에 추가
#pragma once
#include <chrono>
#include <string>

namespace rt_control {
    struct LogAlarm {
        std::chrono::system_clock::time_point time;
        int level;
        int group;
        int index;
        std::string param1;
        std::string param2;
        std::string param3;
    };
}
