cmake_minimum_required(VERSION 3.10)
project(uon_rt_project)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Eigen3 REQUIRED)

# 1. 라이브러리 타겟 생성
add_library(rt_control SHARED
    rt_control/trajectory/trajectory_generator.cpp
)

# 🌟 핵심: lib 접두어(prefix)를 제거하는 설정 추가
set_target_properties(rt_control PROPERTIES PREFIX "")

target_include_directories(rt_control PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(rt_control PUBLIC Eigen3::Eigen)