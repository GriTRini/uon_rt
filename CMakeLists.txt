cmake_minimum_required(VERSION 3.15)
project(UonRtControl)

# C++20 표준 강제 적용
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# CMake 경고(CMP0148) 해결: 구형 파이썬 검색 정책 유지
if(POLICY CMP0148)
    cmake_policy(SET CMP0148 OLD)
endif()

# Pybind11 및 Eigen3 패키지 찾기
find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)

# ==============================================================================
# 🐍 Python Binding (Pybind11) 빌드 설정
# ==============================================================================

# 1. 모듈 이름 정의
pybind11_add_module(dsr_rt_control rt_control_bind/dsr_robot.cpp)

# 🌟 2. 핵심: 파이썬 버전 꼬리표를 떼어내고 무조건 ".so"로 강제 고정!
set_target_properties(dsr_rt_control PROPERTIES SUFFIX ".so")

# 3. 헤더 인클루드 경로 설정
target_include_directories(dsr_rt_control PRIVATE 
    ${CMAKE_SOURCE_DIR}/rt_control
    ${CMAKE_SOURCE_DIR}/drfl/include
    /usr/include/eigen3
)

# 4. 기존 DRFL 라이브러리 링크 디렉토리
target_link_directories(dsr_rt_control PRIVATE 
    ${CMAKE_SOURCE_DIR}/drfl/library/Linux/64bits/amd64/24.04
)

# 5. 라이브러리 링크
target_link_libraries(dsr_rt_control PRIVATE 
    DRFL PocoNet PocoFoundation pthread rt
)

# 🌟 6. 빌드 성공 시, 깔끔해진 dsr_rt_control.so 파일을 파이썬 폴더로 자동 복사!
add_custom_command(TARGET dsr_rt_control POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:dsr_rt_control> ${CMAKE_SOURCE_DIR}/rt_control_py/m1013/
)