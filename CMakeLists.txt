cmake_minimum_required(VERSION 3.12)
project(uon_rt_binding)

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

# 1. C++ 표준 설정
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 2. 필수 패키지 찾기
find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)

# 3. 헤더 경로 설정
include_directories(
    ${PROJECT_SOURCE_DIR}              # rt_control/rt.hpp 포함 루트 경로
    ${EIGEN3_INCLUDE_DIRS}             # Eigen3 헤더 경로
)

# 4. 컴파일 옵션 (하드웨어 최적화 및 정렬)
if(NOT MSVC)
    add_definitions(-march=native)
    add_definitions(-DEIGEN_MAX_ALIGN_BYTES=32)
endif()

# 5. 파이썬 모듈 빌드
# 만약 rt_bind.cpp 외에 다른 .cpp 파일을 추가했다면 여기에 나열하세요.
pybind11_add_module(rt_control_cpp_impl
    rt_bind/rt_bind.cpp
)

# 6. 🌟 결과물(.so) 출력 경로 설정 (핵심 부분)
# 빌드된 파일이 프로젝트 루트의 'rt_py' 폴더로 가도록 설정합니다.
set_target_properties(rt_control_cpp_impl PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/rt_py"
)

# 7. 라이브러리 링크
target_link_libraries(rt_control_cpp_impl PRIVATE Eigen3::Eigen)