cmake_minimum_required(VERSION 3.12)
project(uon_rt_binding)

# 1. C++ 표준 설정 (std::optional, std::chrono 등을 위해 C++17 필요)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 2. 필수 패키지 찾기 (Python & pybind11)
find_package(pybind11 REQUIRED)

# 3. Eigen3 찾기 (로봇 연산의 핵심)
# 시스템에 설치되어 있다면 아래 명령으로 찾습니다.
find_package(Eigen3 REQUIRED)

# 4. 헤더 경로 설정
include_directories(
    ${PROJECT_SOURCE_DIR}              # /home/uon/uon_rt (rt_control 헤더 참조용)
    ${EIGEN3_INCLUDE_DIRS}             # Eigen3 헤더
)

# 5. 파이썬 모듈 빌드
# 모듈 이름은 'rt_control_cpp_impl'로 설정 (rt_py/__init__.py에서 import할 이름)
pybind11_add_module(rt_control_cpp_impl
    rt_bind/rt_bind.cpp
    # 만약 다른 .cpp 파일이 추가된다면 여기에 나열하세요.
)

# 6. 출력 경로 설정
# 빌드된 .so 파일이 'rt_py' 폴더로 바로 들어가도록 설정
set_target_properties(rt_control_cpp_impl PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/rt_py"
)

# 7. 컴파일 옵션 (필요시 추가)
if(MSVC)
    target_compile_options(rt_control_cpp_impl PRIVATE /W4)
else()
    target_compile_options(rt_control_cpp_impl PRIVATE -Wall -Wextra)
endif()