cmake_minimum_required(VERSION 3.12)
project(uon_rt_binding)

# 0. 정책 설정
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

# 1. C++ 표준 설정
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 2. 필수 패키지 찾기
find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

# 3. 🌟 Doosan SDK 경로 설정 (제공해주신 절대 경로 기준)
set(DSR_SDK_DIR "/home/uon/uon_rt/drfl")
set(DSR_SDK_INC "${DSR_SDK_DIR}/include")

# 🌟 라이브러리 경로 (amd64 / 24.04 버전 선택)
set(DSR_LIB_PATH "${DSR_SDK_DIR}/library/Linux/64bits/amd64/24.04")

# 4. 헤더 경로 설정
include_directories(
    ${PROJECT_SOURCE_DIR}               # rt_control, rt_bind 루트
    ${PROJECT_SOURCE_DIR}/rt_control    # 로봇 엔진 헤더
    ${PROJECT_SOURCE_DIR}/rt_bind       # 바인딩 어댑터 헤더
    ${EIGEN3_INCLUDE_DIRS}              # Eigen3
    ${DSR_SDK_INC}                      # Doosan SDK 헤더
)

# 5. 컴파일 옵션
if(NOT MSVC)
    add_definitions(-march=native)
    add_definitions(-DEIGEN_MAX_ALIGN_BYTES=32)
    add_definitions(-D_REENTRANT)
endif()

# 6. 파이썬 모듈 빌드
pybind11_add_module(rt_control_cpp_impl
    rt_bind/rt_bind.cpp
)

# 7. 결과물(.so) 출력 경로 설정
set_target_properties(rt_control_cpp_impl PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/rt_py"
)

# 8. 🌟 라이브러리 링크 (에러 해결의 핵심)
# libDRFL.a 와 Poco 라이브러리들을 순서대로 링크해야 합니다.
target_link_libraries(rt_control_cpp_impl PRIVATE 
    Eigen3::Eigen
    Threads::Threads
    "${DSR_LIB_PATH}/libDRFL.a"
    "${DSR_LIB_PATH}/libPocoNet.so"
    "${DSR_LIB_PATH}/libPocoFoundation.so"
    rt  # Real-time library (Linux 기본)
    dl  # Dynamic linking library (Linux 기본)
)

# 9. 🌟 런타임 라이브러리 경로 설정 (RPATH)
# .so 파일들이 실행 시점에 Poco 라이브러리를 찾을 수 있도록 경로를 박아넣습니다.
set_target_properties(rt_control_cpp_impl PROPERTIES
    INSTALL_RPATH "${DSR_LIB_PATH}"
    BUILD_WITH_INSTALL_RPATH TRUE
)