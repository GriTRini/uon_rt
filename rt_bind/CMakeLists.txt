cmake_minimum_required(VERSION 3.16)
project(uon_rt_control)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# [추가] 정적 라이브러리(.a)를 동적 라이브러리(.so)에 녹일 때 필수적인 옵션입니다.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 1. 의존성 패키지
find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)

# 2. 경로 설정
set(ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/..")
set(DSR_ROOT "${ROOT_DIR}/drfl")
set(DSR_INC_DIR "${DSR_ROOT}/include")
set(DSR_LIB_DIR "${DSR_ROOT}/library/Linux/64bits/amd64/24.04")

# 3. 인클루드
include_directories(
    ${ROOT_DIR}
    ${DSR_INC_DIR}
    ${EIGEN3_INCLUDE_DIRS}
)

# 4. 파이썬 모듈 빌드
pybind11_add_module(uon_rt binding.cpp)

# 5. 라이브러리 경로 힌트 (시스템 폴더 우선)
target_link_directories(uon_rt PRIVATE 
    "/usr/lib/x86_64-linux-gnu" 
    "/usr/local/lib" 
    "${DSR_LIB_DIR}"
)

# 6. 라이브러리 링크 (정적 + 동적 혼합)
target_link_libraries(uon_rt PRIVATE
    "${DSR_LIB_DIR}/libDRFL.a"   # 🌟 파일 경로 직접 지정 -> uon_rt.so 안에 완전히 녹아 들어감
    PocoFoundation               # 🌟 이름만 지정 -> 리눅스 시스템의 libPocoFoundation.so를 사용
    PocoNet                      # 🌟 이름만 지정 -> 리눅스 시스템의 libPocoNet.so를 사용
    pthread
    dl
)

# (CMAKE_POSITION_INDEPENDENT_CODE를 설정했으므로 -fPIC는 생략해도 무방합니다)