cmake_minimum_required(VERSION 3.16)
project(uon_rt_control)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# [중요] 정적 라이브러리(.a)를 동적 라이브러리(.so)에 녹일 때 필수적인 옵션입니다.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(POLICY CMP0148)
    cmake_policy(SET CMP0148 OLD)
endif()

# 1. 의존성 패키지
find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)

# 2. 경로 설정
set(ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/..")
set(DSR_ROOT "${ROOT_DIR}/drfl")
set(DSR_INC_DIR "${DSR_ROOT}/include")
# Ubuntu 24.04 amd64 기준 경로
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
    "${DSR_LIB_DIR}/libDRFL.a"   # 🌟 정적 링크: 파일 경로 직접 지정 -> uon_rt.so 안에 완전히 녹아 들어감
    PocoFoundation               # 🌟 동적 링크: 이름만 지정 -> 리눅스 시스템에 설치된 libPocoFoundation.so 참조
    PocoNet                      # 🌟 동적 링크: 이름만 지정 -> 리눅스 시스템에 설치된 libPocoNet.so 참조
    pthread
    dl
)

# 7. 파이썬 모듈 이름 깔끔하게 정리
# (uon_rt.cpython-312-x86_64-linux-gnu.so -> uon_rt.so 로 강제 고정)
set_target_properties(uon_rt PROPERTIES SUFFIX ".so")