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

# 3. Doosan SDK 경로 설정
set(DSR_SDK_DIR "/home/uon/uon_rt/drfl")
set(DSR_SDK_INC "${DSR_SDK_DIR}/include")
set(DSR_LIB_PATH "${DSR_SDK_DIR}/library/Linux/64bits/amd64/24.04")

# 🌟 3-1. Hanwha SDK 경로 설정 (제공해주신 구조 반영)
set(HCR_SDK_DIR "/home/uon/uon_rt/hcr")
set(HCR_SDK_INC "${HCR_SDK_DIR}/include")
set(HCR_LIB_PATH "${HCR_SDK_DIR}/lib") 
set(HCR_KINE_LIB_PATH "${HCR_LIB_PATH}/kinematics") # 기구학 라이브러리 경로

# 4. 헤더 경로 설정
include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/rt_control
    ${PROJECT_SOURCE_DIR}/rt_bind
    ${EIGEN3_INCLUDE_DIRS}
    ${DSR_SDK_INC}
    ${HCR_SDK_INC}  # 한화 SDK 헤더 경로 추가
)

# 5. 컴파일 옵션
if(NOT MSVC)
    add_definitions(-march=native)
    add_definitions(-DEIGEN_MAX_ALIGN_BYTES=32)
    add_definitions(-D_REENTRANT)
endif()

# 6. 파이썬 모듈 빌드
pybind11_add_module(rt_bind
    rt_bind/rt_bind.cpp
)

# 7. 결과물 파일명 및 경로 설정
set_target_properties(rt_bind PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/rt_py"
    OUTPUT_NAME "rt_bind"
    PREFIX ""     
    SUFFIX ".so"  
)

# 8. 라이브러리 링크
target_link_libraries(rt_bind PRIVATE 
    Eigen3::Eigen
    Threads::Threads
    # --- Doosan ---
    "${DSR_LIB_PATH}/libDRFL.a"
    "${DSR_LIB_PATH}/libPocoNet.so"
    "${DSR_LIB_PATH}/libPocoFoundation.so"
    # --- 🌟 Hanwha (제공된 파일명으로 정확히 링크) --- 
    "${HCR_LIB_PATH}/libclink_api_rpc_client.so"
    "${HCR_KINE_LIB_PATH}/libkinematics_hcr_3gen.so" # 기구학 라이브러리 추가 링크
    # --- System ---
    rt dl
)

# 9. RPATH 설정 (런타임에 모든 .so 파일을 찾을 수 있도록)
set_target_properties(rt_bind PROPERTIES
    # 🌟 한화의 lib 폴더와 kinematics 폴더를 모두 RPATH에 등록
    INSTALL_RPATH "${DSR_LIB_PATH};${HCR_LIB_PATH};${HCR_KINE_LIB_PATH}" 
    BUILD_WITH_INSTALL_RPATH TRUE
)