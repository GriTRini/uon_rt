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

# 3. 경로 설정
set(DSR_SDK_DIR "${CMAKE_SOURCE_DIR}/drfl")
set(DSR_LIB_PATH "${DSR_SDK_DIR}/library/Linux/64bits/amd64/24.04")

set(HCR_SDK_DIR "${CMAKE_SOURCE_DIR}/hcr")
set(HCR_LIB_PATH "${HCR_SDK_DIR}/lib") 
set(HCR_KINE_LIB_PATH "${HCR_LIB_PATH}/kinematics")

# 4. 헤더 경로 설정
include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/rt_control
    ${PROJECT_SOURCE_DIR}/rt_bind
    ${EIGEN3_INCLUDE_DIRS}
    ${DSR_SDK_DIR}/include
    ${HCR_SDK_DIR}/include
)

# 5. 빌드 설정
if(NOT MSVC)
    add_definitions(-march=native -DEIGEN_MAX_ALIGN_BYTES=32 -D_REENTRANT)
    add_definitions(-DHANWHA_ROOT_PATH="${HCR_SDK_DIR}")
    # RPATH를 사용하도록 설정
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--disable-new-dtags")
endif()

# 6. 파이썬 모듈 빌드
pybind11_add_module(rt_bind rt_bind/rt_bind.cpp)

# 7. 결과물 경로 (rt_py 폴더)
set_target_properties(rt_bind PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/rt_py"
    OUTPUT_NAME "rt_bind"
    PREFIX ""     
    SUFFIX ".so"
    # 중요: 자기 자신 폴더($ORIGIN)를 라이브러리 탐색 경로로 설정
    INSTALL_RPATH "$ORIGIN"
    BUILD_WITH_INSTALL_RPATH TRUE
)

# 8. 라이브러리 링크
target_link_libraries(rt_bind PRIVATE 
    Eigen3::Eigen
    Threads::Threads
    "${DSR_LIB_PATH}/libDRFL.a"
    "${DSR_LIB_PATH}/libPocoNet.so"
    "${DSR_LIB_PATH}/libPocoFoundation.so"
    "${HCR_LIB_PATH}/libclink_api_rpc_client.so"
    "${HCR_KINE_LIB_PATH}/libkinematics_hcr_3gen.so"
    rt dl
)

# 9. 빌드 후 의존성 라이브러리를 rt_py 폴더로 복사
# 이렇게 하면 rt_py 폴더만 들고 다니면 어디서든 실행 가능합니다.
add_custom_command(TARGET rt_bind POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${HCR_LIB_PATH}/libclink_api_rpc_client.so" "$<TARGET_FILE_DIR:rt_bind>/"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${HCR_KINE_LIB_PATH}/libkinematics_hcr_3gen.so" "$<TARGET_FILE_DIR:rt_bind>/"
    COMMENT "Copying dependency libraries to rt_py directory..."
)