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

# 🌟 3. 프로젝트 루트(uon_rt) 기준 상대 경로 설정
# ${CMAKE_SOURCE_DIR}은 현재 CMakeLists.txt가 있는 uon_rt 폴더를 의미합니다.
set(DSR_SDK_DIR "${CMAKE_SOURCE_DIR}/drfl")
set(DSR_SDK_INC "${DSR_SDK_DIR}/include")
set(DSR_LIB_PATH "${DSR_SDK_DIR}/library/Linux/64bits/amd64/24.04")

set(HCR_SDK_DIR "${CMAKE_SOURCE_DIR}/hcr")
set(HCR_SDK_INC "${HCR_SDK_DIR}/include")
set(HCR_LIB_PATH "${HCR_SDK_DIR}/lib") 
set(HCR_KINE_LIB_PATH "${HCR_LIB_PATH}/kinematics")

# 4. 헤더 경로 설정
include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/rt_control
    ${PROJECT_SOURCE_DIR}/rt_bind
    ${EIGEN3_INCLUDE_DIRS}
    ${DSR_SDK_INC}
    ${HCR_SDK_INC}
)

# 5. 컴파일 옵션 및 링커 플래그 설정
if(NOT MSVC)
    add_definitions(-march=native)
    add_definitions(-DEIGEN_MAX_ALIGN_BYTES=32)
    add_definitions(-D_REENTRANT)
    
    # 👇 [추가] C++ 코드에 HCR SDK의 절대 경로를 넘겨줍니다.
    add_definitions(-DHANWHA_ROOT_PATH="${HCR_SDK_DIR}")
    
    # 🌟 [추가] 중요: RUNPATH 대신 강제로 전통적인 RPATH를 사용하도록 설정합니다.
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--disable-new-dtags")
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
    # --- Hanwha --- 
    "${HCR_LIB_PATH}/libclink_api_rpc_client.so"
    "${HCR_KINE_LIB_PATH}/libkinematics_hcr_3gen.so"
    # --- System ---
    rt dl
)

# 9. RPATH 설정 (런타임에 모든 .so 파일을 찾을 수 있도록)
# 🌟 [수정 및 강화] 빌드 트리와 인스톨 트리 모두에서 RPATH가 완전 고정되도록 환경 플래그를 추가로 명시합니다.
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_敵RPATH_USE_LINK_PATH TRUE)

set_target_properties(rt_bind PROPERTIES
    INSTALL_RPATH "${DSR_LIB_PATH};${HCR_LIB_PATH};${HCR_KINE_LIB_PATH}" 
    BUILD_WITH_INSTALL_RPATH TRUE
)