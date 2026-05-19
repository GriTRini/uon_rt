//------------------------------------------------------------------------------
//
// Clink RPC Client API
// Copyright © Hanwha. All rights reserved.
//
// Incheol Jeong (ic.jeong at hanwha.com)
// Kiho Jo (kiho0520 at hanwha.com)
// Yongjin Kim (kim.jy at hanwha.com)
//
//------------------------------------------------------------------------------

#include <clink_api_rpc.h>
#include <iostream>
#include <thread>

CLINK_API_RESULT err_val;
#define CALL(f)																		                \
	err_val = f;											                                        \
	if(CLINK_API_RESULT_OK != err_val) { std::cout << "error code: " << err_val << std::endl; };	

extern uint32_t cbox_id;
extern uint32_t robot_id;
extern void move_homing();

void clink_api_test()
{
    // linear 계열 jog 명령
    // x, y, z, rx, ry, rz jog 이동 명령
    // 또는 일반적으로 진행 벡터(x, y, z) 또는 회전 벡터(rx, ry, rz) 입력 가능
    // stop 명령을 통해 멈춘다.

    CALL(clink_rpc_robot_servo_switch_set(cbox_id, robot_id, CLINK_SWITCH_ON));
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    move_homing();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    // 입력 값
    // 기준 좌표계(base 또는 tcp 끝단 기준)
    // 정방향/역방향
    // 속도 [mm/s]
    // 거리 [mm]
    CALL(clink_rpc_robot_jog_tcp_position_x(cbox_id, robot_id, CLINK_REF_COORDINATE_BASE, CLINK_DIRECTION_POSITIVE, 30, 10000));
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    CALL(clink_rpc_robot_stop(cbox_id, robot_id, 0.5));
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // 입력 값
    // 기준 좌표계(base 또는 tcp 끝단 기준)
    // 정방향/역방향
    // 속도 [deg/s]
    // 거리 [deg]
    CALL(clink_rpc_robot_jog_tcp_orientation_x(cbox_id, robot_id, CLINK_REF_COORDINATE_BASE, CLINK_DIRECTION_POSITIVE, 30, 10000));
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    CALL(clink_rpc_robot_stop(cbox_id, robot_id, 0.5));
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    CALL(clink_rpc_robot_servo_switch_set(cbox_id, robot_id, CLINK_SWITCH_OFF));
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}
