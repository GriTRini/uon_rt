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
    // joint 별 jog 명령
    // jog 호출 시 입력한 거리만큼 진행한다,
    // 각도 범위보다 큰 값을 입력했을 때 각도 범위 값까지 진행한다.
    // stop 명령을 통해 멈춘다.

    CALL(clink_rpc_robot_servo_switch_set(cbox_id, robot_id, CLINK_SWITCH_ON));
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    move_homing();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // 입력 값
    // joint 축 index [0~5] 범위
    // 정방향/역방향
    // 속도 [degree/s]
    // 거리 [degree]
    CALL(clink_rpc_robot_jog_joint(cbox_id, robot_id, 4, CLINK_DIRECTION_POSITIVE, 30, 1000));
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    CALL(clink_rpc_robot_stop(cbox_id, robot_id, 0.5));
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    CALL(clink_rpc_robot_servo_switch_set(cbox_id, robot_id, CLINK_SWITCH_OFF));
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}
