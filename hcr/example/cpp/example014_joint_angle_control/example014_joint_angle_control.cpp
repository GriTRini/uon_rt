//------------------------------------------------------------------------------
//
// Clink RPC Client API
// Copyright © Hanwha. All rights reserved.
//
// Incheol Jeong (ic.jeong at hanwha.com)
// Kiho Jo (kiho0520 at hanwha.com)
// Yongjin Kim (kim.jy at hanwha.com)
// Hyeyeon Park (hyeyeon.park at hanwha.com)
// Sungpil Mun (sungpil.mun at hanwha.com)
//
//------------------------------------------------------------------------------

#include <clink_api_rpc.h>
#include <clink_api_rpc_system.h>
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
	// 사용자가 지정한 조인트 각도[deg] 값들을 순차적으로 적용하여 진행한다.
    // 입력한 각도 값의 변화에 따라 조인트 단위의 동작을 확인할 수 있다.
	// 과도한 각도 변화는 에러로 처리될 수 있으며
    // 입력 값이 로봇 사양을 초과하는 경우, 각도 변화가 조정될 수 있다.
	// 본 예제는 J6 조인트를 약 30도 회전 시킨다.
    CALL(clink_rpc_robot_servo_switch_set(cbox_id, robot_id, CLINK_SWITCH_ON));
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    const uint32_t NUM_OF_JOINTS = 6;

    clink_float_t start_angles[NUM_OF_JOINTS];
    start_angles[0] = 0.0;
    start_angles[1] = -90.0;
    start_angles[2] = -90.0;
    start_angles[3] = -90.0;
    start_angles[4] = 90.0;
    start_angles[5] = 0.0;

    clink_float_t target_angle[NUM_OF_JOINTS];
    target_angle[0] = 0.0;
    target_angle[1] = -90.0;
    target_angle[2] = -90.0;
    target_angle[3] = -90.0;
    target_angle[4] = 90.0;
    target_angle[5] = 0.0;

    // 초기 각도로 이동
    CALL(clink_rpc_just_robot_joint_move(
        cbox_id,
        robot_id,
        0.5,
        NUM_OF_JOINTS,
        start_angles));

    // 로봇 모션이 끝날 때까지 wait
    // 해당 함수는 지정된 이벤트가 도착할 때까지 지정된 시간만큼 blocking
    char_t valid_event = -1;
    clink_rpc_system_wait_event_group_subgroup(
        cbox_id,											// control box ID
        CLINK_EVENT_GRP_MOTION_COMMAND,						// event group
        CLINK_EVENT_SUBGRP_MOTION_COMMAND_SETTLED_DOWN,		// event subgroup
        10000,												// 대기 시간 [ms]
        1,													// 해당 이벤트를 내부 이벤트 큐에서 삭제한다.
        &valid_event);										// 정해진 시간내에 이벤트 발생 시 1, 발생안했을 시 0 반환

    // joint chaser 시작
    CALL(clink_rpc_robot_joint_chaser_switch_set(cbox_id, robot_id, CLINK_SWITCH_ON));

    for (uint32_t offset = 0; offset < 300; offset+=2) // 약 30도까지
    {
        target_angle[5] = start_angles[5] + offset * 0.1;
        CALL(clink_rpc_robot_joint_chaser_move(cbox_id, robot_id, NUM_OF_JOINTS, target_angle));
        std::this_thread::sleep_for(std::chrono::milliseconds(15));
    }

    // joint chaser 종료
    CALL(clink_rpc_robot_joint_chaser_switch_set(cbox_id, robot_id, CLINK_SWITCH_OFF));

    // home 으로 이동
    move_homing();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    CALL(clink_rpc_robot_servo_switch_set(cbox_id, robot_id, CLINK_SWITCH_OFF));
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}
