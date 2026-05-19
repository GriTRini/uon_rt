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
    // stop 명령을 실행 가능하다.
    // 로봇은 stop을 실행하면서 flush 된 실행 대기중인 모션이 있다면
    // 모두 삭제된다.

    CALL(clink_rpc_robot_servo_switch_set(cbox_id, robot_id, CLINK_SWITCH_ON));
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // home 으로 이동
    move_homing();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    const uint32_t NUM_OF_MOTIONS = 2;
    const uint32_t NUM_OF_JOINTS = 6;

    clink_float_t angles[NUM_OF_MOTIONS][NUM_OF_JOINTS] = {
        { 180.0, 0.0, -90.0, -180.0, 180.0, 180.0 },
        {-180.0, -180.0, 90.0, 0.0, -180.0, -180.0 }
    };

    const clink_float_t spd = 20;			// [deg/s]
    const clink_float_t acc = spd * 2.0;	// [deg/s^2]
    const clink_float_t jerk = 0.5;			// 0 ~ 1 사이 값

    uint32_t cmd_id = 0;
    CALL(clink_rpc_motion_command_robot_joint_create(cbox_id, &cmd_id));
    CALL(clink_rpc_motion_command_robot_robot_id_set(cbox_id, cmd_id, robot_id));
    for (uint32_t j_idx = 0; j_idx < NUM_OF_JOINTS; j_idx++)
    {
        CALL(clink_rpc_motion_command_robot_joint_angle_end_set(cbox_id, cmd_id, j_idx, angles[0][j_idx]));
    }
    CALL(clink_rpc_motion_command_velocity_profiler_set(cbox_id, cmd_id, CLINK_VELOCITY_PROFILE_ASYMMETRIC_S_CURVE));
    CALL(clink_rpc_motion_command_acc_max_set(cbox_id, cmd_id, acc));
    CALL(clink_rpc_motion_command_dec_max_set(cbox_id, cmd_id, acc));
    CALL(clink_rpc_motion_command_speed_max_set(cbox_id, cmd_id, spd));
    CALL(clink_rpc_motion_command_jerk_percentage_set(cbox_id, cmd_id, jerk));
    CALL(clink_rpc_motion_command_blending_motion_mode_set(cbox_id, cmd_id, CLINK_SWITCH_OFF));
    CALL(clink_rpc_motion_command_queue_add(cbox_id, cmd_id));		// 모션을 queue 에 삽입한다.
    CALL(clink_rpc_motion_command_queue_flush(cbox_id));			// queue 에 삽입되어 있는 모션들을 일괄적으로 전송한다(모션 시작).
    CALL(clink_rpc_motion_command_destroy(cbox_id, cmd_id));

    CALL(clink_rpc_motion_command_robot_joint_create(cbox_id, &cmd_id));
    CALL(clink_rpc_motion_command_robot_robot_id_set(cbox_id, cmd_id, robot_id));
    for (uint32_t j_idx = 0; j_idx < NUM_OF_JOINTS; j_idx++)
    {
        CALL(clink_rpc_motion_command_robot_joint_angle_end_set(cbox_id, cmd_id, j_idx, angles[1][j_idx]));
    }
    CALL(clink_rpc_motion_command_velocity_profiler_set(cbox_id, cmd_id, CLINK_VELOCITY_PROFILE_ASYMMETRIC_S_CURVE));
    CALL(clink_rpc_motion_command_acc_max_set(cbox_id, cmd_id, acc));
    CALL(clink_rpc_motion_command_dec_max_set(cbox_id, cmd_id, acc));
    CALL(clink_rpc_motion_command_speed_max_set(cbox_id, cmd_id, spd));
    CALL(clink_rpc_motion_command_jerk_percentage_set(cbox_id, cmd_id, jerk));
    CALL(clink_rpc_motion_command_blending_motion_mode_set(cbox_id, cmd_id, CLINK_SWITCH_OFF));
    CALL(clink_rpc_motion_command_queue_add(cbox_id, cmd_id));		// 모션을 queue 에 삽입한다.
    CALL(clink_rpc_motion_command_queue_flush(cbox_id));			// queue 에 삽입되어 있는 모션들을 일괄적으로 전송한다(모션 시작).
    CALL(clink_rpc_motion_command_destroy(cbox_id, cmd_id));
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));


    CALL(clink_rpc_robot_stop(cbox_id, robot_id, 0.5));

    char_t valid_event = -1;
    CALL(clink_rpc_system_wait_event_group_subgroup(
        cbox_id,
        CLINK_EVENT_GRP_MOTION_COMMAND,
        CLINK_EVENT_SUBGRP_MOTION_COMMAND_STOP_SETTLED_DOWN,
        1000000,
        1,
        &valid_event));

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    CALL(clink_rpc_robot_servo_switch_set(cbox_id, robot_id, CLINK_SWITCH_OFF));
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}
