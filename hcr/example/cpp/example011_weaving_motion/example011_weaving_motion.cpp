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
	// 용접향 weaving 모션에 대한 설명이다.
	// 기본적으로 tcp motion 과 동일하게 호출하며
	// weaving과 관련한 추가적인 옵션이 붙는다.

    CALL(clink_rpc_robot_servo_switch_set(cbox_id, robot_id, CLINK_SWITCH_ON));
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // home 으로 이동
    move_homing();
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	clink_float_t cmd_pos_x, cmd_pos_y, cmd_pos_z;
	clink_float_t cmd_ort_x, cmd_ort_y, cmd_ort_z;
	CALL(clink_rpc_robot_tcp_pose_command_get(
		cbox_id,
		robot_id,
		&cmd_pos_x,
		&cmd_pos_y,
		&cmd_pos_z,
		&cmd_ort_x,
		&cmd_ort_y,
		&cmd_ort_z));

	uint32_t cmd_id = 0;
	clink_float_t spd = 10;
	clink_float_t acc = spd * 2;
	clink_float_t jerk = 0.5;

	// weaving motion create
	CALL(clink_rpc_motion_command_robot_tcp_welding_create(cbox_id, &cmd_id));

	// tcp motion 설정과 기본적으로 동일
	CALL(clink_rpc_motion_command_robot_robot_id_set(cbox_id, cmd_id, robot_id));
	CALL(clink_rpc_motion_command_robot_tcp_speed_start_set(cbox_id, cmd_id, 0));
	CALL(clink_rpc_motion_command_robot_tcp_speed_end_set(cbox_id, cmd_id, 0));
	CALL(clink_rpc_motion_command_robot_tcp_position_end_set(cbox_id, cmd_id, cmd_pos_x, cmd_pos_y + 203, cmd_pos_z));
	CALL(clink_rpc_motion_command_robot_tcp_orientation_end_set(cbox_id, cmd_id, cmd_ort_x, cmd_ort_y, cmd_ort_z));
	CALL(clink_rpc_motion_command_robot_tcp_interpolator_set(cbox_id, cmd_id, CLINK_INTERPOLATOR_LINEAR));
	CALL(clink_rpc_motion_command_robot_tcp_coordinate_set(cbox_id, cmd_id, CLINK_REF_COORDINATE_BASE));
	CALL(clink_rpc_motion_command_robot_orientation_mode_set(cbox_id, cmd_id, CLINK_ORIENTATION_MODE_AXIS));
	CALL(clink_rpc_motion_command_velocity_profiler_set(cbox_id, cmd_id, CLINK_VELOCITY_PROFILE_ASYMMETRIC_S_CURVE));
	CALL(clink_rpc_motion_command_acc_max_set(cbox_id, cmd_id, acc));
	CALL(clink_rpc_motion_command_dec_max_set(cbox_id, cmd_id, acc));
	CALL(clink_rpc_motion_command_speed_max_set(cbox_id, cmd_id, spd));
	CALL(clink_rpc_motion_command_jerk_percentage_set(cbox_id, cmd_id, jerk));
	CALL(clink_rpc_motion_command_blending_motion_mode_set(cbox_id, cmd_id, CLINK_SWITCH_ON));
	CALL(clink_rpc_motion_command_radius_for_blending_set(cbox_id, cmd_id, 50));
	CALL(clink_rpc_motion_command_blending_auto_start_set(cbox_id, cmd_id, CLINK_SWITCH_ON));
	CALL(clink_rpc_motion_command_robot_tcp_pose_type_set(cbox_id, cmd_id, CLINK_POSE_TYPE_FLANGE));

	// weaving 과 관련된 추가 파라미터
	// weaving motion enable
	CALL(clink_rpc_motion_command_robot_tcp_welding_weaving_motion_switch_set(cbox_id, cmd_id, CLINK_SWITCH_ON));
	// 주파수 설정
	CALL(clink_rpc_motion_command_robot_tcp_welding_frequency_set(cbox_id, cmd_id, 1.0));
	// 좌측 진폭 설정
	CALL(clink_rpc_motion_command_robot_tcp_welding_left_amplitude_set(cbox_id, cmd_id, 4.0));
	// 우측 진폭 설정
	CALL(clink_rpc_motion_command_robot_tcp_welding_right_amplitude_set(cbox_id, cmd_id, 4.0));
	// 위빙 패턴 설정(zigzag, ellipse, ....)
	CALL(clink_rpc_motion_command_robot_tcp_welding_weaving_pattern_set(cbox_id, cmd_id, CLINK_WELDING_WEAVING_PATTERN_ZIGZAG));
	// zigzag의 경우에는 R, U, F 비율을 설정한다.
	CALL(clink_rpc_motion_command_robot_tcp_welding_zigzag_ratio_set(cbox_id, cmd_id, 0.5, 0.0, 0.5, 0.0));
	

	CALL(clink_rpc_motion_command_queue_add(cbox_id, cmd_id));
	CALL(clink_rpc_motion_command_queue_flush(cbox_id));
	std::this_thread::sleep_for(std::chrono::seconds(3));

	// 진행 도중 주파수를 변경한다.
	CALL(clink_rpc_motion_command_robot_tcp_welding_frequency_change(cbox_id, cmd_id, 2.0));
	std::this_thread::sleep_for(std::chrono::seconds(3));

	// 진행 도중 진폭을 븐경한다.
	CALL(clink_rpc_motion_command_robot_tcp_welding_left_amplitude_change(cbox_id, cmd_id, 2.0));
	CALL(clink_rpc_motion_command_robot_tcp_welding_right_amplitude_change(cbox_id, cmd_id, 2.0));
	CALL(clink_rpc_motion_command_destroy(cbox_id, cmd_id));

	char_t valid_event = -1;
	CALL(clink_rpc_system_wait_event_group_subgroup_sender_id(
		cbox_id,
		CLINK_EVENT_GRP_MOTION_COMMAND,
		CLINK_EVENT_SUBGRP_MOTION_COMMAND_SETTLED_DOWN,
		cmd_id,
		1000000,
		1,
		&valid_event));	
	

	move_homing();
    CALL(clink_rpc_robot_servo_switch_set(cbox_id, robot_id, CLINK_SWITCH_OFF));
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}
