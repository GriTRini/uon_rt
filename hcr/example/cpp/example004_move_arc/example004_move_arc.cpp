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
	// start -> mid -> end point 를 지나는 원호를 그리는 모션이다.
	// arc 모션은 circle 모션과 대부분 비슷하다.
	// circle 이 end pos를 지나 다시 start 위치까지 돌아온다면
	// arc는 end pos 에서 종료된다.
	// 예제는 반원 두개 명령으로 원을 그리는 모션이다.

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

	clink_float_t mid_pos1[3] = { 0, };
	clink_float_t end_pos1[3] = { 0, };
	clink_float_t mid_pos2[3] = { 0, };
	clink_float_t end_pos2[3] = { 0, };
	clink_float_t end_ort[3] = { 0, };

	mid_pos1[0] = cmd_pos_x + 100.0;
	mid_pos1[1] = cmd_pos_y - 100.0;
	mid_pos1[2] = cmd_pos_z;

	end_pos1[0] = cmd_pos_x + 200.0;
	end_pos1[1] = cmd_pos_y;
	end_pos1[2] = cmd_pos_z;

	mid_pos2[0] = cmd_pos_x + 100.0;
	mid_pos2[1] = cmd_pos_y + 100.0;
	mid_pos2[2] = cmd_pos_z;

	end_pos2[0] = cmd_pos_x;
	end_pos2[1] = cmd_pos_y;
	end_pos2[2] = cmd_pos_z;

	end_ort[0] = cmd_ort_x;
	end_ort[1] = cmd_ort_y;
	end_ort[2] = cmd_ort_z;

	const clink_float_t spd = 300;		   // [mm/s]
	const clink_float_t acc = spd * 2.0;   // [mm/s^2]
	const clink_float_t jerk = 0.5;		   // 0 ~ 1 사이 값
	const uint32_t NUM_OF_ROTATION = 5;
	for (uint32_t iter = 0; iter < 5; iter++)
	{
		uint32_t cmd_id = 0;
		std::cout << "test loop: " << iter << std::endl;


		CALL(clink_rpc_motion_command_robot_tcp_create(cbox_id, &cmd_id));
		CALL(clink_rpc_motion_command_robot_robot_id_set(cbox_id, cmd_id, robot_id));
		CALL(clink_rpc_motion_command_robot_tcp_speed_start_set(cbox_id, cmd_id, 0));
		CALL(clink_rpc_motion_command_robot_tcp_speed_end_set(cbox_id, cmd_id, 0));
		CALL(clink_rpc_motion_command_robot_tcp_position_mid_set(cbox_id, cmd_id, mid_pos1[0], mid_pos1[1], mid_pos1[2]));
		CALL(clink_rpc_motion_command_robot_tcp_position_end_set(cbox_id, cmd_id, end_pos1[0], end_pos1[1], end_pos1[2]));
		CALL(clink_rpc_motion_command_robot_tcp_orientation_end_set(cbox_id, cmd_id, end_ort[0], end_ort[1], end_ort[2]));
		CALL(clink_rpc_motion_command_robot_tcp_interpolator_set(cbox_id, cmd_id, CLINK_INTERPOLATOR_ARC));
		CALL(clink_rpc_motion_command_robot_tcp_coordinate_set(cbox_id, cmd_id, CLINK_REF_COORDINATE_BASE));
		CALL(clink_rpc_motion_command_robot_orientation_mode_set(cbox_id, cmd_id, CLINK_ORIENTATION_MODE_SLERP));
		CALL(clink_rpc_motion_command_velocity_profiler_set(cbox_id, cmd_id, CLINK_VELOCITY_PROFILE_ASYMMETRIC_S_CURVE));
		CALL(clink_rpc_motion_command_acc_max_set(cbox_id, cmd_id, acc));
		CALL(clink_rpc_motion_command_dec_max_set(cbox_id, cmd_id, acc));
		CALL(clink_rpc_motion_command_speed_max_set(cbox_id, cmd_id, spd));
		CALL(clink_rpc_motion_command_jerk_percentage_set(cbox_id, cmd_id, jerk));
		CALL(clink_rpc_motion_command_blending_motion_mode_set(cbox_id, cmd_id, CLINK_SWITCH_OFF));
		CALL(clink_rpc_motion_command_robot_tcp_pose_type_set(cbox_id, cmd_id, CLINK_POSE_TYPE_FLANGE));
		CALL(clink_rpc_motion_command_queue_add(cbox_id, cmd_id));
		CALL(clink_rpc_motion_command_queue_flush(cbox_id));

		char_t valid_event = -1;
		CALL(clink_rpc_system_wait_event_group_subgroup_sender_id(
			cbox_id,
			CLINK_EVENT_GRP_MOTION_COMMAND,
			CLINK_EVENT_SUBGRP_MOTION_COMMAND_SETTLED_DOWN,
			cmd_id,
			1000000,
			1,
			&valid_event));

		// 아래와 같이 motion command create 이 후 재활용 가능
		// 하지만 같은 command ID를 쓰므로 command ID 관리를 위해 한번씩 쓰는 것을 추천
		CALL(clink_rpc_motion_command_robot_tcp_position_mid_set(cbox_id, cmd_id, mid_pos2[0], mid_pos2[1], mid_pos2[2]));
		CALL(clink_rpc_motion_command_robot_tcp_position_end_set(cbox_id, cmd_id, end_pos2[0], end_pos2[1], end_pos2[2]));
		CALL(clink_rpc_motion_command_queue_add(cbox_id, cmd_id));
		CALL(clink_rpc_motion_command_queue_flush(cbox_id));
		CALL(clink_rpc_motion_command_destroy(cbox_id, cmd_id));
		
		CALL(clink_rpc_system_wait_event_group_subgroup_sender_id(
			cbox_id,
			CLINK_EVENT_GRP_MOTION_COMMAND,
			CLINK_EVENT_SUBGRP_MOTION_COMMAND_SETTLED_DOWN,
			cmd_id,
			1000000,
			1,
			&valid_event));
	}

	move_homing();
	CALL(clink_rpc_robot_servo_switch_set(cbox_id, robot_id, CLINK_SWITCH_OFF));
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}
