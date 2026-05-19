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
	// start -> mid(근처) -> end point 를 지나는 모션
	// 직선 + ARC + 직선으로 이루어진 모션으로 볼 수 있다.
	// 중간 ARC 궤적의 반지름은 radius 값을 추가적으로
	// 입력함으로서 이루어진다.
	// 4개의 CIRCULAR 모션이 연결되어 모서리가 둥근
	// 사각형 모양을 그리는 예제다.

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

	const uint32_t NUM_OF_MOTIONS = 4;
	clink_float_t mid_pos[NUM_OF_MOTIONS][3];
	clink_float_t end_pos[NUM_OF_MOTIONS][3];
	clink_float_t end_ort[NUM_OF_MOTIONS][3];

	mid_pos[0][0] = cmd_pos_x + 100.0;
	mid_pos[0][1] = cmd_pos_y;
	mid_pos[0][2] = cmd_pos_z;
	end_pos[0][0] = cmd_pos_x + 100.0;
	end_pos[0][1] = cmd_pos_y + 100.0;
	end_pos[0][2] = cmd_pos_z;

	mid_pos[1][0] = cmd_pos_x + 100.0;
	mid_pos[1][1] = cmd_pos_y + 200.0;
	mid_pos[1][2] = cmd_pos_z;
	end_pos[1][0] = cmd_pos_x;
	end_pos[1][1] = cmd_pos_y + 200.0;
	end_pos[1][2] = cmd_pos_z;

	mid_pos[2][0] = cmd_pos_x - 100.0;
	mid_pos[2][1] = cmd_pos_y + 200.0;
	mid_pos[2][2] = cmd_pos_z;
	end_pos[2][0] = cmd_pos_x - 100.0;
	end_pos[2][1] = cmd_pos_y + 100.0;
	end_pos[2][2] = cmd_pos_z;

	mid_pos[3][0] = cmd_pos_x - 100.0;
	mid_pos[3][1] = cmd_pos_y;
	mid_pos[3][2] = cmd_pos_z;
	end_pos[3][0] = cmd_pos_x;
	end_pos[3][1] = cmd_pos_y;
	end_pos[3][2] = cmd_pos_z;


	end_ort[0][0] = cmd_ort_x;
	end_ort[0][1] = cmd_ort_y;
	end_ort[0][2] = cmd_ort_z;

	end_ort[1][0] = cmd_ort_x;
	end_ort[1][1] = cmd_ort_y;
	end_ort[1][2] = cmd_ort_z;

	end_ort[2][0] = cmd_ort_x;
	end_ort[2][1] = cmd_ort_y;
	end_ort[2][2] = cmd_ort_z;

	end_ort[3][0] = cmd_ort_x;
	end_ort[3][1] = cmd_ort_y;
	end_ort[3][2] = cmd_ort_z;

	const clink_float_t spd = 300;
	const clink_float_t acc = spd * 2.0F;
	const clink_float_t jerk = 0.5;

	for (uint32_t iter = 0; iter < 10; iter++)
	{
		uint32_t cmd_id = 0;
		std::cout << "test loop: " << iter << std::endl;

		for (uint32_t i = 0; i < NUM_OF_MOTIONS; i++)
		{
			CALL(clink_rpc_motion_command_robot_tcp_create(cbox_id, &cmd_id));
			CALL(clink_rpc_motion_command_robot_robot_id_set(cbox_id, cmd_id, robot_id));
			CALL(clink_rpc_motion_command_robot_tcp_speed_start_set(cbox_id, cmd_id, 0));
			CALL(clink_rpc_motion_command_robot_tcp_speed_end_set(cbox_id, cmd_id, 0));
			CALL(clink_rpc_motion_command_robot_tcp_position_mid_set(cbox_id, cmd_id, mid_pos[i][0], mid_pos[i][1], mid_pos[i][2]));
			CALL(clink_rpc_motion_command_robot_tcp_position_end_set(cbox_id, cmd_id, end_pos[i][0], end_pos[i][1], end_pos[i][2]));
			CALL(clink_rpc_motion_command_robot_tcp_orientation_end_set(cbox_id, cmd_id, end_ort[i][0], end_ort[i][1], end_ort[i][2]));
			CALL(clink_rpc_motion_command_robot_tcp_interpolator_set(cbox_id, cmd_id, CLINK_INTERPOLATOR_CIRCULAR));
			
			// CLINK_INTERPOLATOR_CIRCULAR 선택 시 추가 입력
			CALL(clink_rpc_motion_command_robot_tcp_radius_set(cbox_id, cmd_id, 50));
			
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
			CALL(clink_rpc_motion_command_destroy(cbox_id, cmd_id));
		}

		char_t valid_event = -1;
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
