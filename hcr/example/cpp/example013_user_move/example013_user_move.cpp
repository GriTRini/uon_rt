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
#include <mutex>
#include <conio.h>
#include <atomic>

CLINK_API_RESULT err_val;
#define CALL(f)																		                \
	err_val = f;											                                        \
	if(CLINK_API_RESULT_OK != err_val) { std::cout << "error code: " << err_val << std::endl; };

extern uint32_t cbox_id;
extern uint32_t robot_id;
extern void move_homing();

// position print thread
#define PRINT_TOOL	0x1
#define PRINT_JOINT	0x2
#define PRINT_ALL	0x3
std::thread position_thread;
uint8_t thread_run;
void display_pose(int32_t mode);

// draw rectangle
#define RECT_WIDTH	100
#define RECT_HEIGHT	100
const uint32_t NUM_OF_MOTIONS = 4; // 꼭지점 4개
clink_float_t target_pos[NUM_OF_MOTIONS][3] = { 0, };
clink_float_t target_ort[NUM_OF_MOTIONS][3] = { 0, };
void update_pose(clink_float_t width, clink_float_t height);
void draw_rect (clink_float_t width, clink_float_t height);

// move joint
const uint32_t NUM_OF_JOINTS = 6;
void move_joint(uint32_t joint_idx, clink_float_t joint_angle);

void stop_thread(std::thread & t, uint8_t & is_running) {
	if (is_running) {
		is_running = 0;
		if (t.joinable()) t.join();
	}
}

void display_menu()
{
	stop_thread(position_thread, thread_run);
	std::cout << "\n[T]:draw rectangle (TCP) [J]: move Joint [H]: move Home [P]: Print position [Q]: Quit\n> ";
}

void clink_api_test()
{
	uint8_t running = 1;
    CALL(clink_rpc_robot_servo_switch_set(cbox_id, robot_id, CLINK_SWITCH_ON));
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	std::cout << "\nReady...\n\n";

    // home 으로 이동
    move_homing();
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	display_menu();

	while (running)
	{
        if (_kbhit()) {
            char ch = _getch();

            if (ch == 't' || ch == 'T') {
				stop_thread(position_thread, thread_run);
				std::cout << ch << "\n[T] Drawing rectangle...\n";
				position_thread = std::thread(display_pose, PRINT_TOOL);
				draw_rect(RECT_WIDTH, RECT_HEIGHT);
				display_menu();
            }
			else if (ch == 'j' || ch == 'J') {
				stop_thread(position_thread, thread_run);
                uint32_t id;
				clink_float_t range = 360.0;
				clink_float_t angle;
                std::cout << ch << "\n[J] Joint ID(1~6): J";
				std::cin >> id;

				if (id == 3) range = 170;
				printf("[J] Joint Angle (-%.0f ~ %.0f): ", range, range);
				std::cin >> angle;

				if (id > 0 && id < NUM_OF_JOINTS+1 && (angle >= (range * -1) && angle <= range))
				{
					position_thread = std::thread(display_pose, PRINT_JOINT);
					move_joint(id-1, angle);
				}
				else
				{
					printf("invalid value: J%d, %.2f\n", id, angle);
				}
				display_menu();
			}
			else if (ch == 'h' || ch == 'H') {
				stop_thread(position_thread, thread_run);
				std::cout << ch << "\n[H] Homing...\n";
				position_thread = std::thread(display_pose, PRINT_ALL);
				move_homing();
				display_menu();
			}
			else if (ch == 'p' || ch == 'P') {
				stop_thread(position_thread, thread_run);
				std::cout << ch << "\n[P] Print position\n";
				position_thread = std::thread(display_pose, PRINT_ALL);
				std::this_thread::sleep_for(std::chrono::milliseconds(500));
				display_menu();
			}
			else if (ch == 'q' || ch == 'Q') {
				running = 0;
				std::cout << ch << "\n[Q] Stopping...\n\n";
				stop_thread(position_thread, thread_run);
			}
			else
			{
				display_menu();
			}
		}
	}

	move_homing();
    CALL(clink_rpc_robot_servo_switch_set(cbox_id, robot_id, CLINK_SWITCH_OFF));
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	std::cout << "\nExit.\n";
}

void update_pose(clink_float_t width, clink_float_t height)
{
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

	target_pos[0][0] = cmd_pos_x + width;
	target_pos[0][1] = cmd_pos_y;
	target_pos[0][2] = cmd_pos_z;

	target_pos[1][0] = cmd_pos_x + width;
	target_pos[1][1] = cmd_pos_y + height;
	target_pos[1][2] = cmd_pos_z;

	target_pos[2][0] = cmd_pos_x;
	target_pos[2][1] = cmd_pos_y + height;
	target_pos[2][2] = cmd_pos_z;

	target_pos[3][0] = cmd_pos_x;
	target_pos[3][1] = cmd_pos_y;
	target_pos[3][2] = cmd_pos_z;

	target_ort[0][0] = cmd_ort_x;
	target_ort[0][1] = cmd_ort_y;
	target_ort[0][2] = cmd_ort_z;

	target_ort[1][0] = cmd_ort_x;
	target_ort[1][1] = cmd_ort_y;
	target_ort[1][2] = cmd_ort_z;

	target_ort[2][0] = cmd_ort_x;
	target_ort[2][1] = cmd_ort_y;
	target_ort[2][2] = cmd_ort_z;

	target_ort[3][0] = cmd_ort_x;
	target_ort[3][1] = cmd_ort_y;
	target_ort[3][2] = cmd_ort_z;
}

void draw_rect (clink_float_t width, clink_float_t height)
{
	#define X	0
	#define Y	1
	#define Z	2

	uint32_t cmd_id;
	const clink_float_t spd = 300;		  // [mm/s]
	const clink_float_t acc = spd * 2.0;  // [mm/s^2]
	const clink_float_t jerk = 0.5;		  // 0 ~ 1 사이 값

	CLINK_API_RESULT ret_val = CLINK_API_RESULT_OK;
	char_t valid_event = -1;

	update_pose(width, height); // 4개 꼭지점에 대한 TCP 위치와 방향 계산

	// 해당 point로 TCP 4번 이동
	for (uint32_t pt = 0; pt < NUM_OF_MOTIONS; pt++)
	{
		CALL(clink_rpc_motion_command_robot_tcp_create(cbox_id, &cmd_id));
		CALL(clink_rpc_motion_command_robot_robot_id_set(cbox_id, cmd_id, robot_id));
		CALL(clink_rpc_motion_command_robot_tcp_speed_start_set(cbox_id, cmd_id, 0));
		CALL(clink_rpc_motion_command_robot_tcp_speed_end_set(cbox_id, cmd_id, 0));

		// 이동시킬 TCP 위치값(X, Y, Z)
		CALL(clink_rpc_motion_command_robot_tcp_position_end_set(
			cbox_id, cmd_id,
			target_pos[pt][X],
			target_pos[pt][Y],
			target_pos[pt][Z]));

		// 이동시킬 TCP 방향값(RX, RY, RZ)
		CALL(clink_rpc_motion_command_robot_tcp_orientation_end_set(
			cbox_id, cmd_id,
			target_ort[pt][X],
			target_ort[pt][Y],
			target_ort[pt][Z]));

		CALL(clink_rpc_motion_command_robot_tcp_interpolator_set(cbox_id, cmd_id, CLINK_INTERPOLATOR_LINEAR));
		CALL(clink_rpc_motion_command_robot_tcp_coordinate_set(cbox_id, cmd_id, CLINK_REF_COORDINATE_BASE));
		CALL(clink_rpc_motion_command_robot_orientation_mode_set(cbox_id, cmd_id, CLINK_ORIENTATION_MODE_SLERP));
		CALL(clink_rpc_motion_command_velocity_profiler_set(cbox_id, cmd_id, CLINK_VELOCITY_PROFILE_ASYMMETRIC_S_CURVE));
		CALL(clink_rpc_motion_command_acc_max_set(cbox_id, cmd_id, acc));
		CALL(clink_rpc_motion_command_dec_max_set(cbox_id, cmd_id, acc));
		CALL(clink_rpc_motion_command_speed_max_set(cbox_id, cmd_id, spd));
		CALL(clink_rpc_motion_command_jerk_percentage_set(cbox_id, cmd_id, jerk));
		CALL(clink_rpc_motion_command_blending_motion_mode_set(cbox_id, cmd_id, CLINK_SWITCH_ON));
		CALL(clink_rpc_motion_command_radius_for_blending_set(cbox_id, cmd_id, 30));
		CALL(clink_rpc_motion_command_blending_auto_start_set(cbox_id, cmd_id, CLINK_SWITCH_ON));
		CALL(clink_rpc_motion_command_robot_tcp_pose_type_set(cbox_id, cmd_id, CLINK_POSE_TYPE_FLANGE));
		ret_val = CALL(clink_rpc_motion_command_queue_add(cbox_id, cmd_id)); // 모션을 queue 에 삽입한다.
		if (CLINK_API_RESULT_OK != ret_val)
		{
			std::cout << "[ERROR] Cannot add motion.\n";
			return;
		}
		CALL(clink_rpc_motion_command_queue_flush(cbox_id)); // queue 에 삽입되어 있는 모션들을 일괄적으로 전송한다(모션 시작).
		CALL(clink_rpc_motion_command_destroy(cbox_id, cmd_id)); // 생성한 모션을 삭제한다.
	}

	// 모션 완료 이벤트(CLINK_EVENT_SUBGRP_MOTION_COMMAND_SETTLED_DOWN) 발생을 최대 1000s 대기
	CALL(clink_rpc_system_wait_event_group_subgroup_sender_id(
		cbox_id,
		CLINK_EVENT_GRP_MOTION_COMMAND,
		CLINK_EVENT_SUBGRP_MOTION_COMMAND_SETTLED_DOWN,
		cmd_id,
		1000000,
		1,
		&valid_event));

	// 1000s 내 이벤트 미발생
	if (0 == valid_event)
	{
		std::cout << "[ERROR] Event not received within timeout (1000s).\n";
	}
}

void move_joint(uint32_t joint_idx, clink_float_t joint_angle) {
	uint32_t cmd_id;
	clink_float_t angles[NUM_OF_JOINTS];
	const clink_float_t spd = 50;		  // [mm/s]
	const clink_float_t acc = spd * 2.0;  // [mm/s^2]
	const clink_float_t jerk = 0.5;		  // 0 ~ 1 사이 값

	CLINK_API_RESULT ret_val = CLINK_API_RESULT_OK;
	char_t valid_event = -1;

	// 현재 Joint 각도 받아오기
	for (uint32_t joint_idx = 0; joint_idx < NUM_OF_JOINTS; joint_idx++)
	{
		clink_rpc_robot_joint_angle_actual_get(cbox_id, robot_id, joint_idx, &angles[joint_idx]);
	}

	angles[joint_idx] = joint_angle; // 사용자 입력값으로 덮어쓰기

	CALL(clink_rpc_motion_command_robot_joint_create(cbox_id, &cmd_id));
	CALL(clink_rpc_motion_command_robot_robot_id_set(cbox_id, cmd_id, robot_id));

	for (uint32_t joint_idx = 0; joint_idx < NUM_OF_JOINTS; joint_idx++)
	{
		// 설정 각도로 idx번 Joint 이동
		CALL(clink_rpc_motion_command_robot_joint_angle_end_set(cbox_id, cmd_id, joint_idx, angles[joint_idx]));
	}
	CALL(clink_rpc_motion_command_velocity_profiler_set(cbox_id, cmd_id, CLINK_VELOCITY_PROFILE_ASYMMETRIC_S_CURVE));
	CALL(clink_rpc_motion_command_acc_max_set(cbox_id, cmd_id, acc));
	CALL(clink_rpc_motion_command_dec_max_set(cbox_id, cmd_id, acc));
	CALL(clink_rpc_motion_command_speed_max_set(cbox_id, cmd_id, spd));
	CALL(clink_rpc_motion_command_jerk_percentage_set(cbox_id, cmd_id, jerk));
	CALL(clink_rpc_motion_command_blending_motion_mode_set(cbox_id, cmd_id, CLINK_SWITCH_OFF));
	ret_val = CALL(clink_rpc_motion_command_queue_add(cbox_id, cmd_id)); // 모션을 queue 에 삽입한다.
	if (CLINK_API_RESULT_OK != ret_val)
	{
		std::cout << "[ERROR] Cannot add motion.\n";
		return;
	}
	CALL(clink_rpc_motion_command_queue_flush(cbox_id)); // queue 에 삽입되어 있는 모션들을 일괄적으로 전송한다(모션 시작).
	CALL(clink_rpc_motion_command_destroy(cbox_id, cmd_id)); // 생성한 모션을 삭제한다.

	// 모션 완료 이벤트(CLINK_EVENT_SUBGRP_MOTION_COMMAND_SETTLED_DOWN) 발생을 최대 1000s 대기
	CALL(clink_rpc_system_wait_event_group_subgroup_sender_id(
		cbox_id,
		CLINK_EVENT_GRP_MOTION_COMMAND,
		CLINK_EVENT_SUBGRP_MOTION_COMMAND_SETTLED_DOWN,
		cmd_id,
		1000000,
		1,
		&valid_event));

	// 1000s 내 이벤트 미발생
	if (0 == valid_event)
	{
		std::cout << "[ERROR] Event not received within timeout (1000s).\n";
	}
}

void display_pose(int32_t mode)
{
	clink_float_t x, y, z;
	clink_float_t rx, ry, rz;
	clink_float_t angles[NUM_OF_JOINTS];

	thread_run = 1;
	while (thread_run)
	{
		if (mode & PRINT_TOOL)
		{
			clink_rpc_robot_tcp_pose_command_get(cbox_id, robot_id, &x, &y, &z, &rx, &ry, &rz);
			printf("[Tool ]  X: %7.2f,  Y: %7.2f,  Z: %7.2f  RX: %7.2f, RY: %7.2f, RZ: %7.2f\n", x, y, z, rx, ry, rz);
		}
		if (mode & PRINT_JOINT)
		{
			for (uint32_t i = 0; i < NUM_OF_JOINTS; i++)
			{
				clink_rpc_robot_joint_angle_actual_get(cbox_id, robot_id, i, &angles[i]);
			}
			printf("[Joint] J1: %7.2f, J2: %7.2f, J3: %7.2f, "
				"J4: %7.2f, J5: %7.2f, J6: %7.2f\n",
				angles[0], angles[1], angles[2],
				angles[3], angles[4], angles[5]);
		}

		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
}