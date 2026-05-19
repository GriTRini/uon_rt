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
#include <cmath>

CLINK_API_RESULT err_val;
#define CALL(f)																		                \
	err_val = f;											                                        \
	if(CLINK_API_RESULT_OK != err_val) { std::cout << "error code: " << err_val << std::endl; };	

extern uint32_t cbox_id;
extern uint32_t robot_id;
extern void move_homing();

void clink_api_test()
{
	// 모션 명령을 실행하는 즉시 모션이 blending 되어가면서 진행한다.
	// (앞의 모션이 진행중이더라도 앞선 모션과 섞이면서 바로 진행됨)
	// external control은 함수 호출을 하는 시점과
	// 입력 속도 값에 따라 모션이 달라진다.
	// 두 factor를 잘 조절하여 사용
	// 예제는 spiral 형태의 모션을 그린다.

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

	// external control 시작
	CALL(clink_rpc_robot_ext_move_switch_set(cbox_id, robot_id, CLINK_SWITCH_ON));
	
	const uint32_t NUM_OF_MOTION = 200;								// 삽입할 모션의 총 갯수
	const clink_float_t D_THETA = 360.0 / NUM_OF_MOTION * 10.0;		// 총 10바퀴
	const clink_float_t D_RADIUS_LENGTH = 150.0 / NUM_OF_MOTION;	// 최종 150mm 까지 반지름 증가
	const clink_float_t D_Z_LENGTH = 50.0 / NUM_OF_MOTION;			// z축은 최종 50mm 까지 증가
	const clink_float_t PI = 3.14159265358979323846;

	clink_float_t theta = 0.0;
	clink_float_t radius = 0.0;

	for (uint32_t i = 0; i < NUM_OF_MOTION; i++)
	{
		clink_float_t x = cmd_pos_x + (D_RADIUS_LENGTH * i) * std::cos((D_THETA * i) * PI / 180.0);
		clink_float_t y = cmd_pos_y + (D_RADIUS_LENGTH * i) * std::sin((D_THETA * i) * PI / 180.0);
		clink_float_t z = cmd_pos_z + (D_Z_LENGTH * i);
		CALL(clink_rpc_robot_ext_move_pos_ort_euler_add(
			cbox_id, 
			robot_id, 
			CLINK_POSE_TYPE_FLANGE, 
			x, 
			y, 
			z, 
			cmd_ort_x, 
			cmd_ort_y, 
			cmd_ort_z, 
			60.0));
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}

	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	CALL(clink_rpc_robot_stop(cbox_id, robot_id, 0.5));
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	// external control 종료
	CALL(clink_rpc_robot_ext_move_switch_set(cbox_id, robot_id, CLINK_SWITCH_OFF));

	move_homing();
    CALL(clink_rpc_robot_servo_switch_set(cbox_id, robot_id, CLINK_SWITCH_OFF));
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}
