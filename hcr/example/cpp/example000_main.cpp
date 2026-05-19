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
#include <clink_api_rpc_system.h>
#include <iostream>
#include <thread>

//---------------------------------------------- User 환경에 맞게 설정 ----------------------------------------------

// 실제 로봇과 연결할 경우 해당 define 문 주석 처리
#define SIMULATION_MODE

#ifdef SIMULATION_MODE
constexpr CLINK_CBOX_MODEL CBOX_MODEL_NAME = CLINK_CBOX_MODEL_3GEN_SIM;
#else
constexpr CLINK_CBOX_MODEL CBOX_MODEL_NAME = CLINK_CBOX_MODEL_3GEN;
#endif
constexpr CLINK_ROBOT_MODEL ROBOT_MODEL_NAME = CLINK_ROBOT_MODEL_HCR14;

#define TARGET_IPADDR		"192.168.100.123"				// 제어SW 및 RPC Server가 실행되는 Target PC의 IP Adress
#define CONFIG_FILE_NAME	"../../config/config_rpc.ini"	// ${base_dir}/config/config_rpc.ini 의 경로

//-----------------------------------------------------------------------------------------------------------------

constexpr uint32_t NUM_OF_JOINTS = 6;
uint32_t cbox_id = 0;
uint32_t robot_id = 0;


extern void clink_api_test();

void move_homing()
{
	clink_rpc_system_event_queue_clear(cbox_id);

	// Home 자세로 이동한다.
	// Home 자세는 로봇의 가장 기본이 되는 자세로 각도가 정해져 있다.
	clink_float_t angles[NUM_OF_JOINTS] = { 0.0, -90.0, -90.0, -90.0, 90.0, 0.0 };

	// 간단한 함수로 로봇 joint move를 실행해 볼 수 있는 API
	CLINK_API_RESULT err_ret_val = clink_rpc_just_robot_joint_move(
		cbox_id,
		robot_id,
		0.5,
		NUM_OF_JOINTS,
		angles);

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

}

void clink_event_callback_func(
	uint32_t event_grp,
	uint32_t event_subgrp,
	uint32_t event_sender_id,
	int32_t reserve01,
	int32_t reserve02,
	int32_t reserve03,
	int32_t reserve04,
	int32_t reserve05)
{
	// 이벤트 수신시, 실행되는 callback 함수
	// 이벤트 종류에 따라 필요한 작업을 수행할 수 있다.
	// 이벤트에 대한 사항은 clink_api help 문서의 Modules/40_API/00CONSTANT 내
	// CLINK_EVENT_GRP 및 CLINK_EVENT_SUBGRP 항목을 참조한다.
	// error 관련 이벤트는 clink_error_event_desc.xml 문서도 참조 가능하다.

	std::cout
		<< "[RPC CLIENT REGISTERD CALLBACK FUNC] evt grp: "
		<< event_grp
		<< ", subgrp: "
		<< event_subgrp
		<< ", sender ID: "
		<< event_sender_id
		<< std::endl;
}

//-- for every TEST binary
// 모든 예제에서 공통적으로 들어가는 루틴
// 해당 함수에서는 순서대로
// RPC Server 와의 연결
// System 초기화, 제어권 획득, 로봇 생성, 이벤트 핸들러 등록
// 순으로 함수가 호출된다.
bool setup()
{
	// 모든 API는 기본적으로 error code를 반환한다.
	// error code 에 대한 사항은
	// clink_api help 문서의 Modules/40_API/CLINK_API_RESULT 항목 또는
	// clink_error_api_return_desc.xml 문서를 참조한다.
	CLINK_API_RESULT err_ret_val = CLINK_API_RESULT_OK;

	bool ret = true;
	// RPC Server 연결
	err_ret_val = clink_rpc_system_cbox_connect(CONFIG_FILE_NAME, TARGET_IPADDR, &cbox_id);
	std::this_thread::sleep_for(std::chrono::seconds(1));

	// 이벤트 핸들러 등록
	// 제어SW에서 발생하는 각종 정보는 이벤트를 통해 API 측으로 고지된다.
	// 사용자는 이벤트 핸들러 등록을 통해서 이벤트에 대한 정보를 받고 처리할 수 있다.
	// 초기화 관련 이벤트 수신을 위해 시스템 초기화 전에 등록한다.
	if (ret)
	{
		err_ret_val = clink_rpc_system_event_callback_add(cbox_id, clink_event_callback_func);
		std::this_thread::sleep_for(std::chrono::seconds(1));
		if (CLINK_API_RESULT_OK != err_ret_val)
		{
			ret = false;
			std::cout << "이벤트 callback 등록 실패, code: " << err_ret_val << std::endl;
		}
	}

	if (CLINK_API_RESULT_OK != err_ret_val)
	{
		ret = false;
		std::cout << "RPC Server 연결 실패, code: " << err_ret_val << std::endl;
	}

	// 제어SW 초기화 API 호출
	if (ret)
	{
		err_ret_val = clink_rpc_gen_system_create(cbox_id, "", CBOX_MODEL_NAME);
		std::this_thread::sleep_for(std::chrono::seconds(3));

		// CLINK_API_RESULT_WARNING_SYSTEM_ALREADY_CREATED WARNING은 RPC Server 에서
		// 이미 호출되었을 때 발생
		if (CLINK_API_RESULT_OK != err_ret_val &&
			CLINK_API_RESULT_WARNING_SYSTEM_ALREADY_CREATED != err_ret_val)
		{
			ret = false;
			std::cout << "제어SW 초기화 실패, code: " << err_ret_val << std::endl;
		}
	}

	// 제어권 획득 API 호출
	if (ret)
	{
		err_ret_val = clink_rpc_system_control_take(cbox_id);
		if (CLINK_API_RESULT_OK != err_ret_val)
		{
			ret = false;
			std::cout << "제어권 획득 실패, code: " << err_ret_val << std::endl;
		}
	}

	// 로봇 생성 API 호출
	if (ret)
	{
		err_ret_val = clink_rpc_robot_create(cbox_id, ROBOT_MODEL_NAME, "", 0U, &robot_id);

		// control box의 EtherCAT 연결 상태 확인
		CLINK_ECAT_CONN_STATE ecat_stat = CLINK_ECAT_CONN_STATE_DISCONNECTED;
		clink_rpc_cbox_ecat_connection_state_get(cbox_id, &ecat_stat);

		// RPC Server 에서 이미 control box의 EtherCAT 연결이 완료 되어 있을 수 있음
		// 통신 미연결 시에만 연결 완료 이벤트 대기
		if (CLINK_ECAT_CONN_STATE_CONNECTED != ecat_stat)
		{
			char_t valid_event = -1;
			clink_rpc_system_wait_event_group_subgroup(
				cbox_id,											// control box ID
				CLINK_EVENT_GRP_NOTIFICATION,						// event group
				CLINK_EVENT_SUBGRP_NOTIFICATION_ECAT_CONNECTED,		// event subgroup
				1000000,											// 대기 시간 [ms]
				1,													// 해당 이벤트를 내부 이벤트 큐에서 삭제한다.
				&valid_event);										// 정해진 시간내에 이벤트 발생 시 1, 발생안했을 시 0 반환
		}

		if (CLINK_API_RESULT_OK != err_ret_val)
		{
			ret = false;
			std::cout << "로봇 생성 실패, code: " << err_ret_val << std::endl;
		}
	}


	// 로봇 모션에 있어 속도/가속도 위반 에러를 감지하여 자동 속도 조절해주는 기능 ON
	if (ret)
	{
		clink_rpc_robot_motion_auto_adjust_swith_set(cbox_id, robot_id, CLINK_SWITCH_ON);
	}

	return ret;
}

void tear_down()
{
	// 등록했던 이벤트 핸들러 제거
	clink_rpc_system_event_callback_remove(cbox_id, clink_event_callback_func);

	// 안전 상 로봇 stop 및 servo off 실행 후 종료
	clink_rpc_robot_stop(cbox_id, robot_id, 0.5);
	std::this_thread::sleep_for(std::chrono::seconds(1));
	clink_rpc_robot_servo_switch_set(cbox_id, robot_id, CLINK_SWITCH_OFF);
}

int main(int argc, char** argv)
{
	if (setup())
	{
		clink_api_test();
		tear_down();
	}
}