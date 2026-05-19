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
#include <cstring>

CLINK_API_RESULT err_val;
#define CALL(f)																		                \
	err_val = f;											                                        \
	if(CLINK_API_RESULT_OK != err_val) { std::cout << "error code: " << err_val << std::endl; };

//---------------------------------------------- User 환경에 맞게 설정 ----------------------------------------------

// 실제 로봇과 연결할 경우 해당 define 문 주석 처리
//#define SIMULATION_MODE

#ifdef SIMULATION_MODE
constexpr CLINK_CBOX_MODEL CBOX_MODEL_NAME = CLINK_CBOX_MODEL_EMPTY_SIM;
#else
constexpr CLINK_CBOX_MODEL CBOX_MODEL_NAME = CLINK_CBOX_MODEL_EMPTY;
#endif
constexpr CLINK_ROBOT_MODEL ROBOT_MODEL_NAME = CLINK_ROBOT_MODEL_USERROBOT01;

#define TARGET_IPADDR		"192.168.100.123"				// 제어SW 및 RPC Server가 실행되는 Target PC의 IP Adress
#define CONFIG_FILE_NAME	"../../config/config_rpc.ini"	// ${base_dir}/config/config_rpc.ini 의 경로

//-----------------------------------------------------------------------------------------------------------------

enum MOTOR_ID {
	MOTOR_RIGHT = 0,
	MOTOR_LEFT = 1,
	MOTOR_CNT
};
uint32_t cbox_id = 0;
uint32_t robot_id = 0;

std::thread err_code_thread;
uint8_t thread_run;

void stop_thread(std::thread& t, uint8_t& is_running) {
	if (is_running) {
		is_running = 0;
		if (t.joinable()) t.join();
	}
}

void pretty_print_hex(const uint8_t* data, size_t sz, const char* title)
{
	char out_buf[200];
	size_t pos = 0;
	out_buf[0] = '\0';

	if (sz < 16)
		return;

	for (size_t row = 0; row < (sz + 15) / 16; ++row) {
		size_t row_bytes = (sz - row * 16 >= 16) ? 16 : (sz - row * 16);
		for (size_t col = 0; col < row_bytes; ++col) {
			size_t idx = row * 16 + col;
			size_t remain = (pos < sizeof(out_buf)) ? (sizeof(out_buf) - pos) : 0;
			if (remain <= 1) break;

			pos += std::snprintf(out_buf + pos, remain, "%02X ", data[idx]);

			if ((col + 1) % 8 == 0 && (col + 1) != 16 && (col + 1) != row_bytes) {
				remain = (pos < sizeof(out_buf)) ? (sizeof(out_buf) - pos) : 0;
				if (remain <= 1) break;
				pos += std::snprintf(out_buf + pos, remain, " ");
			}
		}
		size_t remain = (pos < sizeof(out_buf)) ? (sizeof(out_buf) - pos) : 0;
		if (remain > 1) pos += std::snprintf(out_buf + pos, remain, "\n");
	}

	if (pos < sizeof(out_buf))
		out_buf[pos] = '\0';
	else
		out_buf[sizeof(out_buf) - 1] = '\0';

	printf("======================== [%s] ========================\n%s================================================\n", title, out_buf);
}

void clink_api_low_data_test()
{
	uint8_t rx_buf[32] = { 0 };
	uint8_t plc_tx_bkp[32] = { 0 };
	uint8_t io_tx_bkp[32] = { 0 };
	printf("Read rx data:\n");
	clink_rpc_cbox_ecat_low_data_io_rx_data_get(cbox_id, 0, 32, rx_buf);
	pretty_print_hex(rx_buf, 32, "PLC");
	clink_rpc_cbox_ecat_io_link_rx_data_get(cbox_id, 0, 32, rx_buf);
	pretty_print_hex(rx_buf, 32, "IO-LINK");
	printf("\n");

	printf("Backup tx data:\n");
	clink_rpc_cbox_ecat_low_data_io_tx_data_get(cbox_id, 0, 32, plc_tx_bkp);
	clink_rpc_cbox_ecat_io_link_tx_data_get(cbox_id, 0, 32, io_tx_bkp);
	pretty_print_hex(plc_tx_bkp, 32, "PLC");
	pretty_print_hex(io_tx_bkp, 32, "IO");
	printf("\n");

	printf("Write 0x11101 to PLC...\n");
	uint32_t value = 0x11101;
	uint8_t tx_buf[32] = { 0 };
	tx_buf[0] = (value >> 0) & 0xFF;   // 0x01
	tx_buf[1] = (value >> 8) & 0xFF;   // 0x11
	tx_buf[2] = (value >> 16) & 0xFF;  // 0x01
	tx_buf[3] = (value >> 24) & 0xFF;  // 0x00

	clink_rpc_cbox_ecat_low_data_io_tx_data_set(cbox_id, 0, 32, tx_buf);
	std::this_thread::sleep_for(std::chrono::seconds(1));

	printf("Write 0x10AABBCC to IO-LINK...\n");
	value = 0x10AABBCC;
	tx_buf[0] = (value >> 0) & 0xFF;   // 0xCC
	tx_buf[1] = (value >> 8) & 0xFF;   // 0xBB
	tx_buf[2] = (value >> 16) & 0xFF;  // 0xAA
	tx_buf[3] = (value >> 24) & 0xFF;  // 0x10
	clink_rpc_cbox_ecat_io_link_tx_data_set(cbox_id, 0, 32, tx_buf);
	std::this_thread::sleep_for(std::chrono::seconds(1));

	printf("Read tx data:\n");
	clink_rpc_cbox_ecat_low_data_io_tx_data_get(cbox_id, 0, 32, tx_buf);
	pretty_print_hex(tx_buf, 32, "PLC");
	clink_rpc_cbox_ecat_io_link_tx_data_get(cbox_id, 0, 32, tx_buf);
	pretty_print_hex(tx_buf, 32, "IO");
	printf("\n");

	printf("Rollback tx data...\n");
	clink_rpc_cbox_ecat_low_data_io_tx_data_set(cbox_id, 0, 32, plc_tx_bkp);
	clink_rpc_cbox_ecat_io_link_tx_data_set(cbox_id, 0, 32, io_tx_bkp);
	std::this_thread::sleep_for(std::chrono::seconds(1));

	printf("Read tx data:\n");
	clink_rpc_cbox_ecat_low_data_io_tx_data_get(cbox_id, 0, 32, tx_buf);
	pretty_print_hex(tx_buf, 32, "PLC");
	clink_rpc_cbox_ecat_io_link_tx_data_get(cbox_id, 0, 32, tx_buf);
	pretty_print_hex(tx_buf, 32, "IO");
	printf("\n");
}

void log_motor_error_code(int robot_id, int motor_id)
{
	static uint16_t prev_error_code[MOTOR_CNT][4] = {0};
	uint16_t error_code[MOTOR_CNT][4] = {0};

	if (CLINK_API_RESULT_OK == clink_rpc_robot_joint_error_code_get(cbox_id, robot_id, motor_id,
		&error_code[motor_id][0], &error_code[motor_id][1], &error_code[motor_id][2], &error_code[motor_id][3]))
	{
		if (error_code[motor_id][3] != prev_error_code[motor_id][3] ||
			error_code[motor_id][2] != prev_error_code[motor_id][2] ||
			error_code[motor_id][1] != prev_error_code[motor_id][1] ||
			error_code[motor_id][0] != prev_error_code[motor_id][0])
		{
			printf("[INFO] Motor[%d] | ErrorCode: [0x%X] | Motion error: [0x%X] | error detail1: [0x%X] | error detail2: [0x%X]\n",
				motor_id, error_code[motor_id][0], error_code[motor_id][1], error_code[motor_id][2], error_code[motor_id][3]);
			memcpy(prev_error_code[motor_id], error_code[motor_id], sizeof(prev_error_code[motor_id]));
		}
	}
	else
	{
		printf("[WARN] Failed to get error code for motor %d\n", motor_id);
	}
}


void err_code_logging_thread()
{
	thread_run = 1;
	while (thread_run)
	{
		log_motor_error_code(robot_id, MOTOR_LEFT);
		log_motor_error_code(robot_id, MOTOR_RIGHT);
	}
	printf("err_code_logging_thread : terminate\n");
}

void clink_api_velocity_test()
{
	// error code logging 시작
	// servo on 시, 발생하는 error code:[0x8100]은 No Error로, 자동 Reset 된다.
	err_code_thread = std::thread(err_code_logging_thread);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	// 이동 로봇향 velocity 제어에 대한 설명이다.
	// Velocity mode로 전환 후, accel과 speed 값을 수정하여 사용한다.
	CALL(clink_rpc_robot_servo_switch_set(cbox_id, robot_id, CLINK_SWITCH_ON));
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	clink_float_t speed[5] = { 5, 10.0, 0.0, -5.0, 0.0 };

	// velocity mode로 전환
	CALL(clink_rpc_robot_control_mode_set(cbox_id, robot_id, CLINK_DRIVE_CONTROL_MODE_VELOCITY_PROFILE)); // PV MODE
	// CALL(clink_rpc_robot_control_mode_set(cbox_id, robot_id, CLINK_DRIVE_CONTROL_MODE_VELOCITY)); // CSV MODE

	// Accel = 150.0 RPM/s^2로 설정
	CALL(clink_rpc_robot_joint_target_acceleration_rpm_set(cbox_id, robot_id, MOTOR_LEFT, 150.0));
	CALL(clink_rpc_robot_joint_target_acceleration_rpm_set(cbox_id, robot_id, MOTOR_RIGHT, 150.0));
	std::this_thread::sleep_for(std::chrono::seconds(1));


	for (uint32_t iter = 0; iter < 5; iter++)
	{
		std::cout << "test loop: " << iter << std::endl;

		for (uint32_t i = 0; i < 5; i++)
		{
			// 3초 단위로 Speed = speed[i] rpm/s 로 설정
			CALL(clink_rpc_robot_joint_target_velocity_rpm_set(cbox_id, robot_id, MOTOR_LEFT, speed[i]));
			CALL(clink_rpc_robot_joint_target_velocity_rpm_set(cbox_id, robot_id, MOTOR_RIGHT, speed[i]));
			std::this_thread::sleep_for(std::chrono::milliseconds(3000));
		}
	}
	CALL(clink_rpc_robot_servo_switch_set(cbox_id, robot_id, CLINK_SWITCH_OFF));
	stop_thread(err_code_thread, thread_run);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void clink_api_test()
{
	clink_api_low_data_test();
	clink_api_velocity_test();
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
		<< "[REGISTERD CALLBACK FUNC] evt grp: "
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

	if (CLINK_API_RESULT_OK != err_ret_val)
	{
		ret = false;
		std::cout << "RPC Server 연결 실패, code: " << err_ret_val << std::endl;
	}

	if (ret)
	{
		// 이벤트 핸들러 등록
		// 제어SW에서 발생하는 각종 정보는 이벤트를 통해 API 측으로 고지된다.
		// 사용자는 이벤트 핸들러 등록을 통해서 이벤트에 대한 정보를 받고 처리할 수 있다.
		// 초기화 관련 이벤트 수신을 위해 시스템 초기화 전에 등록한다.
		err_ret_val = clink_rpc_system_event_callback_add(cbox_id, clink_event_callback_func);
		std::this_thread::sleep_for(std::chrono::seconds(1));
		if (CLINK_API_RESULT_OK != err_ret_val)
		{
			ret = false;
			std::cout << "이벤트 callback 등록 실패, code: " << err_ret_val << std::endl;
		}
	}

	// 제어SW 초기화 API 호출
	if (ret)
	{
		err_ret_val = clink_rpc_gen_system_create(cbox_id, "", CBOX_MODEL_NAME);
		std::this_thread::sleep_for(std::chrono::seconds(1));

		// CLINK_API_RESULT_WARNING_SYSTEM_ALREADY_CREATED WARNING은 RPC Server 에서
		// 이미 호출되었을 때 발생
		if (CLINK_API_RESULT_OK != err_ret_val &&
			CLINK_API_RESULT_WARNING_SYSTEM_ALREADY_CREATED != err_ret_val)
		{
			ret = false;
			std::cout << "제어SW 초기화 실패, code: " << err_ret_val << std::endl;
		}
	}

	// 해당 함수는 지정된 이벤트가 도착할 때까지 지정된 시간만큼 blocking
	char_t valid_event = -1;
	clink_rpc_system_wait_event_group_subgroup(
		cbox_id,												// control box ID
		CLINK_EVENT_GRP_NOTIFICATION,							// event group
		CLINK_EVENT_SUBGRP_NOTIFICATION_SYSTEM_INITIALIZED,		// event subgroup
		10000,													// 대기 시간 [ms]
		1,														// 해당 이벤트를 내부 이벤트 큐에서 삭제한다.
		&valid_event);											// 정해진 시간내에 이벤트 발생 시 1, 발생안했을 시 0 반환

	// 제어권 획득 API 호출
	// clink_gen_system_create 수행시 clink_system_control_take을 진행하여 제어권을 획득한 상태
	if (ret)
	{
		err_ret_val = clink_rpc_system_control_take(cbox_id);
		if (CLINK_API_RESULT_OK != err_ret_val && CLINK_API_RESULT_WARNING_YOU_DONT_HAVE_THE_CONTROL != err_ret_val)
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

		// 이미 control box의 EtherCAT 연결이 완료 되어 있을 수 있음
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

	return ret;
}

void tear_down()
{
	// 등록했던 이벤트 핸들러 제거
	clink_rpc_system_event_callback_remove(cbox_id, clink_event_callback_func);

	CALL(clink_rpc_robot_joint_target_velocity_set(cbox_id, robot_id, 0, 0.0));
	CALL(clink_rpc_robot_joint_target_velocity_set(cbox_id, robot_id, 1, 0.0));
	std::this_thread::sleep_for(std::chrono::seconds(2));
	clink_rpc_robot_servo_switch_set(cbox_id, robot_id, CLINK_SWITCH_OFF);
	CALL(clink_rpc_system_destroy(cbox_id));
	std::this_thread::sleep_for(std::chrono::seconds(1));
}

int main(int argc, char** argv)
{
	if (setup())
	{
		clink_api_test();
		tear_down();
	}
}