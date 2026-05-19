//------------------------------------------------------------------------------
//
// ControLINK-API
// Copyright ⓒ Hanwha Robotics. All rights reserved.
// (https://www.hanwharobotics.co.kr/)
//
// Seokhwan Kim    (seokhwan at hanwha dot com)
// Kazuma   Fukuda (k.fukuda at hanwha dot com)
// Kiho     Cho    (kiho0520 at hanwha dot com)
// Incheol  Jeong  (ic.jeong at hanwha dot com)
// 
//------------------------------------------------------------------------------

/** \addtogroup 40_API 40_API
*  @{
*/

#ifndef __CLINK_API_CALL_RPC_H__
#define __CLINK_API_CALL_RPC_H__

#include "clink_def.h"
#include "clink_constant.h"

#ifdef __cplusplus
extern "C"
{
#endif


//------------------------------------------------------------------
// 100_SYSTEM
//------------------------------------------------------------------
/** \addtogroup 100_SYSTEM 100_SYSTEM
*  @{
*/

/**
@brief      시스템 초기화를 실행한다. 최초 실행 시, 반드시 호출해야 한다.
            해당 API는 HCR 3GEN CBOX를 생성한다.
@param[in]  p_config_file_path
            설정 파일의 full path
@remark     이 함수를 호출하지 않고, 다른 함수를 호출하면
            예상할 수 없는 동작 (undefined behavior) 가 발생할 수 있다.
@return     @ref clink_system_create_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_system_create(
        uint32_t cbox_id,
        const char_t* p_config_file_path);

/**
@brief      시스템 초기화를 실행한다. 최초 실행 시, 반드시 호출해야 한다.
            TCP/IP 를 통해 연결할 경우 해당 system 함수를 호출한다.
@param[in]  p_config_file_path
            설정 파일의 full path
@param[in]  p_cbox_ip
            연결할 control box 의 IP
@param[in]  cbox_model
            cbox 모델
@remark     이 함수를 호출하지 않고, 다른 함수를 호출하면
            예상할 수 없는 동작 (undefined behavior) 가 발생할 수 있다.
@return     @ref clink_gen_system_connect_and_create_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_gen_system_connect_and_create(
        uint32_t cbox_id,
        const char_t* p_config_file_path,
        const char_t* p_cbox_ip,
        CLINK_CBOX_MODEL cbox_model);

/**
@brief      시스템 초기화를 실행한다. 최초 실행 시, 반드시 호출해야 한다.
@param[in]  p_config_file_path
            설정 파일의 full path
@param[in]  cbox_model
            cbox 모델
@remark     이 함수를 호출하지 않고, 다른 함수를 호출하면
            예상할 수 없는 동작 (undefined behavior) 가 발생할 수 있다.
@return     @ref clink_gen_system_create_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_gen_system_create(
        uint32_t cbox_id,
        const char_t* p_config_file_path,
        CLINK_CBOX_MODEL cbox_model);

/**
@brief      시스템 초기화를 실행한다. 최초 실행 시, 반드시 호출해야 한다.
@param[in]  p_config_file_path
            설정 파일의 full path
@param[in]  cbox_model
            cbox 모델
@param[in]  num_of_robots
			system 에 연결될 총 로봇의 수를 입력한다. (robot create 호출 수)
@remark     이 함수를 호출하지 않고, 다른 함수를 호출하면
            예상할 수 없는 동작 (undefined behavior) 가 발생할 수 있다.
@return     @ref clink_gen_multi_robot_system_create_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_gen_multi_robot_system_create(
        uint32_t cbox_id,
	    const char_t* p_config_file_path,
	    CLINK_CBOX_MODEL cbox_model,
        uint32_t num_of_robots);

/**
@brief      시스템 메모리 초기화를 실행한다.
@remark     다른 process에서 초기화 후 또 다른 process 에서 get 계열 API 호출 시 필요
@return     @ref clink_system_memory_create_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_system_memory_create(
        uint32_t cbox_id);

/**
@brief      시스템 종료 루틴을 실행한다. 시스템 종료 시, 반드시 호출해야 한다.
@remark     이 함수를 호출하지 않고, 시스템을 종료할 경우 \n
            메모리 누수 (memory leak) 등의 문제가 발생할 수 있다.
@return     @ref clink_system_destroy_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_system_destroy(
        uint32_t cbox_id);

/**
@brief      실시간 시스템의 제어SW를 exit 한다.
@return     @ref clink_system_control_sw_exit_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_system_control_sw_exit(
        uint32_t cbox_id);

/**
@brief      API 를 통한 제어하는 한 순간에 하나의 프로세스만 가능하다.
            이 함수를 호출하여 제어권을 얻은 뒤 다른 API 의 사용이 가능하다. \n
            어떤 프로세스든지 API 사용전에 아래 함수를 호출하여 \n
            제어권을 획득한 후 다른 API 를 사용할 수 있다.
@return     @ref clink_system_control_take_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_system_control_take(
        uint32_t cbox_id);

/**
@brief      API 를 통한 제어하는 한 순간에 하나의 프로세스만 가능하다.
            이 함수를 호출하여 현재 프로세스는 제어권을 포기한다. \n
            그리고, 이 순간부터 다른 프로세스는 clink_system_control_take() 를 \n
            통해 제어권을 획득할 수 있다.
@return     @ref clink_system_control_release_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_system_control_release(
        uint32_t cbox_id);

/**
@brief      현재 제어권을 가지고 있는지 받아온다.
@param[out] p_control
            제어권을 가지고 있을 시, CLINK_SWITCH_ON
@return     @ref clink_system_controllable_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_system_controllable_get(
        uint32_t cbox_id,
        CLINK_SWITCH* p_control);

/**
@brief      시스템에 연결된 로봇, Control Box 의 모든 상태를 초기화 한다. \n
            Error 가 있는 경우, Error Clear 를 시도한다. \n
@remark     내부적으로 시스템을 초기화한 후, \n
            clink_rpc_cbox_reset() 을 호출하여 \n
            Control Box 및 로봇을 초기화한다.
@return     @ref clink_system_reset_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_system_reset(
        uint32_t cbox_id);

/**
@brief      API SW 버전을 반환한다.
@return     API SW 버전.
*/
CLINK_EXPORT const char_t*
    clink_rpc_system_api_version_get(
        uint32_t cbox_id);

/**
@brief      제어 SW 버전을 반환한다.
@return     제어 SW 버전.
*/
CLINK_EXPORT const char_t*
    clink_rpc_system_control_sw_version_get(
        uint32_t cbox_id);

/**
@brief      에러코드에 대한 설명을 리턴한다.
@param[in]  error_code
            에러코드
@return     에러코드에 대한 설명의 문자열을 반환한다. \n
            설명을 찾지 못하는 경우 "NOT FOUND" 문자열을 반환한다.
*/
CLINK_EXPORT const char_t*
    clink_rpc_system_api_result_description_get(
        uint32_t cbox_id,
        CLINK_API_RESULT error_code);

/**
@brief      CLINK_API_RESULT 코드에 대해 개발자가 조치해야할 내용을 설명한다.
@param[in]  error_code
            에러코드
@return     조치 방법 \n
            설명을 찾지 못하는 경우 "NOT FOUND" 문자열을 반환한다.
*/
CLINK_EXPORT const char_t*
    clink_rpc_system_api_result_how_to_handle_get(
        uint32_t cbox_id,
        CLINK_API_RESULT error_code);

/**
@brief      API 의 모든 에러코드를 XML 형식의 파일로 저장한다.
@param[in]  p_file_path
            저장 XML 파일의 경로
@return     @ref clink_system_api_result_description_all_xml_save_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_system_api_result_description_all_xml_save(
        uint32_t cbox_id,
        const char_t* p_file_path);

/**
@brief      에러와 관련된 EVENT 정보를 XML 형식의 파일로 저장한다.
@param[in]  p_file_path
            저장 XML 파일의 경로
@return     @ref clink_system_error_event_description_all_xml_save_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_system_error_event_description_all_xml_save(
        uint32_t cbox_id,
        const char_t* p_file_path);

/**
@brief      모든 이벤트 발생 시, 호출되는 callback 함수를 추가한다.
@param[in]  p_event_callback_funcptr
            callback 함수의 함수 포인터
@remark     동일 event group 에 대해 복수개의 callback 을 등록할 수 있다.
@return     @ref clink_system_event_callback_add_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_system_event_callback_add(
        uint32_t cbox_id,
        clink_event_callback_t p_event_callback_funcptr);

/**
@brief      모든 이벤트 발생 시, 호출되는 callback 함수를 삭제한다.
@param[in]  p_event_callback_funcptr
            이미 등록된 callback 함수의 함수 포인터
@remark     복수개의 callback 함수가 해당 group 에 등록이 되어 있는 경우 \n
            해당 함수만이 삭제가 된다. (즉, 다른 callback 함수는 삭제하지 않음)
@return     @ref clink_system_event_callback_remove_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_system_event_callback_remove(
        uint32_t cbox_id,
        clink_event_callback_t p_event_callback_funcptr);

/**
@brief      이벤트 발생 시, 호출되는 callback 함수를 추가한다.
@param[in]  p_event_callback_funcptr
            callback 함수의 함수 포인터
@param[in]  event_group
            수신할 event 의 그룹명 \n
            CLINK_EVENT_GRP 참조.
@remark     동일 event group 에 대해 복수개의 callback 을 등록할 수 있다.
@return     @ref clink_system_event_group_callback_add_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_system_event_group_callback_add(
        uint32_t cbox_id,
        clink_event_callback_t p_event_callback_funcptr,
        uint32_t event_group);

/**
@brief      이벤트 발생 시, 호출되는 callback 함수를 삭제한다.
@param[in]  p_event_callback_funcptr
            이미 등록된 callback 함수의 함수 포인터
@param[in]  event_group
            수신할 event 의 그룹명 \n
            CLINK_EVENT_GRP 참조.
@remark     복수개의 callback 함수가 해당 group 에 등록이 되어 있는 경우 \n
            해당 함수만이 삭제가 된다. (즉, 다른 callback 함수는 삭제하지 않음)
@return     @ref clink_system_event_group_callback_remove_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_system_event_group_callback_remove(
        uint32_t cbox_id,
        clink_event_callback_t p_event_callback_funcptr,
        uint32_t event_group);

/**
@brief      이벤트 발생 시, 호출되는 callback 함수를 추가한다.
@param[in]  p_event_callback_funcptr
            callback 함수의 함수 포인터
@param[in]  event_group
            수신할 event 의 그룹명 \n
            CLINK_EVENT_GRP 참조.
@param[in]  event_subgroup
            event_group 에 속한 subgroup 명
@remark     동일 event group 및 subgroup 에 대해 복수개의 callback 을 등록할 수 있다.
@return     @ref clink_system_event_group_subgroup_callback_add_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_system_event_group_subgroup_callback_add(
        uint32_t cbox_id,
        clink_event_callback_t p_event_callback_funcptr,
        uint32_t event_group,
        uint32_t event_subgroup);

/**
@brief      이벤트 발생 시, 호출되는 callback 함수를 삭제한다.
@param[in]  p_event_callback_funcptr
            이미 등록된 callback 함수의 함수 포인터
@param[in]  event_group
            수신할 event 의 그룹명 \n
            CLINK_EVENT_GRP 참조.
@param[in]  event_subgroup
            event_group 에 속한 subgroup 명
@remark     복수개의 callback 함수가 해당 group 및 subgroup 에 등록이 되어 있는 경우 \n
            해당 함수만이 삭제가 된다. (즉, 다른 callback 함수는 삭제하지 않음)
@return     @ref clink_system_event_group_subgroup_callback_remove_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_system_event_group_subgroup_callback_remove(
        uint32_t cbox_id,
        clink_event_callback_t p_event_callback_funcptr,
        uint32_t event_group,
        uint32_t event_subgroup);

/**
@brief      지정된 group 명의 이벤트가 발생할 때까지 기다린다. \n
            (blocking 상태 지속)
@param[in]  event_group
            수신할 event 의 그룹명 \n
            CLINK_EVENT_GRP 참조.
@param[in]  timeout_milliseconds
            event 를 대기하는 최대 시간 (단위 : milliseconds)
@param[in]  is_delete_in_queue
            <STRONG>0 인 경우:</STRONG> 이벤트 발생 후, 해당 이벤트를 내부 queue 에서 \n
            삭제하지 않는다. 결과적으로, \n
            clink_rpc_system_event_group_callback_add(), \n
            clink_rpc_system_event_group_subgroup_callback_add() 등을 통해 \n
            등록한 callback 함수는 호출된다. \n
            <STRONG>0 이 아닌 경우:</STRONG> 이벤트 발생 후, 해당 이벤트를 내부 queue 에서 \n
            삭제한다. 결과적으로, \n
            clink_rpc_system_event_group_callback_add(), \n
            clink_rpc_system_event_group_subgroup_callback_add() 등을 통해 \n
            등록한 callback 함수는 호출되지 않는다. \n
@param[out]  p_result
            이벤트의 발생 여부를 반환한다. \n
            0 인 경우: 이벤트 발생하지 않음 \n
            0 이 아닌 경우 : 이벤트 발생 \n
@return     @ref clink_system_wait_event_group_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_system_wait_event_group(
        uint32_t cbox_id,
        uint32_t event_group,
        uint32_t timeout_milliseconds,
        char_t is_delete_in_queue,
        char_t* p_result);

/**
@brief      지정된 group 명의 이벤트가 발생할 때까지 기다린다. \n
            (blocking 상태 지속)
@param[in]  event_group
            수신할 event 의 그룹명 \n
            CLINK_EVENT_GRP 참조.
@param[in]  event_subgroup
            수신할 event 의 subgroup 명
@param[in]  timeout_milliseconds
            event 를 대기하는 최대 시간 (단위 : milliseconds)
@param[in]  is_delete_in_queue
            <STRONG>0 인 경우:</STRONG> 이벤트 발생 후, 해당 이벤트를 내부 queue 에서 \n
            삭제하지 않는다. 결과적으로, \n
            clink_rpc_system_event_group_callback_add(), \n
            clink_rpc_system_event_group_subgroup_callback_add() 등을 통해 \n
            등록한 callback 함수는 호출된다. \n
            <STRONG>0 이 아닌 경우:</STRONG> 이벤트 발생 후, 해당 이벤트를 내부 queue 에서 \n
            삭제한다. 결과적으로, \n
            clink_rpc_system_event_group_callback_add(), \n
            clink_rpc_system_event_group_subgroup_callback_add() 등을 통해 \n
            등록한 callback 함수는 호출되지 않는다. \n
@param[out]  p_result
            이벤트의 발생 여부를 반환한다. \n
            0 인 경우: 이벤트 발생하지 않음 \n
            0 이 아닌 경우 : 이벤트 발생 \n
@return     @ref clink_system_wait_event_group_subgroup_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_system_wait_event_group_subgroup(
        uint32_t cbox_id,
        uint32_t event_group,
        uint32_t event_subgroup,
        uint32_t timeout_milliseconds,
        char_t is_delete_in_queue,
        char_t* p_result);

/**
@brief      지정된 group 명의 이벤트가 발생할 때까지 기다린다. \n
            (blocking 상태 지속)
@param[in]  event_group
            수신할 event 의 그룹명 \n
            CLINK_EVENT_GRP 참조.
@param[in]  event_subgroup
            수신할 event 의 subgroup 명
@param[in]  event_sender_id
            이벤트 송신 객체의 ID \n
            CLINK_EVENT_GRP 및 각 subgroup 문서 참조
@param[in]  timeout_milliseconds
            event 를 대기하는 최대 시간 (단위 : milliseconds)
@param[in]  is_delete_in_queue
            <STRONG>0 인 경우:</STRONG> 이벤트 발생 후, 해당 이벤트를 내부 queue 에서 \n
            삭제하지 않는다. 결과적으로, \n
            clink_rpc_system_event_group_callback_add(), \n
            clink_rpc_system_event_group_subgroup_callback_add() 등을 통해 \n
            등록한 callback 함수는 호출된다. \n
            <STRONG>0 이 아닌 경우:</STRONG> 이벤트 발생 후, 해당 이벤트를 내부 queue 에서 \n
            삭제한다. 결과적으로, \n
            clink_rpc_system_event_group_callback_add(), \n
            clink_rpc_system_event_group_subgroup_callback_add() 등을 통해 \n
            등록한 callback 함수는 호출되지 않는다. \n
@param[out]  p_result
            이벤트의 발생 여부를 반환한다. \n
            0 인 경우: 이벤트 발생하지 않음 \n
            0 이 아닌 경우 : 이벤트 발생 \n
@return     @ref clink_system_wait_event_group_subgroup_sender_id_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_system_wait_event_group_subgroup_sender_id(
        uint32_t cbox_id,
        uint32_t event_group,
        uint32_t event_subgroup,
        uint32_t event_sender_id,
        uint32_t timeout_milliseconds,
        char_t is_delete_in_queue,
        char_t* p_result);

/**
@brief      이벤트 버퍼를 비운다. \n
            해당 이벤트 버퍼는 clink_system_wait_event_xxx() 계열의  \n
            API에서 사용된다.
@return     @ref clink_system_event_queue_clear_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_system_event_queue_clear(
        uint32_t cbox_id);

/**
@brief      입력된 ms 만큼 RTOS 와 시간을 동기화하여 sleep 한다.
@param[in]  sleep_ms
            sleep 시간 [ms]
@return     @ref clink_rt_sleep_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_rt_sleep(
        uint32_t cbox_id,
        uint32_t sleep_ms);

/** @} */


//------------------------------------------------------------------
// 200_CONTROL_BOX
//------------------------------------------------------------------
/** \addtogroup 200_CONTROL_BOX 200_CONTROL_BOX
*  @{
*/

/**
@brief      Control Box 와 연결된 모든 로봇의 상태를 초기화 한다. \n
            Error 가 있는 경우, Error Clear 를 시도한다. \n
@remark     내부적으로 Control Box 를 리셋한 후, \n
            clink_rpc_robot_reset() 를 호출하여 로봇을 초기화한다.
@return     @ref clink_cbox_reset_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_reset(
        uint32_t cbox_id);

/**
@brief      현재 cbox에 설정된 Operational Mode 를 반환한다.
@param[out] p_mode
            현재 cbox에 설정된 Operational Mode 값
@return     @ref clink_cbox_master_operational_mode_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_master_operational_mode_get(
        uint32_t cbox_id,
        CLINK_OPERATIONAL_MODE* p_mode);

/**
@brief      cbox의 Operational Mode 를 설정한다.
@param[in]  operational_mode
            EtherCAT 의 4가지 operational mode 중 한 가지를 선택한다.
@remark     내부적으로 연결된 모든 slave의 operational mode 를 변경한다. \n
            Servo On 을 하기 위해서는 CLINK_OPERATIONAL_MODE_OPERATIONAL \n
            를 선택해야 한다. 그 외 모드는 특수한 경우를 제외하고는 \n
            일반 사용자가 선택할 필요는 없다.\n
@return     @ref clink_cbox_master_operational_mode_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_master_operational_mode_set(
        uint32_t cbox_id,
        CLINK_OPERATIONAL_MODE operational_mode);

/**
@brief      cbox의 ecat 연결을 시도한다.
@return     @ref clink_cbox_master_ecat_connect_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_master_ecat_connect(
        uint32_t cbox_id);

/**
@brief      cbox의 ecat 연결을 해제한다.
@return     @ref clink_cbox_master_ecat_disconnect_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_master_ecat_disconnect(
        uint32_t cbox_id);

/**
@brief      제어할 Control Box 의 이름을 반환한다.
@param[out] p_cbox_name 
            Control Box 의 고유 이름.
@return     @ref clink_cbox_name_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_cbox_name_get(
        uint32_t cbox_id,
        CLINK_CBOX_MODEL* p_cbox_name);

/**
@brief      제어할 Control Box 에 연결된 비상정지 버튼의 상태를 반환한다.
@param[out] p_button_state
            CLINK_BUTTON_STATE_PUSHED : 버튼이 눌린상태 \n
            CLINK_BUTTON_STATE_RELEASED : 버튼에 눌려지지 않은 상태.
@return     @ref clink_cbox_emergency_button_state_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_cbox_emergency_button_state_get(
        uint32_t cbox_id,
        CLINK_BUTTON_STATE* p_button_state);

/**
@brief      AMR에 연결된 비상정지 버튼의 상태를 반환한다.
@param[in]  amr_emg_index
            amr emg index
@param[out] p_button_state
            CLINK_BUTTON_STATE_PUSHED : 버튼이 눌린상태 \n
            CLINK_BUTTON_STATE_RELEASED : 버튼에 눌려지지 않은 상태.
@return     @ref clink_cbox_amr_emergency_button_state_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_amr_emergency_button_state_get(
        uint32_t cbox_id,
        uint32_t amr_emg_index,
        CLINK_BUTTON_STATE* p_button_state);

/**
@brief      cbox의 ecat 연결 상태를 반환한다.
@param[out] p_ecat_conn_state
            CLINK_ECAT_CONN_STATE_CONNECT_TRYING : 연결 시도 중 \n
            CLINK_ECAT_CONN_STATE_CONNECTED : 연결 완료 \n
            CLINK_ECAT_CONN_STATE_DISCONNECT_TRYING : 연결 해제 시도 중 \n
            CLINK_ECAT_CONN_STATE_DISCONNECTED : 연결 해제 완료
@return     @ref clink_cbox_ecat_connection_state_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ecat_connection_state_get(
        uint32_t cbox_id,
        CLINK_ECAT_CONN_STATE* p_ecat_conn_state);

/**
@brief      Teaching Pendant 에 연결된 파워 버튼의 상태를 반환한다.
@param[out] p_button_state
            CLINK_BUTTON_STATE_PUSHED : 버튼이 눌린상태 \n
            CLINK_BUTTON_STATE_RELEASED : 버튼에 눌려지지 않은 상태.
@return     @ref clink_cbox_power_button_state_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_power_button_state_get(
        uint32_t cbox_id,
        CLINK_BUTTON_STATE* p_button_state);

/** @} */


//------------------------------------------------------------------
// 201_CBOX_IO
//------------------------------------------------------------------
/** \addtogroup 201_CBOX_IO 201_CBOX_IO
*  @{
*/

/**
@brief      Control Box 의 Digital IO 의 특정 포트 (Input) 의 값을 읽는다.
@param[in]  io_number 
            Digital IO 번호
@param[out] p_value 
            현재 해당 IO 를 통해 입력되고 있는 값
@return     @ref clink_cbox_io_digital_input_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_cbox_io_digital_input_get(
        uint32_t cbox_id,
        uint32_t io_number,
        uint32_t* p_value);

/**
@brief      clink_cbox_io_digital_output_set() 를 통해 설정한 값을 반환한다. \n
            즉, 현재 해당 포트를 통해 출력되는 값을 반환한다.
@param[in]  io_number 
            Digital IO 번호
@param[out] p_value
            현재 io_number 에 해당하는 Digital IO 를 통해 출력되는 값
@return     @ref clink_cbox_io_digital_output_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_io_digital_output_get(
        uint32_t cbox_id,
        uint32_t io_number,
        uint32_t* p_value);

/**
@brief      Control Box 의 Digital IO 의 특정 포트 (Output) 의 값을 설정한다.
@param[in]  io_number 
            Digital IO 번호
@param[in]  io_value  
            Digital IO 에 설정할 값
@return     @ref clink_cbox_io_digital_output_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_cbox_io_digital_output_set(
        uint32_t cbox_id,
        uint32_t io_number, 
        uint32_t io_value);

/**
@brief      Control Box 의 Digital IO 의 여러 특정 포트 (Output) 의 값을 설정한다.
@param[in]  num_of_ios
            설정할 IO 갯수
@param[in]  p_io_number
            Digital IO 번호
@param[in]  p_io_value
            Digital IO 에 설정할 값
@return     @ref clink_cbox_io_digital_output_multiple_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_io_digital_output_multiple_set(
        uint32_t cbox_id,
        uint32_t num_of_ios,
        const uint32_t* p_io_number,
        const uint32_t* p_io_value);

/**
@brief      Control Box 의 Analog IO 의 특정 포트 (Input) 값을 반환한다.
@param[in]  io_number 
            Analog IO 번호
@param[out] p_value
            현재 io_number 에 해당하는 Analog IO 를 통해 입력되는 값
@return     @ref clink_cbox_io_analog_input_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_cbox_io_analog_input_get(
        uint32_t cbox_id,
        uint32_t io_number,
        clink_float_t* p_value);

/**
@brief      clink_cbox_io_analog_output_set() 를 통해 설정한 값을 반환한다. \n
@param[in]  io_number
            Analog IO 번호
@param[out] p_value
            현재 io_number 에 해당하는 Analog IO 를 통해 출력되는 값
@return     @ref clink_cbox_io_analog_output_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_cbox_io_analog_output_get(
        uint32_t cbox_id,
        uint32_t io_number,
        clink_float_t* p_value);

/**
@brief      Control Box 의 Analog IO 의 특정 포트 (Output) 의 값을 설정한다.
@param[in]  io_number
            Analog IO 번호
@param[in]  io_value
            Analog IO 에 설정할 값
@return     @ref clink_cbox_io_analog_output_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_cbox_io_analog_output_set(
        uint32_t cbox_id,
        uint32_t io_number, 
        clink_float_t io_value);

/**
@brief      Control Box 의 Analog IO 의 특정 포트 (Input) 의 모드를 반환한다. \n
            현재, Voltage (전압), Current (전류) 모드가 선택가능하다.
@param[in]  io_number 
            Analog IO 번호
@param[out] p_mode
            CLINK_ANALOG_IO_MODE_VOLTAGE : 전압 모드 \n
            CLINK_ANALOG_IO_MODE_CURRENT : 전류 모드
@param[out] p_i_reserved
            Reserved Option \n
            각 사용법은 각 Control Box의 문서 참조.
@param[out] p_f_reserved
            Reserved Option \n
            각 사용법은 각 Control Box의 문서 참조.
@return     @ref clink_cbox_io_analog_input_mode_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_io_analog_input_mode_get(
        uint32_t cbox_id,
        uint32_t io_number,
        CLINK_ANALOG_IO_MODE* p_mode,
        int32_t* p_i_reserved,
        clink_float_t* p_f_reserved);

/**
@brief      Control Box 의 Analog IO 의 특정 포트 (Output) 의 모드를 반환한다. \n
            현재, Voltage (전압), Current (전류) 모드가 선택가능하다.
@param[in]  io_number
            Analog IO 번호
@param[out] p_mode
            CLINK_ANALOG_IO_MODE_VOLTAGE : 전압 모드 \n
            CLINK_ANALOG_IO_MODE_CURRENT : 전류 모드
@param[out] p_i_reserved
            Reserved Option \n
            각 사용법은 각 Control Box의 문서 참조.
@param[out] p_f_reserved
            Reserved Option \n
            각 사용법은 각 Control Box의 문서 참조.
@return     @ref clink_cbox_io_analog_output_mode_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_io_analog_output_mode_get(
        uint32_t cbox_id,
        uint32_t io_number,
        CLINK_ANALOG_IO_MODE* p_mode,
        int32_t* p_i_reserved,
        clink_float_t* p_f_reserved);

/**
@brief      Control Box 의 Analog IO 의 특정 포트 (Input) 의 모드를 설정한다. \n
            현재, Voltage (전압), Current (전류) 모드가 선택가능하다.
@param[in]  io_number
            Analog IO 번호
@param[in]  io_mode
            전류 / 전압 선택
@param[in]  i_reserved
            Reserved Option \n
            각 사용법은 각 Control Box의 문서 참조.
@param[in]  f_reserved
            Reserved Option \n
            각 사용법은 각 Control Box의 문서 참조.
@return     @ref clink_cbox_io_analog_input_mode_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_io_analog_input_mode_set(
        uint32_t cbox_id,
    uint32_t io_number,
    CLINK_ANALOG_IO_MODE io_mode,
    int32_t i_reserved,
    clink_float_t f_reserved);

/**
@brief      Control Box 의 Analog IO 의 특정 포트 (output) 의 모드를 설정한다. \n
            현재, Voltage (전압), Current (전류) 모드가 선택가능하다.
@param[in]  io_number 
            Analog IO 번호
@param[in]  io_mode
            전류 / 전압 선택
@param[in]  i_reserved
            Reserved Option \n
            각 사용법은 각 Control Box의 문서 참조.
@param[in]  f_reserved
            Reserved Option \n
            각 사용법은 각 Control Box의 문서 참조.
@return     @ref clink_cbox_io_analog_output_mode_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_cbox_io_analog_output_mode_set(
        uint32_t cbox_id,
        uint32_t io_number, 
        CLINK_ANALOG_IO_MODE io_mode,
        int32_t i_reserved,
        clink_float_t f_reserved);

/**
@brief      Control Box 의 Flexible Digital IO 의 특정 포트 (Input) 의 값을 읽는다.
@param[in]  io_number
            Digital IO 번호
@param[out] p_value
            현재 해당 IO 를 통해 입력되고 있는 값
@return     @ref clink_cbox_redundant_io_digital_input_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_redundant_io_digital_input_get(
        uint32_t cbox_id,
        uint32_t io_number,
        uint32_t* p_value);

/**
@brief      Control Box 의 Flexible Digital IO 의 특정 포트 (Output) 의 값을 읽는다.
@param[in]  io_number
            Digital IO 번호
@param[out] p_value
            현재 해당 IO 를 통해 출력되고 있는 값
@return     @ref clink_cbox_redundant_io_digital_output_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_redundant_io_digital_output_get(
        uint32_t cbox_id,
        uint32_t io_number,
        uint32_t* p_value);

/**
@brief      Control Box 의 특정 두 포트를 활용하여 기능을 실행한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 ID 값
@param[in]  io_number1
            첫 번째 IO 번호
@param[in]  io_number2
            두 번째 IO 번호
@param[in]  io_func
            할당 기능 이하가 사용 가능하다. \n
            CLINK_CBOX_IO_REDUNDANT_IN_EMERGENCY_STOP \n
            CLINK_CBOX_IO_REDUNDANT_IN_REDUCED_MODE
@return     @ref clink_cbox_io_redundant_in_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_io_redundant_in_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t io_number1,
        uint32_t io_number2,
        CLINK_CBOX_IO_REDUNDANT io_func);

/**
@brief      clink_cbox_io_redundant_in_set() 을 통해 할당한 기능을 삭제한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 ID 값
@param[in]  io_number1
            첫 번째 IO 번호
@param[in]  io_number2
            두 번째 IO 번호
@return     @ref clink_cbox_io_redundant_in_remove_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_io_redundant_in_remove(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t io_number1,
        uint32_t io_number2);

/**
@brief      clink_cbox_io_redundant_in_set() 를 통해 할당된 
            모든 포트의 모든 기능을 일괄 삭제한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 ID 값
@return     @ref clink_cbox_io_redundant_in_remove_all_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_io_redundant_in_remove_all(
        uint32_t cbox_id,
        uint32_t robot_id);

/**
@brief      Control Box 의 특정 두 포트를 활용하여 기능을 실행한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 ID 값
@param[in]  io_number1
            첫 번째 IO 번호
@param[in]  io_number2
            두 번째 IO 번호
@param[in]  io_func
            할당 기능 이하가 사용 가능하다. \n
            CLINK_CBOX_IO_REDUNDANT_OUT_EMERGENCY_STOP \n
            CLINK_CBOX_IO_REDUNDANT_OUT_ROBOT_MOVING \n
            CLINK_CBOX_IO_REDUNDANT_OUT_ROBOT_NOT_STOPPING \n
            CLINK_CBOX_IO_REDUNDANT_OUT_COLLISION_DETECTED \n
            CLINK_CBOX_IO_REDUNDANT_OUT_REDUCED_MODE \n
            CLINK_CBOX_IO_REDUNDANT_OUT_NOT_REDUCED_MODE
@return     @ref clink_cbox_io_redundant_out_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_io_redundant_out_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t io_number1,
        uint32_t io_number2,
        CLINK_CBOX_IO_REDUNDANT io_func);

/**
@brief      clink_cbox_io_redundant_out_set() 을 통해 할당한 기능을 삭제
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 ID 값
@param[in]  io_number1
            첫 번째 IO 번호
@param[in]  io_number2
            두 번째 IO 번호
@return     @ref clink_cbox_io_redundant_out_remove_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_io_redundant_out_remove(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t io_number1,
        uint32_t io_number2);

/**
@brief      clink_cbox_io_redundant_out_set() 을 통해 할당된 
            모든 포트의 모든 기능을 일괄 삭제한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 ID 값
@return     @ref clink_cbox_io_redundant_out_remove_all_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_io_redundant_out_remove_all(
        uint32_t cbox_id,
        uint32_t robot_id);

/**
@brief      제어박스에 설치된 IO 보드 모듈의 SW 버전을 리턴한다.
@return     io module sw version
*/
CLINK_EXPORT const char_t*
    clink_rpc_cbox_io_module_sw_version_get(
        uint32_t cbox_id);

/** @} */


//------------------------------------------------------------------
// 202_GEN_CBOX_IO
//------------------------------------------------------------------
/** \addtogroup 202_GEN_CBOX_IO 202_GEN_CBOX_IO
*  @{
*/

/**
@brief      Control Box 의 Digital IO 의 특정 포트 (Input) 의 값을 읽는다.
@param[in]  io_index
            io module index
@param[in]  io_number
            Digital IO 번호
@param[out] p_value
            현재 해당 IO 를 통해 입력되고 있는 값
@return     @ref clink_gen_cbox_io_digital_input_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_gen_cbox_io_digital_input_get(
        uint32_t cbox_id,
        uint32_t io_index,
        uint32_t io_number,
        uint32_t* p_value);

/**
@brief      clink_cbox_digital_output_set() 를 통해 설정한 값을 반환한다. \n
            즉, 현재 해당 포트를 통해 출력되는 값을 반환한다.
@param[in]  io_index
            io module index
@param[in]  io_number
            Digital IO 번호
@param[out] p_value
            현재 io_number 에 해당하는 Digital IO 를 통해 출력되는 값
@return     @ref clink_gen_cbox_io_digital_output_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_gen_cbox_io_digital_output_get(
        uint32_t cbox_id,
        uint32_t io_index,
        uint32_t io_number,
        uint32_t* p_value);

/**
@brief      Control Box 의 Digital IO 의 특정 포트 (Output) 의 값을 설정한다.
@param[in]  io_index
            io module index
@param[in]  io_number
            Digital IO 번호
@param[in]  io_value
            Digital IO 에 설정할 값
@return     @ref clink_gen_cbox_io_digital_output_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_gen_cbox_io_digital_output_set(
        uint32_t cbox_id,
        uint32_t io_index,
        uint32_t io_number,
        uint32_t io_value);

/**
@brief      Control Box 의 Digital IO 의 여러 특정 포트 (Output) 의 값을 설정한다.
@param[in]  io_index
            io module index
@param[in]  num_of_ios
            설정할 IO 갯수
@param[in]  p_io_number
            Digital IO 번호
@param[in]  p_io_value
            Digital IO 에 설정할 값
@return     @ref clink_gen_cbox_io_digital_output_multiple_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_gen_cbox_io_digital_output_multiple_set(
        uint32_t cbox_id,
        uint32_t io_index,
        uint32_t num_of_ios,
        const uint32_t* p_io_number,
        const uint32_t* p_io_value);

/**
@brief      Control Box 의 Analog IO 의 특정 포트 (Input) 값을 반환한다.
@param[in]  io_index
            io module index
@param[in]  io_number
            Analog IO 번호
@param[out] p_value
            현재 io_number 에 해당하는 Analog IO 를 통해 출력되는 값
@return     @ref clink_gen_cbox_io_analog_input_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_gen_cbox_io_analog_input_get(
        uint32_t cbox_id,
        uint32_t io_index,
        uint32_t io_number,
        clink_float_t* p_value);

/**
@brief      clink_cbox_analog_output_set() 를 통해 설정한 값을 반환한다. \n
@param[in]  io_index
            io module index
@param[in]  io_number
            Analog IO 번호
@param[out] p_value
            현재 io_number 에 해당하는 Analog IO 를 통해 출력되는 값
@return     @ref clink_gen_cbox_io_analog_output_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_gen_cbox_io_analog_output_get(
        uint32_t cbox_id,
        uint32_t io_index,
        uint32_t io_number,
        clink_float_t* p_value);

/**
@brief      Control Box 의 Analog IO 의 특정 포트 (Output) 의 값을 설정한다.
@param[in]  io_index
            io module index
@param[in]  io_number
            Analog IO 번호
@param[in]  io_value
            Analog IO 에 설정할 값
@return     @ref clink_gen_cbox_io_analog_output_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_gen_cbox_io_analog_output_set(
        uint32_t cbox_id,
        uint32_t io_index,
        uint32_t io_number,
        clink_float_t io_value);

/**
@brief      Control Box 의 Analog IO 의 특정 포트 (Input) 의 모드를 반환한다. \n
            현재, Voltage (전압), Current (전류) 모드가 선택가능하다.
@param[in]  io_index
            io module index
@param[in]  io_number
            Analog IO 번호
@param[out] p_mode
            CLINK_ANALOG_IO_MODE_VOLTAGE : 전압 모드 \n
            CLINK_ANALOG_IO_MODE_CURRENT : 전류 모드
@param[out] p_i_reserved
            Reserved Option \n
            각 사용법은 각 Control Box의 문서 참조.
@param[out] p_f_reserved
            Reserved Option \n
            각 사용법은 각 Control Box의 문서 참조.
@return     @ref clink_gen_cbox_io_analog_input_mode_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_gen_cbox_io_analog_input_mode_get(
        uint32_t cbox_id,
        uint32_t io_index,
        uint32_t io_number,
        CLINK_ANALOG_IO_MODE* p_mode,
        int32_t* p_i_reserved,
        clink_float_t* p_f_reserved);

/**
@brief      Control Box 의 Analog IO 의 특정 포트 (Output) 의 모드를 반환한다. \n
            현재, Voltage (전압), Current (전류) 모드가 선택가능하다.
@param[in]  io_index
            io module index
@param[in]  io_number
            Analog IO 번호
@param[out] p_mode
            CLINK_ANALOG_IO_MODE_VOLTAGE : 전압 모드 \n
            CLINK_ANALOG_IO_MODE_CURRENT : 전류 모드
@param[out] p_i_reserved
            Reserved Option \n
            각 사용법은 각 Control Box의 문서 참조.
@param[out] p_f_reserved
            Reserved Option \n
            각 사용법은 각 Control Box의 문서 참조.
@return     @ref clink_gen_cbox_io_analog_output_mode_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_gen_cbox_io_analog_output_mode_get(
        uint32_t cbox_id,
        uint32_t io_index,
        uint32_t io_number,
        CLINK_ANALOG_IO_MODE* p_mode,
        int32_t* p_i_reserved,
        clink_float_t* p_f_reserved);

/**
@brief      Control Box 의 Analog IO 의 특정 포트 (Input) 의 모드를 설정한다. \n
            현재, Voltage (전압), Current (전류) 모드가 선택가능하다.
@param[in]  io_index
            io module index
@param[in]  io_number
            Analog IO 번호
@param[in]  io_mode
            전류 / 전압 선택
@param[in]  i_reserved
            Reserved Option \n
            각 사용법은 각 Control Box의 문서 참조.
@param[in]  f_reserved
            Reserved Option \n
            각 사용법은 각 Control Box의 문서 참조.
@return     @ref clink_gen_cbox_io_analog_input_mode_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_gen_cbox_io_analog_input_mode_set(
        uint32_t cbox_id,
        uint32_t io_index,
        uint32_t io_number,
        CLINK_ANALOG_IO_MODE io_mode,
        int32_t i_reserved,
        clink_float_t f_reserved);

/**
@brief      Control Box 의 Analog IO 의 특정 포트 (output) 의 모드를 설정한다. \n
            현재, Voltage (전압), Current (전류) 모드가 선택가능하다.
@param[in]  io_index
            io module index
@param[in]  io_number
            Analog IO 번호
@param[in]  io_mode
            전류 / 전압 선택
@param[in]  i_reserved
            Reserved Option \n
            각 사용법은 각 Control Box의 문서 참조.
@param[in]  f_reserved
            Reserved Option \n
            각 사용법은 각 Control Box의 문서 참조.
@return     @ref clink_gen_cbox_io_analog_output_mode_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_gen_cbox_io_analog_output_mode_set(
        uint32_t cbox_id,
        uint32_t io_index,
        uint32_t io_number,
        CLINK_ANALOG_IO_MODE io_mode,
        int32_t i_reserved,
        clink_float_t f_reserved);

/**
@brief      Control Box 의 Flexible Digital IO 의 특정 포트 (Input) 의 값을 읽는다.
@param[in]  io_index
            io module index
@param[in]  io_number
            Digital IO 번호
@param[out] p_value
            현재 해당 IO 를 통해 입력되고 있는 값
@return     @ref clink_gen_cbox_redundant_io_digital_input_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_gen_cbox_redundant_io_digital_input_get(
        uint32_t cbox_id,
        uint32_t io_index,
        uint32_t io_number,
        uint32_t* p_value);

/**
@brief      Control Box 의 Flexible Digital IO 의 특정 포트 (Output) 의 값을 읽는다.
@param[in]  io_index
            io module index
@param[in]  io_number
            Digital IO 번호
@param[out] p_value
            현재 해당 IO 를 통해 출력되고 있는 값
@return     @ref clink_gen_cbox_redundant_io_digital_output_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_gen_cbox_redundant_io_digital_output_get(
        uint32_t cbox_id,
        uint32_t io_index,
        uint32_t io_number,
        uint32_t* p_value);

/**
@brief      Control Box 의 FSOE Digital IO 의 특정 포트 (INPUT) 의 값을 읽는다. \n
            즉, 현재 해당 포트를 통해 입력되는 값을 반환한다.
@param[in]  io_index
            io module index
@param[in]  io_number
            Digital IO 번호
@param[out] p_value
            현재 io_number 에 해당하는 Digital IO 를 통해 입력되는 값
@return     @ref clink_gen_cbox_fsoe_io_digital_input_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_gen_cbox_fsoe_io_digital_input_get(
        uint32_t cbox_id,
        uint32_t io_index,
        CLINK_FSOE_IO_INPUT_DATA io_number,
        uint32_t* p_value);

/**
@brief      Control Box 의 FSOE Digital IO 의 특정 포트 (OUTPUT) 의 값을 읽는다. \n
            즉, 현재 해당 포트를 통해 출력되는 값을 반환한다.
@param[in]  io_index
            io module index
@param[in]  io_number
            Digital IO 번호
@param[out] p_value
            현재 io_number 에 해당하는 Digital IO 를 통해 출력되는 값
@return     @ref clink_gen_cbox_fsoe_io_digital_output_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_gen_cbox_fsoe_io_digital_output_get(
        uint32_t cbox_id,
        uint32_t io_index,
        CLINK_FSOE_IO_OUTPUT_DATA io_number,
        uint32_t* p_value);

/**
@brief      Control Box 의 Digital FSOE IO 의 특정 포트 (OUTPUT)의 값을 설정한다.
@param[in]  io_index
            io module index
@param[in]  io_number
            Digital IO 번호
@param[in]  io_value
            Digital IO 에 설정할 값
@return     @ref clink_gen_cbox_fsoe_io_digital_output_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_gen_cbox_fsoe_io_digital_output_set(
        uint32_t cbox_id,
        uint32_t io_index,
        CLINK_FSOE_IO_OUTPUT_DATA io_number,
        uint32_t io_value);

/** @} */


//------------------------------------------------------------------
// 203_CBOX_SAFETY_MODULE
//------------------------------------------------------------------
/** \addtogroup 203_CBOX_SAFETY_MODULE 203_CBOX_SAFETY_MODULE
*  @{
*/

/**
@brief      Control Box 내 Safety Module의 Digital IO 의 특정 포트 (Input) 의 값을 읽는다.
@param[in]  io_number
            Digital IO 번호
@param[out] p_value
            현재 해당 IO 를 통해 입력되고 있는 값
@return     @ref clink_cbox_safety_module_redundant_io_digital_input_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_safety_module_redundant_io_digital_input_get(
        uint32_t cbox_id,
        uint32_t io_number,
        uint32_t* p_value);

/**
@brief      Control Box 에 설치된 Safety 모듈의 Software 버전을 리턴한다. \n
@return     safety module sw version
*/
CLINK_EXPORT const char_t*
    clink_rpc_cbox_safety_module_sw_version_get(
        uint32_t cbox_id);

/**
@brief      Control Box 에 설치된 Safety 모듈의 Software 버전을 리턴한다. \n
@param[in]  safety_index
            해당 하는 safety 의 index \n
            HCR 로봇의 경우 0 or 1 입력
@return     index 별 safety module sw version
*/
CLINK_EXPORT const char_t*
    clink_rpc_cbox_safety_module_sw_version_get_with_index(
        uint32_t cbox_id,
        uint32_t safety_index);

/**
@brief      Control Box 에 설치된 Safety 모듈의 48V SMPS 상태를 리턴한다.
@param[out] p_switch 
            CLINK_SWITCH_ON : SMPS 48V 가 Enable 된 상태 \n
            CLINK_SWITCH_OFF : SMPS 48V 가 Disable 된 상태
@return     @ref clink_cbox_safety_module_48v_smps_switch_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_safety_module_48v_smps_switch_get(
        uint32_t cbox_id,
        CLINK_SWITCH* p_switch);

/**
@brief      Control Box 에 있는 Safety Module 의 특정 두 포트를 활용하여 기능을 실행한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 ID 값
@param[in]  io_number1
            첫 번째 IO 번호
@param[in]  io_number2
            두 번째 IO 번호
@param[in]  io_func
            할당 기능 이하가 사용 가능하다. \n
            CLINK_SAFETY_MODULE_IO_REDUNDANT_IN_PAUSE_RESUME \n
            CLINK_SAFETY_MODULE_IO_REDUNDANT_IN_ENABLE_SWITCH
@return     @ref clink_cbox_safety_module_io_redundant_in_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_safety_module_io_redundant_in_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t io_number1,
        uint32_t io_number2,
        CLINK_SAFETY_MODULE_IO_REDUNDANT io_func);

/**
@brief      clink_cbox_safety_module_io_redundant_in_set() 를 통해 할당한 기능을 삭제
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 ID 값
@param[in]  io_number1
            첫 번째 IO 번호
@param[in]  io_number2
            두 번째 IO 번호
@return     @ref clink_cbox_safety_module_io_redundant_in_remove_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_safety_module_io_redundant_in_remove(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t io_number1,
        uint32_t io_number2);

/**
@brief      clink_cbox_safety_module_io_redundant_in_set() 
            을 통해 추가한 모든 기능을 일괄 삭제한다. 
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 ID 값
@return     @ref clink_cbox_safety_module_io_redundant_in_remove_all_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_safety_module_io_redundant_in_remove_all(
        uint32_t cbox_id,
        uint32_t robot_id);

/** @} */


//------------------------------------------------------------------
// 204_CBOX_EXT
//------------------------------------------------------------------
/** \addtogroup 204_CBOX_EXT 204_CBOX_EXT
*  @{
*/

/**
@brief      UDP 를 활용하여 설정된 목적지로 CBOX 의 데이터 (예: CBOX 의 IO) 및 \n
            연결된 모든 로봇의 기본적인 정보를 전송한다. (Realtime Data Monitoring)
@param[in]  p_my_ip
            현재 이 API 를 사용하는 PC 의 NIC 의 IP 주소 
@param[in]  my_port
            UDP 소켓에 사용할 포트 번호. 0 을 입력하는 경우 임의로 할당
@param[in]  p_dest_ip
            목적지 PC 의 NIC 의 IP 주소
@param[in]  dest_port
            목적지 PC 의 포트 번호
@param[in]  hz_in_sec
            1초에 UDP 패킷을 전송할 횟수. \n
            예: 100 인 경우, 100 회를 전송한다.
@remark     본 기능은 실시간을 보장하지 않는다. \n
            약, 99% 정도의 확률로 100회를 보장한다고 간주하면 된다.
@return     @ref clink_cbox_ext_rdm_start_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_rdm_start(
        uint32_t cbox_id,
        const char_t* p_my_ip,
        uint16_t my_port,
        const char_t* p_dest_ip,
        uint16_t dest_port,
        uint32_t hz_in_sec);

/**
@brief      clink_cbox_ext_rdm_start() 를 통해 시작한 서비스를 종료한다.
@remark     clink_cbox_ext_rdm_start() 를 통해 시작하지 않은 경우, 에러를 반환한다.
@return     @ref clink_cbox_ext_rdm_stop_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_rdm_stop(
        uint32_t cbox_id);

/**
@brief      UDP/IP 를 활용하여 설정된 목적지로 로봇의 각종 정보를 전달한다. (Real-Time State Streaming) \n
            RDM 기능에 비해 정확한 주기적인 데이터를 제공한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 ID 값
@param[in]  p_dest_ip
            목적지 PC 의 NIC 의 IP 주소
@param[in]  dest_port
            목적지 PC 의 포트 번호
@param[in]  hz_in_sec
			1초에 UDP 패킷을 전송할 횟수. 최대 500Hz 까지 지원한다.
@remark     목적지의 PC에서 UDP 패킷을 수신하기 위해서 아래와 같은 설정이 필요하다.(목적지 PC 설정)
			Control Box 내 EPC 의 1번 NIC 와 목적지 PC 간 Local Network 구성이 필요하다. \n
            route add 192.168.222.0 mask 255.255.255.0 [Control Box IP] (Windows Command Prompt 에서 입력)\n 
            Control Box 의 Default IP 는 192.168.100.200 이다. 즉 변경사항이 없을 시 필요한 기본 설정은 아래와 같다. \n
			ex) route add 192.168.222.0 mask 255.255.255.0 192.168.100.200 \n
            해당 기능은 네트워크 구조 상 하나의 Control Box 대상으로만 사용이 가능하다. \n
            2개 이상의 Control Box 접속 상태에서 사용 불가
@return     @ref clink_cbox_ext_robot_rtss_start_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_robot_rtss_start(
        uint32_t cbox_id,
        uint32_t robot_id,
        const char_t* p_dest_ip,
        uint16_t dest_port,
        uint32_t hz_in_sec);

/**
@brief      clink_cbox_ext_robot_rtss_start() 를 통해 시작한 서비스를 종료한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 ID 값
@return     @ref clink_cbox_ext_robot_rtss_stop_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_robot_rtss_stop(
        uint32_t cbox_id,
        uint32_t robot_id);

/**
@brief      clink_cbox_ext_robot_rtss_start() 를 통해 시작한 서비스를 종료한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 ID 값
@param[out] p_on_off
            현재 해당 로봇의 RTSS 서비스 상태 반환
@return     @ref clink_cbox_ext_robot_rtss_running_state_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_robot_rtss_running_state_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH* p_on_off);

/**
@brief      IO-LINK 의 특정 WP의 상태 정보를 가져온다.
@param[in]  wp_id
            WP의 ID 값을 입력한다. (ID < MAX_NUM_OF_IO_LINK_WP)
@param[out] p_wp_status_byte
            WP의 상태 정보 저장 버퍼 포인터 (1byte)
@return     @ref clink_cbox_ecat_io_link_wp_status_data_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ecat_io_link_wp_status_data_get(
        uint32_t cbox_id,
        uint32_t wp_id,
        uint8_t* p_wp_status_byte);

/**
@brief      IO-LINK 의 특정 WP로 부터 RX 데이터를 가져온다.
@param[in]  wp_id
            WP의 ID 값을 입력한다. (ID < MAX_NUM_OF_IO_LINK_WP)
@param[in]  size_of_data
            입력 값 저장 버퍼 사이즈를 입력한다.
@param[out] p_val
            입력 값 저장 버퍼 포인터 (버퍼사이즈 MAX_NUM_OF_IO_LINK_WP_IO_BYTE byte 고정)
@return     @ref clink_cbox_ecat_io_link_wp_rx_data_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ecat_io_link_wp_rx_data_get(
        uint32_t cbox_id,
        uint32_t wp_id,
        uint32_t size_of_data,
        uint8_t* p_val);

/**
@brief      IO-LINK 의 특정 WP로 부터 TX 데이터를 가져온다.
@param[in]  wp_id
            WP의 ID 값을 입력한다. (ID < MAX_NUM_OF_IO_LINK_WP)
@param[in]  size_of_data
            출력 값 저장 버퍼 사이즈를 입력한다.
@param[out] p_val
            출력 값 저장 버퍼 포인터 (버퍼사이즈 MAX_NUM_OF_IO_LINK_WP_IO_BYTE byte 고정)
@return     @ref clink_cbox_ecat_io_link_wp_tx_data_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ecat_io_link_wp_tx_data_get(
        uint32_t cbox_id,
        uint32_t wp_id,
        uint32_t size_of_data,
        uint8_t* p_val);

/**
@brief      IO-LINK 의 특정 WP로 TX 데이터를 설정한다.
@param[in]  wp_id
            WP의 ID 값을 입력한다. (ID < MAX_NUM_OF_IO_LINK_WP)
@param[in]  size_of_data
            출력 값 저장 버퍼 사이즈를 입력한다.
@param[in]  p_val
            입력할 값을 가지고 있는 버퍼 포인터 (버퍼사이즈 MAX_NUM_OF_IO_LINK_WP_IO_BYTE byte 고정)
@return     @ref clink_cbox_ecat_io_link_wp_tx_data_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ecat_io_link_wp_tx_data_set(
        uint32_t cbox_id,
        uint32_t wp_id,
        uint32_t size_of_data,
        const uint8_t* p_val);

/**
@brief      IO-LINK 의 상태 정보 비트를 가져온다.
@param[in]  io_link_id
            IO-LINK 의 ID 값을 입력한다. (ID < MAX_NUM_OF_IO_LINK)
@param[out] p_status_bit
            status bit 정보 저장 버퍼 포인터 (2 byte)
@return     @ref clink_cbox_ecat_io_link_status_bit_data_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ecat_io_link_status_bit_data_get(
        uint32_t cbox_id,
        uint32_t io_link_id,
        uint16_t* p_status_bit);

/**
@brief      IO-LINK 의 control 정보 비트를 가져온다.
@param[in]  io_link_id
            IO-LINK 의 ID 값을 입력한다. (ID < MAX_NUM_OF_IO_LINK)
@param[out] p_control_bit
            control_bit 정보 저장 버퍼 포인터 (2 byte)
@return     @ref clink_cbox_ecat_io_link_control_bit_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ecat_io_link_control_bit_get(
        uint32_t cbox_id,
        uint32_t io_link_id,
        uint16_t* p_control_bit);

/**
@brief      IO-LINK 의 control 정보 비트를 설정한다.
@param[in]  io_link_id
            IO-LINK 의 ID 값을 입력한다. (ID < MAX_NUM_OF_IO_LINK)
@param[in]  p_control_bit
            control 입력 값을 가지고 있는 버퍼 포인터 (2 byte)
@return     @ref clink_cbox_ecat_io_link_control_bit_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ecat_io_link_control_bit_set(
        uint32_t cbox_id,
        uint32_t io_link_id,
        const uint16_t* p_control_bit);

/**
@brief      IO-LINK 의 input 정보 비트를 가져온다.
@param[in]  io_link_id
            IO-LINK 의 ID 값을 입력한다. (ID < MAX_NUM_OF_IO_LINK)
@param[out]  p_input_bit
            input_bit 정보 저장 버퍼 포인터 (2 byte)
@return     @ref clink_cbox_ecat_io_link_input_bit_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ecat_io_link_input_bit_get(
        uint32_t cbox_id,
        uint32_t io_link_id,
        uint16_t* p_input_bit);

/**
@brief      IO-LINK 의 output 정보 비트를 가져온다.
@param[in]  io_link_id
            IO-LINK 의 ID 값을 입력한다. (ID < MAX_NUM_OF_IO_LINK)
@param[out]  p_output_bit
            output_bit 정보 저장 버퍼 포인터 (2 byte)
@return     @ref clink_cbox_ecat_io_link_output_bit_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ecat_io_link_output_bit_get(
        uint32_t cbox_id,
        uint32_t io_link_id,
        uint16_t* p_output_bit);

/**
@brief      IO-LINK 의 output 정보 비트를 설정한다.
@param[in]  io_link_id
            IO-LINK 의 ID 값을 입력한다. (ID < MAX_NUM_OF_IO_LINK)
@param[in]  p_output_bit
            output_bit 입력 값을 가지고 있는 버퍼 포인터 (2 byte)
@return     @ref clink_cbox_ecat_io_link_output_bit_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ecat_io_link_output_bit_set(
        uint32_t cbox_id,
        uint32_t io_link_id,
        const uint16_t* p_output_bit);

/**
@brief      IO-LINK 의 RX 데이터를 가져온다.
@param[in]  io_link_id
            IO-LINK 의 ID 값을 입력한다. (ID < MAX_NUM_OF_IO_LINK)
@param[in]  size_of_data
            입력 값 저장 버퍼 사이즈를 입력한다.
@param[out] p_val
            입력 값 저장 버퍼 포인터 (버퍼사이즈 MAX_NUM_OF_IO_LINK_IO_BYTE byte 고정)
@return     @ref clink_cbox_ecat_io_link_rx_data_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ecat_io_link_rx_data_get(
        uint32_t cbox_id,
        uint32_t io_link_id,
        uint32_t size_of_data,
        uint8_t* p_val);

/**
@brief      IO-LINK 의 TX 데이터를 가져온다.
@param[in]  io_link_id
            IO-LINK 의 ID 값을 입력한다. (ID < MAX_NUM_OF_IO_LINK)
@param[in]  size_of_data
            출력 값 저장 버퍼 사이즈를 입력한다.
@param[out] p_val
            출력 값 저장 버퍼 포인터 (버퍼사이즈 MAX_NUM_OF_IO_LINK_IO_BYTE byte 고정)
@return     @ref clink_cbox_ecat_io_link_tx_data_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ecat_io_link_tx_data_get(
        uint32_t cbox_id,
        uint32_t io_link_id,
        uint32_t size_of_data,
        uint8_t* p_val);

/**
@brief      IO-LINK 의 TX 데이터를 설정한다.
@param[in]  io_link_id
            IO-LINK 의 ID 값을 입력한다. (ID < MAX_NUM_OF_IO_LINK)
@param[in]  size_of_data
            출력 값 저장 버퍼 사이즈를 입력한다.
@param[in]  p_val
            입력할 값을 가지고 있는 버퍼 포인터 (버퍼사이즈 MAX_NUM_OF_IO_LINK_IO_BYTE byte 고정)
@return     @ref clink_cbox_ecat_io_link_tx_data_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ecat_io_link_tx_data_set(
        uint32_t cbox_id,
        uint32_t io_link_id,
        uint32_t size_of_data,
        const uint8_t* p_val);

/**
@brief      LOW Data IO Device 의 RX 데이터를 가져온다.
@param[in]  device_id
            LOW Data IO Device 의 ID 값을 입력한다. (범위 0-3)
@param[in]  size_of_data
            출력 값 저장 버퍼 사이즈를 입력한다.
@param[out] p_val
            출력 값 저장 버퍼 포인터
@return     @ref clink_cbox_ecat_low_data_io_rx_data_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ecat_low_data_io_rx_data_get(
        uint32_t cbox_id,
        uint32_t device_id,
        uint32_t size_of_data,
        uint8_t* p_val);

/**
@brief      LOW Data IO Device 의 TX 데이터를 가져온다.
@param[in]  device_id
            LOW Data IO Device 의 ID 값을 입력한다. (최대 0-3)
@param[in]  size_of_data
            출력 값 저장 버퍼 사이즈를 입력한다.
@param[out] p_val
            읽어온 값을 저장할 버퍼 포인터
@return     @ref clink_cbox_ecat_low_data_io_tx_data_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ecat_low_data_io_tx_data_get(
        uint32_t cbox_id,
        uint32_t device_id,
        uint32_t size_of_data,
        uint8_t* p_val);

/**
@brief      LOW Data IO Device 의 TX 데이터를 설정한다.
@param[in]  device_id
            LOW Data IO Device 의 ID 값을 입력한다. (최대 0-3)
@param[in]  size_of_data
            입력할 값을 가지고 있는 버퍼 사이즈를 입력한다.
@param[in]  p_val
            입력할 값을 가지고 있는 버퍼 포인터
@return     @ref clink_cbox_ecat_low_data_io_tx_data_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ecat_low_data_io_tx_data_set(
        uint32_t cbox_id,
        uint32_t device_id,
        uint32_t size_of_data,
        const uint8_t* p_val);

/**
@brief      Modbus 서버를 시작한다. 시작과 동시에 TCP/IP 소켓은 listen 을 한다.
            accept 이 되면, 해당 client 는 하나의 thread 로 생성되거나 \n
            또는 모든 소켓을 비동기로 동작해야 한다. \n
            client 는 여러개가 접속이 가능하기 때문에 accept 이후에는 \n
            다시 listen 으로 동작한다.
@param[in]  p_my_ip
            현재 이 API 를 사용하는 PC 의 NIC 의 IP 주소
@param[in]  port_num
             소켓에 사용할 포트 번호. 0 을 입력하는 경우 임의로 할당
@return     @ref clink_cbox_ext_modbus_server_start_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_modbus_server_start(
        uint32_t cbox_id,
        const char_t* p_my_ip,
        uint16_t port_num);

/**
@brief      Modbus 서버를 종료한다.
@return     @ref clink_cbox_ext_modbus_server_stop_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_modbus_server_stop(
        uint32_t cbox_id);

/**
@brief      Local 모드버스 맵의 특정 bit 를 0 또는 1로 설정한다.
@param[in]  address
            서버 모드버스 맵의 주소 값 을 결정 한다. (1~800)
@param[in]  num_of_val
            입력할 값의 개수 (최대 250)
@param[in]  p_val
            입력할 값을 가지고 있는 버퍼 포인터 (버퍼사이즈 num_of_val * sizeof(bool))
@return     @ref clink_cbox_ext_modbus_local_bit_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_modbus_local_bit_set(
        uint32_t cbox_id,
        uint32_t address,
        uint32_t num_of_val,
        const uint8_t* p_val);

/**
@brief      Local 모드버스 맵의 특정 bit 값을 read 한다.
@param[in]  address
            서버 모드버스 맵의 주소 값 을 결정 한다. (1~800)
@param[in]  num_of_val
            읽을 값의 개수 (최대 256)
@param[out] p_val
            읽은 값 저장 버퍼 포인터 (버퍼사이즈 num_of_val * sizeof(bool))
@return     @ref clink_cbox_ext_modbus_local_bit_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_modbus_local_bit_get(
        uint32_t cbox_id,
        uint32_t address,
        uint32_t num_of_val,
        uint8_t* p_val);

/**
@brief      Local 모드버스 맵의 특정 register 값을 설정한다.
@param[in]  address
            서버 모드버스 맵의 주소 값 을 결정 한다. (1~800)
@param[in]  num_of_val
            입력할 값의 개수 (최대 128)
@param[in]  p_val
            입력할 값을 가지고 있는 버퍼 포인터 (버퍼사이즈 num_of_val * sizeof(uint16_t))
@return     @ref clink_cbox_ext_modbus_local_register_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_modbus_local_register_set(
        uint32_t cbox_id,
        uint32_t address,
        uint32_t num_of_val,
        const uint16_t* p_val);

/**
@brief      Local 모드버스 맵의 특정 register 값을 read 한다.
@param[in]  address
            서버 모드버스 맵의 주소 값 을 결정 한다. (1~800)
@param[in]  num_of_val
            읽을 값의 개수 (최대 128)
@param[out] p_val
            읽은 값 저장 버퍼 포인터 (버퍼사이즈 num_of_val * sizeof(uint16_t))
@return     @ref clink_cbox_ext_modbus_local_register_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_modbus_local_register_get(
        uint32_t cbox_id,
        uint32_t address,
        uint32_t num_of_val,
        uint16_t* p_val);

/**
@brief      Modbus 클라이언트를 시작하고 Modbus 클라이언트 인스턴스 ID를 반환한다.
@remark     본 API 에서 client 란, HCR / UR 의 UI 에서 device 의 개념과 동일하다.
@param[in]  p_my_ip
            현재 이 API 를 사용하는 PC 의 NIC 의 IP 주소
@param[in]  p_client_name
            client 의 유니크한 이름
@param[in]  my_port
            클라이언트 소켓에 사용할 포트 번호. 0 을 입력하는 경우 임의로 할당
@param[in]  p_dest_ip
            Modbus 서버 IP 주소
@param[in]  dest_port
            Modbus 서버  접속 포트 번호
@return     @ref clink_cbox_ext_modbus_client_create_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_modbus_client_create(
        uint32_t cbox_id,
        const char_t* p_client_name,
        const char_t* p_my_ip,
        uint16_t my_port,
        const char_t* p_dest_ip,
        uint16_t dest_port);

/**
@brief      Modbus 클라이언트를 종료한다.
@param[in]  p_client_name
            client 의 유니크한 이름
@return     @ref clink_cbox_ext_modbus_client_destroy_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_modbus_client_destroy(
        uint32_t cbox_id,
        const char_t* p_client_name);

/**
@brief      Modbus 클라이언트 상태 정보를 확인한다.
@param[in]  p_client_name
            client 의 유니크한 이름
@param[out] p_status
            클라이언트 상태 정보
@return     @ref clink_cbox_ext_modbus_client_status_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_modbus_client_status_get(
        uint32_t cbox_id,
        const char_t* p_client_name,
        CLINK_MODBUS_CLIENT_STATUS* p_status);

/**
@brief      Modbus 클라이언트 sequential mode 동작을 on/off 입력 한다.
@param[in]  p_client_name
            client 의 유니크한 이름
@param[in] on_off
            CLINK_SWITCH_ON : sequential mode 로 동작 \n
            CLINK_SWITCH_OFF : non-sequential mode 로 동작
@return     @ref clink_cbox_ext_modbus_client_seq_mode_switch_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_modbus_client_seq_mode_switch_set(
        uint32_t cbox_id,
        const char_t* p_client_name,
        CLINK_SWITCH on_off);

/**
@brief      Modbus 클라이언트 sequential mode 동작 on_off를 반환 한다.
@param[in]  p_client_name
            client 의 유니크한 이름
@param[out] p_on_off
            CLINK_SWITCH_ON : sequential mode 로 동작 \n
            CLINK_SWITCH_OFF : non-sequential mode 로 동작
@return     @ref clink_cbox_ext_modbus_client_seq_mode_switch_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_modbus_client_seq_mode_switch_get(
        uint32_t cbox_id,
        const char_t* p_client_name,
        CLINK_SWITCH* p_on_off);

/**
@brief      Modbus 클라이언트 IO를 생성 한다.
@param[in]  p_client_name
            client 의 유니크한 이름
@param[in]  io_type
            Modbus 클라이언트 IO 의 CLINK_MODBUS_CLIENT_IO_TYPE 을 결정 한다.
@param[in]  address
            Modbus 클라이언트 IO 에서 사용할 서버 모드버스 맵의 주소 값 을 결정 한다.
@param[in]  num_of_val
            주소 값 부터 Read or Write 데이터의 개수 를 결정 한다.
@param[in]  p_client_io_name
            client_io 의 유니크한 이름
@return     @ref clink_cbox_ext_modbus_client_io_create_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_modbus_client_io_create(
        uint32_t cbox_id,
        const char_t* p_client_name,
        CLINK_MODBUS_CLIENT_IO_TYPE io_type,
        uint32_t address,
        uint32_t num_of_val,
        const char_t* p_client_io_name);

/**
@brief      Modbus 클라이언트 IO를 삭제 한다.
@param[in]  p_client_io_name
            client_io 의 유니크한 이름
@return     @ref clink_cbox_ext_modbus_client_io_destroy_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_modbus_client_io_destroy(
        uint32_t cbox_id,
        const char_t* p_client_io_name);

/**
@brief      Modbus 클라이언트 IO를 시작 한다.
@param[in]	p_client_io_name
            client_io 의 유니크한 이름
@param[in]  hz
            Modbus 클라이언트 IO의 동작 주기(hz)를 결정 한다.
@return     @ref clink_cbox_ext_modbus_client_io_start_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_modbus_client_io_start(
        uint32_t cbox_id,
        const char_t* p_client_io_name,
        CLINK_MODBUS_CLIENT_IO_UPDATE_HZ hz);

/**
@brief      Modbus 클라이언트 IO 동작을 정지 한다.
@param[in]	p_client_io_name
            client_io 의 유니크한 이름
@return     @ref clink_cbox_ext_modbus_client_io_stop_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_modbus_client_io_stop(
        uint32_t cbox_id,
        const char_t* p_client_io_name);

/**
@brief      Modbus 클라이언트 IO 종류(type)정보를 반환 한다.
@param[in]	p_client_io_name
            client_io 의 유니크한 이름
@param[out] p_io_type
            Modbus 클라이언트 IO 종류(CLINK_MODBUS_CLIENT_IO_TYPE)정보
@return     @ref clink_cbox_modbus_client_io_type_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_modbus_client_io_type_get(
        uint32_t cbox_id,
        const char_t* p_client_io_name,
        CLINK_MODBUS_CLIENT_IO_TYPE* p_io_type);

/**
@brief      Ext Modbus 클라이언트 IO 종류(type)정보를 반환 한다.
@param[in]	p_client_io_name
            client_io 의 유니크한 이름
@param[out] p_io_type
            Modbus 클라이언트 IO 종류(CLINK_MODBUS_CLIENT_IO_TYPE)정보
@return     @ref clink_cbox_ext_modbus_client_io_type_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_modbus_client_io_type_get(
        uint32_t cbox_id,
        const char_t* p_client_io_name,
        CLINK_MODBUS_CLIENT_IO_TYPE* p_io_type);

/**
@brief      Modbus 클라이언트 IO 상태(status)정보를 반환 한다.
@param[in]	p_client_io_name
            client_io 의 유니크한 이름
@param[out] p_status
            Modbus 클라이언트 IO 상태(CLINK_MODBUS_CLIENT_IO_STATUS)정보
@return     @ref clink_cbox_modbus_client_io_status_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_modbus_client_io_status_get(
        uint32_t cbox_id,
        const char_t* p_client_io_name,
        CLINK_MODBUS_CLIENT_IO_STATUS* p_status);

/**
@brief      Modbus 클라이언트 IO 상태(status)정보를 반환 한다.
@param[in]	p_client_io_name
            client_io 의 유니크한 이름
@param[out] p_status
            Modbus 클라이언트 IO 상태(CLINK_MODBUS_CLIENT_IO_STATUS)정보
@param[out] p_response_time_ms
            Modbus 클라이언트 IO 응답 속도(millisec)
@return     @ref clink_cbox_ext_modbus_client_io_status_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_modbus_client_io_status_get(
        uint32_t cbox_id,
        const char_t* p_client_io_name,
        CLINK_MODBUS_CLIENT_IO_STATUS* p_status,
        clink_float_t* p_response_time_ms);

/**
@brief      Modbus 클라이언트 bit IO 값을 Read 한다.
            IO를 생성할때 등록한 address(서버 모드버스 맵의 주소 값)부터 num_of_val 개수 만큼 Read 하여 \n
            p_val(메모리)를 통해 반환한다.
@param[in]	p_client_io_name
            client_io 의 유니크한 이름
@param[in]  num_of_val
            Read 할 데이터 개수
@param[out] p_val
            읽어들인 데이터를 반환하기 위한 버퍼 포인터
@return     @ref clink_cbox_modbus_client_bit_io_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_modbus_client_bit_io_get(
        uint32_t cbox_id,
        const char_t* p_client_io_name,
        uint32_t num_of_val,
        uint8_t* p_val);

/**
@brief      Ext Modbus 클라이언트 bit IO 값을 Read 한다.
            IO를 생성할때 등록한 address(서버 모드버스 맵의 주소 값)부터 num_of_val 개수 만큼 Read 하여 \n
            p_val(메모리)를 통해 반환한다.
@param[in]	p_client_io_name
            client_io 의 유니크한 이름
@param[in]  num_of_val
            Read 할 데이터 개수
@param[out] p_val
            읽어들인 데이터를 반환하기 위한 버퍼 포인터
@return     @ref clink_cbox_ext_modbus_client_bit_io_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_modbus_client_bit_io_get(
        uint32_t cbox_id,
        const char_t* p_client_io_name,
        uint32_t num_of_val,
        uint8_t* p_val);

/**
@brief      Modbus 클라이언트 bit IO를 이용하여 값을 Write 한다.
            IO를 생성할때 등록한 address(서버 모드버스 맵의 주소 값)부터 num_of_val 개수 만큼 \n
            p_val(메모리)의 데이터를 Write 한다.
@param[in]	p_client_io_name
            client_io 의 유니크한 이름
@param[in]  num_of_val
            Write 할 데이터 개수
@param[in]  p_val
            Write 할 데이터를 담고 있는 버퍼 포인터
@return     @ref clink_cbox_ext_modbus_client_bit_io_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_modbus_client_bit_io_set(
        uint32_t cbox_id,
        const char_t* p_client_io_name,
        uint32_t num_of_val,
        const uint8_t* p_val);

/**
@brief      Modbus 클라이언트 register IO를 이용하여 값을 Read 한다.
            IO를 생성할때 등록한 address(서버 모드버스 맵의 주소 값)부터 num_of_val 개수 만큼 Read 하여 \n
            p_val(메모리)를 통해 반환한다.
@param[in]	p_client_io_name
            client_io 의 유니크한 이름
@param[in]  num_of_val
            Read 할 데이터 개수
@param[out] p_val
            읽어들인 데이터를 반환하기 위한 버퍼 포인터
@return     @ref clink_cbox_modbus_client_register_io_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_modbus_client_register_io_get(
        uint32_t cbox_id,
        const char_t* p_client_io_name,
        uint32_t num_of_val,
        uint16_t* p_val);

/**
@brief      Ext Modbus 클라이언트 register IO를 이용하여 값을 Read 한다.
            IO를 생성할때 등록한 address(서버 모드버스 맵의 주소 값)부터 num_of_val 개수 만큼 Read 하여 \n
            p_val(메모리)를 통해 반환한다.
@param[in]	p_client_io_name
            client_io 의 유니크한 이름
@param[in]  num_of_val
            Read 할 데이터 개수
@param[out] p_val
            읽어들인 데이터를 반환하기 위한 버퍼 포인터
@return     @ref clink_cbox_ext_modbus_client_register_io_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_modbus_client_register_io_get(
        uint32_t cbox_id,
        const char_t* p_client_io_name,
        uint32_t num_of_val,
        uint16_t* p_val);

/**
@brief      Modbus 클라이언트 register IO를 이용하여 값을 Write 한다.
            IO를 생성할때 등록한 address(서버 모드버스 맵의 주소 값)부터 num_of_val 개수 만큼 \n
            p_val(메모리)의 데이터를 Write 한다.
@param[in]	p_client_io_name
            client_io 의 유니크한 이름
@param[in]  num_of_val
            Write 할 데이터 개수
@param[in]  p_val
            Write 할 데이터를 담고 있는 버퍼 포인터
@return     @ref clink_cbox_ext_modbus_client_register_io_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_modbus_client_register_io_set(
        uint32_t cbox_id,
        const char_t* p_client_io_name,
        uint32_t num_of_val,
        const uint16_t* p_val);

/**
@brief      manual_control 동작 여부를 UI에서 제어쪽으로 알려준다
@param[in]	state_manual_control
            manual_control 상태 값
@return     @ref clink_cbox_state_manual_control_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_state_manual_control_set(
        uint32_t cbox_id,
        CLINK_SWITCH state_manual_control);

/**
@brief      ui_program 상태 정보를 UI에서 제어쪽으로 알려준다
@param[in]	state_ui_program
            ui_program 상태 값
@return     @ref clink_cbox_state_ui_program_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_state_ui_program_set(
        uint32_t cbox_id,
        CLINK_UI_PROGRAM_STATE state_ui_program);

/**
@brief      Ethernet/IP adaptor를 시작한다.
@param[in]  p_my_ip
            현재 이 API 를 사용하는 PC 의 NIC 의 IP 주소
@param[in]  port_num
             소켓에 사용할 포트 번호. 0 을 입력하는 경우 임의로 할당
@return     @ref clink_cbox_ext_eip_adaptor_start_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_eip_adaptor_start(
        uint32_t cbox_id,
        const char_t* p_my_ip,
        uint16_t port_num);

/**
@brief      Ethernet/IP adaptor 종료한다.
@return     @ref clink_cbox_ext_eip_adaptor_stop_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_eip_adaptor_stop(
        uint32_t cbox_id);

/**
@brief      Ethernet/IP adaptor 상태 정보를 반환한다.
@param[out] p_io_led_state
            EIP 상태 값
@return     @ref clink_cbox_ext_eip_adaptor_status_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_eip_adaptor_status(
        uint32_t cbox_id,
        CLINK_LED_STATE* p_io_led_state);

/**
@brief      Ethernet/IP scanner를 시작한다.
@param[in]  p_target_ip
            접속할 Target IOD 의 IP 주소
@param[in]  port_num
             소켓에 사용할 포트 번호. 0 을 입력하는 경우 임의로 할당
@param[in]  iod_model
             접속할 Target IOD 의 Model
@return     @ref clink_cbox_ext_eip_scanner_start_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_eip_scanner_start(
        uint32_t cbox_id,
        const char_t* p_target_ip,
        uint16_t port_num,
        CLINK_EIP_IO_DEVICE_MODEL iod_model);

/**
@brief      Ethernet/IP scanner 종료한다.
@return     @ref clink_cbox_ext_eip_scanner_stop_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_eip_scanner_stop(
        uint32_t cbox_id);

/**
@brief      Ethernet/IP scanner 상태 정보를 반환한다.
@param[out] p_io_led_state
            EIP 상태 값
@return     @ref clink_cbox_ext_eip_scanner_status_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_eip_scanner_status(
        uint32_t cbox_id,
        CLINK_LED_STATE* p_io_led_state);

/**
@brief      Ethernet/IP scanner 의 RX 데이터를 가져온다.
@param[in]  size_of_data
            출력 값 저장 버퍼 사이즈를 입력한다.
@param[out] p_val
            출력 값 저장 버퍼 포인터
@return     @ref clink_cbox_ext_eip_scanner_rx_data_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_eip_scanner_rx_data_get(
        uint32_t cbox_id,
        uint32_t size_of_data,
        uint8_t* p_val);

/**
@brief      Ethernet/IP scanner 의 TX 데이터를 가져온다.
@param[in]  size_of_data
            출력 값 저장 버퍼 사이즈를 입력한다.
@param[out] p_val
            읽어온 값을 저장할 버퍼 포인터
@return     @ref clink_cbox_ext_eip_scanner_tx_data_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_eip_scanner_tx_data_get(
        uint32_t cbox_id,
        uint32_t size_of_data,
        uint8_t* p_val);

/**
@brief      Ethernet/IP scanner 의 TX 데이터를 설정한다.
@param[in]  size_of_data
            입력할 값을 가지고 있는 버퍼 사이즈를 입력한다.
@param[in]  p_val
            입력할 값을 가지고 있는 버퍼 포인터
@return     @ref clink_cbox_ext_eip_scanner_tx_data_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_eip_scanner_tx_data_set(
        uint32_t cbox_id,
        uint32_t size_of_data,
        const uint8_t* p_val);

/**
@brief      User memory output bit register 값을 Write 한다.
@param[in]  address
            bit register의 address
@param[in]  val
            Write 할 bit register 값
@return     @ref clink_cbox_ext_output_bit_register_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_output_bit_register_set(
        uint32_t cbox_id,
        uint16_t address,
        uint32_t val);

/**
@brief      User memory output int register 값을 Write 한다.
@param[in]  address
            int register의 address
@param[in]  val
            Write 할 int register 값
@return     @ref clink_cbox_ext_output_int_register_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_output_int_register_set(
        uint32_t cbox_id,
        uint16_t address,
        uint32_t val);

/**
@brief      User memory output float register 값을 Write 한다.
@param[in]  address
            float register의 address
@param[in]  val
            Write 할 float register 값
@return     @ref clink_cbox_ext_output_float_register_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_output_float_register_set(
        uint32_t cbox_id,
        uint16_t address,
        clink_float_t val);

/**
@brief      User memory output bit register 값을 Read 한다.
@param[in]  address
            bit register의 address
@param[out] p_val
            bit register 값
@return     @ref clink_cbox_ext_output_bit_register_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_output_bit_register_get(
        uint32_t cbox_id,
        uint16_t address,
        uint32_t* p_val);

/**
@brief      User memory output int register 값을 Read 한다.
@param[in]  address
            int register의 address
@param[out] p_val
            int register 값
@return     @ref clink_cbox_ext_output_int_register_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_output_int_register_get(
        uint32_t cbox_id,
        uint16_t address,
        uint32_t* p_val);

/**
@brief      User memory output float register 값을 Read 한다.
@param[in]  address
            float register의 address
@param[out] p_val
            float register 값
@return     @ref clink_cbox_ext_output_float_register_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_output_float_register_get(
        uint32_t cbox_id,
        uint16_t address,
        clink_float_t* p_val);

/**
@brief      User memory input bit register 값을 Read 한다.
@param[in]  address
            bit register의 address
@param[out] p_val
            bit register 값
@return     @ref clink_cbox_ext_input_bit_register_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_input_bit_register_get(
        uint32_t cbox_id,
        uint16_t address,
        uint32_t* p_val);

/**
@brief      User memory input int register 값을 Read 한다.
@param[in]  address
            int register의 address
@param[out] p_val
            int register 값
@return     @ref clink_cbox_ext_input_int_register_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_input_int_register_get(
        uint32_t cbox_id,
        uint16_t address,
        uint32_t* p_val);

/**
@brief      User memory input float register 값을 Read 한다.
@param[in]  address
            float register의 address
@param[out] p_val
            float register 값
@return     @ref clink_cbox_ext_input_float_register_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_cbox_ext_input_float_register_get(
        uint32_t cbox_id,
        uint16_t address,
        clink_float_t* p_val);

/** @} */

//------------------------------------------------------------------
// 300_ROBOT
//------------------------------------------------------------------
/** \addtogroup 300_ROBOT 300_ROBOT
*  @{
*/

/**
@brief      다른 프로세스에서 로봇에 관한 정보를 읽어올 경우 해당 함수 호출이 필요
@param[in]  p_config_file_path
            설정 파일의 path
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID \n
            즉, clink_robot_create() 를 통해 반환 받은 값
@param[in]  robot_model_name
            로봇 모델 명, CLINK_ROBOT_MODEL 참조
@return     @ref clink_robot_info_initialze_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_info_initialze(
        uint32_t cbox_id,
        const char_t* p_config_file_path,
        uint32_t robot_id,
        CLINK_ROBOT_MODEL robot_model_name);

/**
@brief      제어할 로봇의 인스턴스를 생성한다.
@param[in]  robot_model_name
            로봇 모델 명, CLINK_ROBOT_MODEL 참조
@param[in]  p_config_file_path
            설정 파일의 full path
@param[in]  first_motor_index
            Control Box 에 연결된 모터 중, 해당 로봇의 \n
            첫 번째 축에 대응하는 모터의 index \n
            로봇이 1대 연결된 경우, 일반적으로 0 으로 설정된다.
@param[out] p_robot_id
            로봇 인스턴스의 ID
@return     @ref clink_robot_create_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_create(
        uint32_t cbox_id,
        CLINK_ROBOT_MODEL robot_model_name,
        const char_t* p_config_file_path,
        uint32_t first_motor_index,
        uint32_t* p_robot_id);

/**
@brief      로봇 인스턴스에 할당된 메모리를 해제한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@return     @ref clink_robot_destroy_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_destroy(
        uint32_t cbox_id,
        uint32_t robot_id);

/**
@brief      로봇의 모든 상태를 초기화 한다. \n
            Error 가 있는 경우, Error Clear 를 시도한다. \n
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@return     @ref clink_robot_reset_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_reset(
        uint32_t cbox_id,
        uint32_t robot_id);

/**
@brief      로봇 모델명을 반환한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_model_name
            로봇 모델명
@return     @ref clink_robot_model_name_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_model_name_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_ROBOT_MODEL* p_model_name);

/**
@brief      로봇의 Joint 개수를 반환한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_number_of_joints
            로봇의 Joint 개수
@return     @ref clink_robot_number_of_joints_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_number_of_joints_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t* p_number_of_joints);

/**
@brief      현재 로봇을 정지한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  stop_time
            단위 : 초 (sec) \n
            명령 지령 시점부터 command velocity 가 0 가 될때까지의 시간. \n
@remark     일반적으로 0.5 초 이하를 설정한다. \n
            clink_rpc_robot_stop() 실행 후, 해당 로봇에 \n
            지령되었던 모든 motion command 는 삭제된다.     
@return     @ref clink_robot_stop_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_stop(
        uint32_t cbox_id,
        uint32_t robot_id, 
        clink_float_t stop_time);

/**
@brief      로봇의 모든 모션을 종료한다. \n
            기본적인 동작 원리는 clink_robot_stop() \n
            와 동일하다. 단, stop time 은 \n
            자동적으로 (즉, 속도가 느린 경우 짧은 시간, \n
            속도가 빠른 경우 긴 시간) 설정한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@return     @ref clink_robot_stop_auto_time_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_stop_auto_time(
        uint32_t cbox_id,
        uint32_t robot_id);

/**
@brief      현재 로봇을 응급정지 한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  stop_time
            단위 : 초 (sec) \n
            명령 지령 시점부터 command velocity 가 0 가 될때까지의 시간. \n
@remark     clink_robot_stop() 의 경우, 로봇은 joint 모션을 실행중에는 \n
            각 joint 를 멈추고, tcp 모션을 실행중에는 tcp 수준에서 정지한다. \n
            그리고, 설정된 velocity profiler 를 활용하여 stop 한다. \n
            clink_rpc_robot_emergency_stop() 의 경우에는 현재 실행중인 \n
            모든 모션을 무시하고, 현재 각 축의 velocity 를 설정된 \n
            stop_time 시간내에 모두 0 가 되게끔 속도를 profiling 하여 \n
            로봇을 정지시킨다.
@return     @ref clink_robot_emergency_stop_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_emergency_stop(
        uint32_t cbox_id,
        uint32_t robot_id, 
        clink_float_t stop_time);

/**
@brief      현재 로봇의 모션을 잠시 멈춘다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  stop_time
            단위 : 초 (sec) \n
            명령 지령 시점부터 command velocity 가 0 가 될때까지의 시간. \n
@remark     일반적으로 0.5 초 이하를 설정한다. \n
            clink_rpc_robot_resume() 실행 시, 해당 motion command 를 재실행한다. \n
            이전 지령된 motion command 는 삭제되지 않는다.
@return     @ref clink_robot_pause_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_pause(
        uint32_t cbox_id,
        uint32_t robot_id, 
        clink_float_t stop_time);

/**
@brief      clink_robot_pause() 와 동일하나, 시간을 자동으로 설정한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@return     @ref clink_robot_pause_auto_time_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_pause_auto_time(
        uint32_t cbox_id,
        uint32_t robot_id);

/**
@brief      clink_robot_pause() 를 통해 일시 정지한 motion command 를 \n
            재실행 한다. 
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@return     @ref clink_robot_resume_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_resume(
        uint32_t cbox_id,
        uint32_t robot_id);

/**
@brief      로봇 모션 이동 상태를 반환한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_state
            CLINK_MOVING_STATE 참조.
@return     @ref clink_robot_state_moving_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_state_moving_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_MOVING_STATE* p_state);

/**
@brief      로봇 Safety Limit 준수  상태를 반환한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_state
            CLINK_ROBOT_SAFETY_STATE_SAFE : 로봇은 어떤 safety limit 도 위반하지 않은 상태 \n
            CLINK_ROBOT_SAFETY_STATE_NOT_SAFE : 로봇은 어떤 safety limit 을 위반한 상태
@return     @ref clink_robot_state_safety_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_state_safety_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_ROBOT_SAFETY_STATE* p_state);

/**
@brief      현재 로봇에서 동작 중인 모션 커맨드 이름을 반환한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_motion_name
            현재 로봇에서 동작 중인 모션 커맨드 이름
@return     @ref clink_robot_current_motion_name_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_current_motion_name_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        char_t* p_motion_name);

/**
@brief      현재 로봇에 설정된 Servo On / Off 상태를 반환한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_on_off
            CLINK_SWITCH_ON : Servo On  상태 \n
            CLINK_SWITCH_OFF : Servo Off 상태
@return     @ref clink_robot_servo_switch_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_servo_switch_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH* p_on_off);

/**
@brief      로봇의 Servo On / Off 를 설정한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  on_off
            CLINK_SWITCH_ON : Servo On \n
            CLINK_SWITCH_OFF : Servo Off
@return     @ref clink_robot_servo_switch_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_servo_switch_set(
        uint32_t cbox_id,
        uint32_t robot_id, 
        CLINK_SWITCH on_off);

/**
@brief      현재 설정된 로봇의 speed factor 를 반환한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_speed_factor
            설정된 speed factor 값
@return     @ref clink_robot_speed_factor_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_speed_factor_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t* p_speed_factor);

/**
@brief      로봇에 실행하는 motion command 의 speed factor 를 설정한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  speed_factor 
            최소 값 : 0.1 \n
            최대 값 : 1.0 \n
@remark     최대 값 (1.0) 으로 설정된 경우, 로봇은 motion command 에 \n
            설정된 max speed 값으로 이동한다. \n
            만약 0.5 가 설정되면, 절반의 속도로 이동한다.\n
            즉, 새로운 max speed 는 max speed X speed_factor \n
            의 값으로 설정된다.
@return     @ref clink_robot_speed_factor_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_speed_factor_set(
        uint32_t cbox_id,
        uint32_t robot_id, 
        clink_float_t speed_factor);

/**
@brief      로봇 speed factor 변경 시, 가감속 자동 조절 여부 설정 값을 반환한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_adjust
            CLINK_SWITCH_ON: speed factor 비율만큼 속도와 마찬가지로 가감속도를 수정한다. \n
            CLINK_SWITCH_OFF: 속도만 변경, 가감속은 입력값 그대로 실행 (default)
@return     @ref clink_robot_acc_dec_auto_adjustment_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_acc_dec_auto_adjustment_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH* p_adjust);

/**
@brief      로봇 speed factor 변경 시, 가감속 자동 조절 여부를 설정한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  adjust
            CLINK_SWITCH_ON: speed factor 비율만큼 속도와 마찬가지로 가감속도를 수정한다. \n
            CLINK_SWITCH_OFF: 속도만 변경, 가감속은 입력값 그대로 실행 (default)
@return     @ref clink_robot_acc_dec_auto_adjustment_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_acc_dec_auto_adjustment_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH adjust);

/**
@brief      현재 설정된 로봇의 설치면 (바닥, 벽, 천장) \n
            및 설치 각도 (tilt) 를 반환한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_tilt_x
            설치 tilt 각도 X
@param[out] p_tilt_y
            설치 tilt 각도 Y
@return     @ref clink_robot_installation_mount_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_installation_mount_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t* p_tilt_x,
        clink_float_t* p_tilt_y);

/**
@brief      로봇의 설치면 (바닥, 벽, 천장) \n
            및 설치 각도 (tilt) 를 설정한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  tilt_x
            설치 tilt 각도 X
@param[in]  tilt_y
            설치 tilt 각도 Y
@remark     잘못 설정하는 경우, 충돌감지 및 직접교시가 \n
            정확히 동작하지 않는다.
@return     @ref clink_robot_installation_mount_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_installation_mount_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t tilt_x,
        clink_float_t tilt_y);

/**
@brief      로봇의 모든 조인트의 홈 위치를 설정한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@return     @ref clink_robot_home_position_reset_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_home_position_reset(
        uint32_t cbox_id,
        uint32_t robot_id);

/**
@brief      지정한 조인트의 홈 위치를 설정한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  joint_index
            해당 하는 Joint 의 index \n
            6축 로봇의 경우, 첫 번째 축은 0 \n
            마지막 축은 5
@return     @ref clink_robot_home_joint_position_reset_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_home_joint_position_reset(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t joint_index);

/**
@brief      clink_robot_motion_auto_adjust_swith_set() 으로 설정한 값을 리턴한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_on_off
            CLINK_SWITCH_ON : 기능 활성화 (자동조정 실행) \n
            CLINK_SWITCH_OFF : 기능 비활성화 (자동조정 실행하지 않음)
@return     @ref clink_robot_motion_auto_adjust_swith_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_motion_auto_adjust_swith_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH* p_on_off);

/**
@brief      실행하는 모션의 파라메터를 에러가 발생하지 않게끔 자동으로 조정하는 기능을 설정한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  on_off
            CLINK_SWITCH_ON : 기능 활성화 (자동조정 실행) \n
            CLINK_SWITCH_OFF : 기능 비활성화 (자동조정 실행하지 않음)
@return     @ref clink_robot_motion_auto_adjust_swith_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_motion_auto_adjust_swith_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH on_off);

/**
@brief      로봇의 모든 조인트가 본래 Home Reset 을 실행하는 각도를 \n
            유지한다는 가정하에 자동으로 리셋을 실행한다. 
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@remark     Not Supported
@return     @ref clink_robot_home_position_auto_reset_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_home_position_auto_reset(
        uint32_t cbox_id,
        uint32_t robot_id);

/**
@brief		로봇 Kinematics의 DH Parameter값을 받아온다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  num_of_joints
            로봇 Joint의 갯수 (배열의 크기)
@param[out]	p_param_a
            DH Paramter: a 배열 (단위 : mm)
@param[out]	 p_param_alpha
            DH Paramter: alpha 배열 (단위 : degree)
@param[out]	p_param_d
            DH Paramter: d 배열 (단위 : mm)
@param[out]	p_param_theta
            DH Paramter: theta 배열 (단위 : degree)
@remark		각 배열은 이 함수를 호출할 때 num_of_joints 이상 할당해야 한다.
@return     @ref clink_robot_kinematics_dh_parameter_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_kinematics_dh_parameter_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        int32_t num_of_joints,
        clink_float_t* p_param_a,
        clink_float_t* p_param_alpha,
        clink_float_t* p_param_d,
        clink_float_t* p_param_theta);

/**
@brief		Robot Kinematics의 DH Parameter를 설정한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  num_of_joints
            로봇 Joint의 갯수 (배열의 크기)
@param[in]  p_param_a
            DH Paramter: a 배열 (단위 : mm)
@param[in]  p_param_alpha
            DH Paramter: alpha 배열 (단위 : degree)
@param[in]  p_param_d
            DH Paramter: d 배열 (단위 : mm)
@param[in]  p_param_theta
            DH Paramter: theta 배열 (단위 : degree)
@remark     Servo Off 상태에서 호출돼야 한다.
@return     @ref clink_robot_kinematics_dh_parameter_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_kinematics_dh_parameter_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t num_of_joints,
        const clink_float_t* p_param_a,
        const clink_float_t* p_param_alpha,
        const clink_float_t* p_param_d,
        const clink_float_t* p_param_theta);

/**
@brief      로봇의 control 모드 상태를 받아온다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_mode
            로봇 모터 컨트롤 모드
@return     @ref clink_robot_control_mode_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_control_mode_get(
        uint32_t cbox_id,
	    uint32_t robot_id,
	    CLINK_DRIVE_CONTROL_MODE* p_mode);

/**
@brief      로봇의 velocity control 모드를 설정한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  mode
            로봇 모터 컨트롤 모드
@return     @ref clink_robot_control_mode_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_control_mode_set(
        uint32_t cbox_id,
	    uint32_t robot_id,
        CLINK_DRIVE_CONTROL_MODE mode);

/**
@brief      로봇의 velocity control 모드 상태를 받아온다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_on_off
            CLINK_SWITCH_ON : velocity mode 실행 \n
            CLINK_SWITCH_OFF : velocity mode 종료
@return     @ref clink_robot_velocity_control_switch_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_velocity_control_switch_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH* p_on_off);

/**
@brief      로봇의 velocity control 모드를 설정한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  on_off
            CLINK_SWITCH_ON : velocity mode 실행 \n
            CLINK_SWITCH_OFF : velocity mode 종료
@return     @ref clink_robot_velocity_control_switch_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_velocity_control_switch_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH on_off);

/**
@brief      velocity mode에서 사용되는 목표 속도를 받아온다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  joint_index
            해당 하는 Joint 의 index \n
            6축 로봇의 경우, 첫 번째 축은 0 \n
            마지막 축은 5 
@param[out] p_vel
            현재 지정된 목표 속도
@return     @ref clink_robot_joint_target_velocity_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_joint_target_velocity_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t joint_index,
        clink_float_t* p_vel);

/**
@brief      velocity mode에서 사용되는 목표 속도를 설정한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  joint_index
            해당 하는 Joint 의 index \n
            6축 로봇의 경우, 첫 번째 축은 0 \n
            마지막 축은 5
@param[in]  vel
            목표 속도
@return     @ref clink_robot_joint_target_velocity_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_joint_target_velocity_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t joint_index,
        clink_float_t vel);

/**
@brief      velocity mode에서 사용되는 목표 가속도를 받아온다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  joint_index
            해당 하는 Joint 의 index \n
            6축 로봇의 경우, 첫 번째 축은 0 \n
            마지막 축은 5
@param[out] p_acc
            현재 지정된 목표 가속도
@return     @ref clink_robot_joint_target_acceleration_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_joint_target_acceleration_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t joint_index,
        clink_float_t* p_acc);

/**
@brief      velocity mode에서 사용되는 목표 가속도를 설정한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  joint_index
            해당 하는 Joint 의 index \n
            6축 로봇의 경우, 첫 번째 축은 0 \n
            마지막 축은 5
@param[in]  acc
            목표 가속도
@return     @ref clink_robot_joint_target_acceleration_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_joint_target_acceleration_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t joint_index,
        clink_float_t acc);

/**
@brief      velocity mode에서 사용되는 목표 속도(RPM)를 받아온다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  joint_index
            해당 하는 Joint 의 index \n
            6축 로봇의 경우, 첫 번째 축은 0 \n
            마지막 축은 5
@param[out] p_vel
            현재 지정된 목표 속도
@return     @ref clink_robot_joint_target_velocity_rpm_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_joint_target_velocity_rpm_get(
        uint32_t cbox_id,
	    uint32_t robot_id,
	    uint32_t joint_index,
	    clink_float_t* p_vel);

/**
@brief      velocity mode에서 사용되는 목표 속도(RPM)를 설정한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  joint_index
            해당 하는 Joint 의 index \n
            6축 로봇의 경우, 첫 번째 축은 0 \n
            마지막 축은 5
@param[in]  vel
            목표 속도
@return     @ref clink_robot_joint_target_velocity_rpm_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_joint_target_velocity_rpm_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t joint_index,
        clink_float_t vel);

/**
@brief      velocity mode에서 사용되는 목표 가속도(RPM)를 받아온다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  joint_index
            해당 하는 Joint 의 index \n
            6축 로봇의 경우, 첫 번째 축은 0 \n
            마지막 축은 5
@param[out] p_acc
            현재 지정된 목표 가속도
@return     @ref clink_robot_joint_target_acceleration_rpm_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_joint_target_acceleration_rpm_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t joint_index,
        clink_float_t* p_acc);

/**
@brief      velocity mode에서 사용되는 목표 가속도(RPM)를 설정한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  joint_index
            해당 하는 Joint 의 index \n
            6축 로봇의 경우, 첫 번째 축은 0 \n
            마지막 축은 5
@param[in]  acc
            목표 가속도
@return     @ref clink_robot_joint_target_acceleration_rpm_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_joint_target_acceleration_rpm_set(
        uint32_t cbox_id,
	    uint32_t robot_id,
	    uint32_t joint_index,
	    clink_float_t acc);

/**
@brief      로봇의 joint pos chaser 상태를 받아온다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_on_off
            CLINK_SWITCH_ON : joint pos chaser 모드 실행 중 \n
			CLINK_SWITCH_OFF : joint pos chaser 모드 실행 중 아님
@return     @ref clink_robot_joint_chaser_state_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_joint_chaser_state_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH* p_on_off);

/**
@brief      로봇의 joint pos chaser 상태를 설정한다. \n
            직접적인 호출은 금지한다. (remote 전용 기능)
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  on_off
            CLINK_SWITCH_ON : joint pos chaser 실행 \n
            CLINK_SWITCH_OFF : joint pos chaser 종료
@return     @ref clink_robot_joint_chaser_state_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_joint_chaser_state_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH on_off);
/** @} */


//------------------------------------------------------------------
// 301_ROBOT_SPEC
//------------------------------------------------------------------
/** \addtogroup 301_ROBOT_SPEC 301_ROBOT_SPEC
*  @{
*/
/**
@brief      로봇의 specification 상의 해당 Joint 의 최대 가속도를 반환한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  joint_index
            해당 하는 Joint 의 index \n
            6축 로봇의 경우, 첫 번째 축은 0 \n
            마지막 축은 5 
@param[out] p_acc_max
            최대 가속도 값 (단위 : degree / second^2)
@remark     로봇의 spec 은 사용자가 설정하는 값이 아니고 \n
            로봇의 설계상의 spec 이다. 따라서 별도로 set \n
            함수를 제공하지 않는다.
@return     @ref clink_robot_spec_joint_acc_max_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_spec_joint_acc_max_get(
        uint32_t cbox_id,
        uint32_t robot_id, 
        uint32_t joint_index,
        clink_float_t* p_acc_max);

/**
@brief      로봇의 specification 상의 해당 Joint 의 최대 속도를 반환한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  joint_index
            해당 하는 Joint 의 index \n
            6축 로봇의 경우, 첫 번째 축은 0 \n
            마지막 축은 5 
@param[out] p_speed_max
            최대 속도 값 (단위 : degree / second)
@remark     로봇의 spec 은 사용자가 설정하는 값이 아니고 \n
            로봇의 설계상의 spec 이다. 따라서 별도로 set \n
            함수를 제공하지 않는다.
@return     @ref clink_robot_spec_joint_speed_max_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_spec_joint_speed_max_get(
        uint32_t cbox_id,
        uint32_t robot_id, 
        uint32_t joint_index,
        clink_float_t* p_speed_max);

/**
@brief      로봇의 specification 상의 해당 Joint 의 구동 범위 \n
            최대, 최소 각도를 반환한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  joint_index
            해당 하는 Joint 의 index \n
            6축 로봇의 경우, 첫 번째 축은 0 \n
            마지막 축은 5 
@param[out] p_angle_min
            최소 각도 값 (단위 : degree)
@param[out] p_angle_max
            최대 각도 값 (단위 : degree)
@remark     로봇의 spec 은 사용자가 설정하는 값이 아니고 \n
            로봇의 설계상의 spec 이다. 따라서 별도로 set \n
            함수를 제공하지 않는다.
@return     @ref clink_robot_spec_joint_angle_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_spec_joint_angle_get(
        uint32_t cbox_id,
        uint32_t robot_id, 
        uint32_t joint_index,
        clink_float_t* p_angle_min,
        clink_float_t* p_angle_max);

/**
@brief      로봇의 specification 상의 TCP 의 최대 가속도를 반환한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_acc_max
            최대 가속도 값 (단위 : millimeter / second^2)
@remark     로봇의 spec 은 사용자가 설정하는 값이 아니고 \n
            로봇의 설계상의 spec 이다. 따라서 별도로 set \n
            함수를 제공하지 않는다.
@return     @ref clink_robot_spec_tcp_acc_max_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_spec_tcp_acc_max_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t* p_acc_max);

/**
@brief      로봇의 specification 상의 TCP 의 최대 속도를 반환한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_speed_max
            최대 속도 값 (단위 : millimeter / second)
@remark     로봇의 spec 은 사용자가 설정하는 값이 아니고 \n
            로봇의 설계상의 spec 이다. 따라서 별도로 set \n
            함수를 제공하지 않는다.
@return     @ref clink_robot_spec_tcp_speed_max_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_spec_tcp_speed_max_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t* p_speed_max);
  
/** @} */


//------------------------------------------------------------------
// 302_ROBOT_TCP
//------------------------------------------------------------------
/** \addtogroup 302_ROBOT_TCP 302_ROBOT_TCP
*  @{
*/

/**
@brief      로봇의 Flange to TCP 관계에 설정된 값을 반환한다. 
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_pos_x
            Flange to TCP position X[mm]
@param[out] p_pos_y
            Flange to TCP position Y[mm]
@param[out] p_pos_z
            Flange to TCP position Z[mm]
@param[out] p_ort_x
            Flange to TCP orientation X[degree]
@param[out] p_ort_y
            Flange to TCP orientation Y[degree]
@param[out] p_ort_z
            Flange to TCP orientation Z[degree]
@remark     본 함수의 모든 데이터는 Base 좌표계 기준이다. \n
            <STRONG>(즉, TCP 좌표계 아님)</STRONG>
@return     @ref clink_robot_tcp_from_flange_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_tcp_from_flange_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t* p_pos_x, 
        clink_float_t* p_pos_y, 
        clink_float_t* p_pos_z,
        clink_float_t* p_ort_x, 
        clink_float_t* p_ort_y, 
        clink_float_t* p_ort_z);

/**
@brief      로봇의 Flange to TCP 관계를 정의한다. \n
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  pos_x
            Flange to TCP position X[mm]
@param[in]  pos_y
            Flange to TCP position Y[mm]
@param[in]  pos_z
            Flange to TCP position Z[mm]
@param[in]  ort_x
            Flange to TCP orientation X[degree]
@param[in]  ort_y
            Flange to TCP orientation Y[degree]
@param[in]  ort_z
            Flange to TCP orientation Z[degree]
@remark     Flange to TCP 관계라 함은 Flange 로부터 \n
            TCP 종단까지의 거리 및 방향을 의미한다. \n
            본 함수를 통해 TCP 를 별도로 셋팅하지 않은 경우 \n
            TCP 의 위치는 Flange 와 동일한 것으로 간주한다. \n
            <STRONG>즉, 모든 TCP 관련 함수는 Flange 의 위치를 \n
            TCP 의 위치로 간주한다. </STRONG> \n
            TCP 설정 후, TCP 를 삭제하고 싶은 경우에는 \n
            (즉, 다시 Flange 의 위치를 TCP 의 위치로 사용하고 싶은 경우) \n
            모든 parameter (즉, pos_x ... pos_z, ort_x ... ort_z) 를 \n
            0 으로 설정한다. \n
            본 함수의 모든 데이터는 Base 좌표계 기준이다. \n
            <STRONG>(즉, TCP 좌표계 아님)</STRONG>
@return     @ref clink_robot_tcp_from_flange_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_tcp_from_flange_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t pos_x, 
        clink_float_t pos_y, 
        clink_float_t pos_z,
        clink_float_t ort_x, 
        clink_float_t ort_y, 
        clink_float_t ort_z);

/**
@brief      Flange 끝에 달리는 TCP 의 길이 성분을 \n
            여러 각도에서의 pose 데이터를 입력하여 \n
            계산 및 적용한다. (3개 이상 pose 데이터 입력 필수)
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  input_size
            p_pos_array_x ... p_pos_array_z 및 \n
            p_ort_array_x ... p_ort_array_z 의 배열 중 \n
            valid 한 데이터의 사이즈 \n
            (즉, TCP 길이 및 방향 계산에 사용할 pose 의 개수)
@param[in]  p_pos_array_x
            Pose 데이터 중 위치의 X 성분 배열
@param[in]  p_pos_array_y
            Pose 데이터 중 위치의 Y 성분 배열
@param[in]  p_pos_array_z
            Pose 데이터 중 위치의 Z 성분 배열
@param[in]  p_ort_array_x
            Pose 데이터 중 방향의 X 성분 배열
@param[in]  p_ort_array_y
            Pose 데이터 중 방향의 Y 성분 배열
@param[in]  p_ort_array_z
            Pose 데이터 중 방향의 Z 성분 배열
@param[out] p_tool_pos
            계산 결과 tcp from flange position 성분
@remark     여기서의 pose 데이터는 명확히 말하면  \n
            Flange 의 Base 좌표계 기준의 pose 이다. \n
            즉, pose 데이터를 취득할 때, \n
            clink_rpc_robot_tcp_from_flange_set() 함수를 통해 \n
            모든 position 및 orientation 을 0 으로 설정한다. \n
            이 때는 Flange 와 TCP 는 동일시 된다. \n
            이후, 다양한 방향으로 로봇을 위치시킨 후, \n
            clink_rpc_robot_tcp_from_flange_get() 를 통해 \n
            데이터를 취득한다. 
@return     @ref clink_robot_tcp_from_flange_multiple_position_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_tcp_from_flange_multiple_position_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t input_size,
        const clink_float_t* p_pos_array_x, 
        const clink_float_t* p_pos_array_y,
        const clink_float_t* p_pos_array_z,
        const clink_float_t* p_ort_array_x,
        const clink_float_t* p_ort_array_y,
        const clink_float_t* p_ort_array_z,
        clink_float_t* p_tool_pos);

/**
@brief      Flange 끝에 달리는 TCP 의 방향 성분을 \n
            미리 지정되어있는 고정된 좌표계와 일치 시킨 후 \n
            적용한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  ort_x
            Base 좌표계 기준 방향 X 성분
@param[in]  ort_y
            Base 좌표계 기준 방향 Y 성분
@param[in]  ort_z
            Base 좌표계 기준 방향 Z 성분
@param[out] p_tool_ort
            계산 결과 tcp from flange orientation 성분
@remark     고정 좌표계는 다음과 같다. \n
            Base 좌표계 기준으로 -z축(즉, 아래방향)으로 TCP의 x축 정렬 \n
            Base 좌표계 기준으로 x축(즉, 앞쪽방향)으로 TCP의 z축 정렬 \n
            (y축은 x,z 축에 의해 결정) \n
            해당 API 적용 전, clink_robot_tcp_from_flange_set() 함수를 통해 \n
            모든 orientation 을 0 으로 설정한다. \n
            clink_rpc_robot_tcp_from_flange_get() 를 통해 \n
            계산된 데이터를 취득 가능하다. 
@return     @ref clink_robot_tcp_from_flange_multiple_orientation_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_tcp_from_flange_multiple_orientation_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t ort_x,
        clink_float_t ort_y,
        clink_float_t ort_z,
        clink_float_t* p_tool_ort);
 
/**
@brief      clink_robot_tcp_payload_set() 를 통해 설정된
            payload 를 반환한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_cur_payload
            설정된 Payload 무게 (단위 : kilogram)
@param[out] p_center_of_mass_x
            무게중심의 X 성분 (단위 : millimeter)
@param[out] p_center_of_mass_y
            무게중심의 Y 성분 (단위 : millimeter)
@param[out] p_center_of_mass_z
            무게중심의 Z 성분 (단위 : millimeter)
@return     @ref clink_robot_tcp_payload_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_tcp_payload_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t* p_cur_payload,
        clink_float_t* p_center_of_mass_x,
        clink_float_t* p_center_of_mass_y,
        clink_float_t* p_center_of_mass_z);

/**
@brief      현재 로봇에 부착된 TCP 를 포함한 payload 를 설정한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  cur_payload
            Payload 무게 (단위 : kilogram)
@param[in]  center_of_mass_x
            무게중심의 X 성분 (단위 : millimeter)
@param[in]  center_of_mass_y
            무게중심의 Y 성분 (단위 : millimeter)
@param[in]  center_of_mass_z
            무게중심의 Z 성분 (단위 : millimeter)
@return     @ref clink_robot_tcp_payload_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_tcp_payload_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t cur_payload,
        clink_float_t center_of_mass_x,
        clink_float_t center_of_mass_y,
        clink_float_t center_of_mass_z);

/**
@brief      로봇 TCP 의 현재 tcp command pose 데이터 (위치 및 방향) 을 반환한다. \n
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_pos_x
            위치 중 X 성분
@param[out] p_pos_y
            위치 중 Y 성분
@param[out] p_pos_z
            위치 중 Z 성분
@param[out] p_ort_x
            방향 중 X 성분
@param[out] p_ort_y
            방향 중 Y 성분
@param[out] p_ort_z
            방향 중 Z 성분
@return     @ref clink_robot_tcp_pose_command_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_tcp_pose_command_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t* p_pos_x,
        clink_float_t* p_pos_y,
        clink_float_t* p_pos_z,
        clink_float_t* p_ort_x,
        clink_float_t* p_ort_y,
        clink_float_t* p_ort_z);

/**
@brief      로봇 TCP 의 현재 tcp actual pose 데이터 (위치 및 방향) 을 반환한다. \n
            (내부적으로 각 Joint 의 actual angle 로 부터 forward kinematics 를 \n
            통해 계산한다.)
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_pos_x
            위치 중 X 성분
@param[out] p_pos_y
            위치 중 Y 성분
@param[out] p_pos_z
            위치 중 Z 성분
@param[out] p_ort_x
            방향 중 X 성분
@param[out] p_ort_y
            방향 중 Y 성분
@param[out] p_ort_z
            방향 중 Z 성분
@return     @ref clink_robot_tcp_pose_actual_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_tcp_pose_actual_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t* p_pos_x,
        clink_float_t* p_pos_y,
        clink_float_t* p_pos_z,
        clink_float_t* p_ort_x,
        clink_float_t* p_ort_y,
        clink_float_t* p_ort_z);

/**
@brief      로봇 TCP 의 현재 flange command pose 데이터 (위치 및 방향) 을 반환한다. \n
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_pos_x
            위치 중 X 성분
@param[out] p_pos_y
            위치 중 Y 성분
@param[out] p_pos_z
            위치 중 Z 성분
@param[out] p_ort_x
            방향 중 X 성분
@param[out] p_ort_y
            방향 중 Y 성분
@param[out] p_ort_z
            방향 중 Z 성분
@return     @ref clink_robot_flange_pose_command_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_flange_pose_command_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t* p_pos_x,
        clink_float_t* p_pos_y,
        clink_float_t* p_pos_z,
        clink_float_t* p_ort_x,
        clink_float_t* p_ort_y,
        clink_float_t* p_ort_z);

/**
@brief      로봇 TCP 의 현재 flange actual pose 데이터 (위치 및 방향) 을 반환한다. \n
            (내부적으로 각 Joint 의 actual angle 로 부터 forward kinematics 를 \n
            통해 계산한다.)
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_pos_x
            위치 중 X 성분
@param[out] p_pos_y
            위치 중 Y 성분
@param[out] p_pos_z
            위치 중 Z 성분
@param[out] p_ort_x
            방향 중 X 성분
@param[out] p_ort_y
            방향 중 Y 성분
@param[out] p_ort_z
            방향 중 Z 성분
@return     @ref clink_robot_flange_pose_actual_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_flange_pose_actual_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t* p_pos_x,
        clink_float_t* p_pos_y,
        clink_float_t* p_pos_z,
        clink_float_t* p_ort_x,
        clink_float_t* p_ort_y,
        clink_float_t* p_ort_z);

/**
@brief      로봇 TCP 의 현재 tcp command 위치 변화량 기반 선속도를 반환한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_pos_spd_x
            X축 속도
@param[out] p_pos_spd_y
            Y축 속도
@param[out] p_pos_spd_z
            Z축 속도
@return     @ref clink_robot_tcp_pos_speed_command_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_tcp_pos_speed_command_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t* p_pos_spd_x,
        clink_float_t* p_pos_spd_y,
        clink_float_t* p_pos_spd_z);

/**
@brief      로봇 TCP 의 현재 tcp command 방향 변화량 기반 축별 회전 속도를 반환한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_ort_spd_x
            X축 각속도
@param[out] p_ort_spd_y
            Y축 각속도
@param[out] p_ort_spd_z
            Z축 각속도
@return     @ref clink_robot_tcp_ort_speed_command_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_tcp_ort_speed_command_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t* p_ort_spd_x,
        clink_float_t* p_ort_spd_y,
        clink_float_t* p_ort_spd_z);

/**
@brief      로봇 TCP 의 현재 tcp actual 위치 변화량 기반 선속도를 반환한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_pos_spd_x
            X축 속도
@param[out] p_pos_spd_y
            Y축 속도
@param[out] p_pos_spd_z
            Z축 속도
@return     @ref clink_robot_tcp_pos_speed_actual_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_tcp_pos_speed_actual_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t* p_pos_spd_x,
        clink_float_t* p_pos_spd_y,
        clink_float_t* p_pos_spd_z);

/**
@brief      로봇 TCP 의 현재 tcp actual 방향 변화량 기반 축별 회전 속도를 반환한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_ort_spd_x
            X축 각속도
@param[out] p_ort_spd_y
            Y축 각속도
@param[out] p_ort_spd_z
            Z축 각속도
@return     @ref clink_robot_tcp_ort_speed_actual_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_tcp_ort_speed_actual_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t* p_ort_spd_x,
        clink_float_t* p_ort_spd_y,
        clink_float_t* p_ort_spd_z);

/**
@brief      주어진 TCP 의 orientation 에 해당하는 Quaternion을 계산하여 반환한다.
@param[in]  ort_x
            orientation 중 X 성분
@param[in]  ort_y
            orientation 중 Y 성분
@param[in]  ort_z
            orientation 중 Z 성분
@param[out] p_qtn_x
            Quaternion 중 X 성분
@param[out] p_qtn_y
            Quaternion 중 Y 성분
@param[out] p_qtn_z
            Quaternion 중 Z 성분
@param[out] p_qtn_w
            Quaternion 중 W 성분
@return     @ref clink_robot_tcp_quaternion_from_orientation_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_tcp_quaternion_from_orientation_get(
        uint32_t cbox_id,
        clink_float_t ort_x,
        clink_float_t ort_y,
        clink_float_t ort_z,
        clink_float_t* p_qtn_x,
        clink_float_t* p_qtn_y,
        clink_float_t* p_qtn_z,
        clink_float_t* p_qtn_w);

/**
@brief      주어진 Quaternion 에 해당하는 TCP 의 orientation을 계산하여 반환한다.
@param[in]  qtn_x
            Quaternion 중 X 성분
@param[in]  qtn_y
            Quaternion 중 Y 성분
@param[in]  qtn_z
            Quaternion 중 Z 성분
@param[in]  qtn_w
            Quaternion 중 W 성분
@param[out] p_ort_x
            orientation 중 X 성분
@param[out] p_ort_y
            orientation 중 Y 성분
@param[out] p_ort_z
            orientation 중 Z 성분
@return     @ref clink_robot_tcp_orientation_from_quaternion_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_tcp_orientation_from_quaternion_get(
        uint32_t cbox_id,
        clink_float_t qtn_x,
        clink_float_t qtn_y,
        clink_float_t qtn_z,
        clink_float_t qtn_w,
        clink_float_t* p_ort_x,
        clink_float_t* p_ort_y,
        clink_float_t* p_ort_z);

/**
@brief		원을 이루는 3 점과 n등분 할 파라미터를 입력하여 n등분된 원 위의 점들을 계산한다.
@param[in]	p_start_pos	    원을 이루는 Start Point (X, Y, Z)
@param[in]	p_mid_pos		원을 이루는 Mid Point (X, Y, Z)
@param[in]	p_end_pos		원을 이루는 End Point (X, Y, Z)
@param[in]	divided_num		원의 둘레를 등분하는 개수
@param[out]	p_point_pos	    원 위의 등분된 점 리스트 (X, Y, Z), ...
@remark		p_point_pos는 3 x(p_point_pos - 1) 만큼 크기를 할당한 후 주소값을 넘겨야 한다.
@return     @ref clink_robot_tcp_divide_circumference_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_tcp_divide_circumference_get(
        uint32_t cbox_id,
        const clink_float_t* p_start_pos,
        const clink_float_t* p_mid_pos,
        const clink_float_t* p_end_pos,
        int32_t divided_num,
        clink_float_t* p_point_pos);

/** @} */


//------------------------------------------------------------------
// 303_ROBOT_JOINT
//------------------------------------------------------------------
/** \addtogroup 303_ROBOT_JOINT 303_ROBOT_JOINT
*  @{
*/

/**
@brief      로봇 Joint 의 Software 버전을 반환한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  joint_index
            해당 하는 Joint 의 index \n
            6축 로봇의 경우, 첫 번째 축은 0 \n
            마지막 축은 5 
@return     로봇 Joint 의 Software 버전 \n
            (통상적으로 Drive 의 Firmware 버전)
*/
CLINK_EXPORT const char_t*
    clink_rpc_robot_joint_sw_version_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t joint_index);

/**
@brief      로봇 Joint 의 온도를 반환한다. \n
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  joint_index
            해당 하는 Joint 의 index \n
            6축 로봇의 경우, 첫 번째 축은 0 \n
            마지막 축은 5 
@param[out] p_temperature
            Joint 의 온도
@return     @ref clink_robot_joint_temperature_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_joint_temperature_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t joint_index,
        clink_float_t* p_temperature);

/**
@brief      로봇 Joint 의 error code를 반환한다. \n
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  joint_index
            해당 하는 Joint 의 index \n
            6축 로봇의 경우, 첫 번째 축은 0 \n
            마지막 축은 5
@param[out] p_error_code
            Error code
@param[out] p_error_status1
            Error status1
@param[out] p_error_status2
            Error status2
@param[out] p_error_status3
            Error status3
@return     @ref clink_robot_joint_error_code_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_joint_error_code_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t joint_index,
        uint16_t* p_error_code,
        uint16_t* p_error_status1,
        uint16_t* p_error_status2,
        uint16_t* p_error_status3);

/**
@brief      로봇 Joint 의 전압을 반환한다. \n
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  joint_index
            해당 하는 Joint 의 index \n
            6축 로봇의 경우, 첫 번째 축은 0 \n
            마지막 축은 5 
@param[out] p_voltage
            전압
@return     @ref clink_robot_joint_voltage_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_joint_voltage_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t joint_index,
        clink_float_t* p_voltage);

/**
@brief      로봇 Joint 의 전류를 반환한다. \n
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  joint_index
            해당 하는 Joint 의 index \n
            6축 로봇의 경우, 첫 번째 축은 0 \n
            마지막 축은 5 
@param[out] p_current
            전류
@return     @ref clink_robot_joint_current_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_joint_current_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t joint_index,
        clink_float_t* p_current);

/**
@brief      로봇 Joint 의 현재 command 가속도를 반환한다. \n
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  joint_index
            해당 하는 Joint 의 index \n
            6축 로봇의 경우, 첫 번째 축은 0 \n
            마지막 축은 5
@param[out] p_acc
            로봇 Joint 의 현재 command 가속도 (단위 : degree / second ^ 2)
@return     @ref clink_robot_joint_acc_command_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_joint_acc_command_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t joint_index,
        clink_float_t* p_acc);

/**
@brief      로봇 Joint 의 현재 실제 가속도를 반환한다. \n
            (내부적으로 actual speed 를 미분한 값)
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  joint_index
            해당 하는 Joint 의 index \n
            6축 로봇의 경우, 첫 번째 축은 0 \n
            마지막 축은 5
@param[out] p_acc
            로봇 Joint 의 현재 실제 가속도 (단위 : degree / second ^ 2)
@return     @ref clink_robot_joint_acc_actual_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_joint_acc_actual_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t joint_index,
        clink_float_t* p_acc);

/**
@brief      로봇 Joint 에 현재 지령하는 command 속도 를 반환한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  joint_index
            해당 하는 Joint 의 index \n
            6축 로봇의 경우, 첫 번째 축은 0 \n
            마지막 축은 5
@param[out] p_speed
            로봇 Joint 의 현재 command 속도 (단위 : count / second)
@return     @ref clink_robot_joint_speed_command_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_joint_speed_command_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t joint_index,
        clink_float_t* p_speed);

/**
@brief      로봇 Joint 의 현재 실제 속도를 반환한다. \n
            (내부적으로 actual angle 를 미분한 값)
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  joint_index
            해당 하는 Joint 의 index \n
            6축 로봇의 경우, 첫 번째 축은 0 \n
            마지막 축은 5
@param[out] p_speed
            로봇 Joint 의 현재 실제 속도 (단위 : degree / second)
@return     @ref clink_robot_joint_speed_actual_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_joint_speed_actual_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t joint_index,
        clink_float_t* p_speed);

/**
@brief      로봇 Joint 에 현재 지령하는 command angle 값을 반환한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  joint_index
            해당 하는 Joint 의 index \n
            6축 로봇의 경우, 첫 번째 축은 0 \n
            마지막 축은 5 
@param[out] p_angle
            현재 지령하는 command angle (단위 :degree)
@return     @ref clink_robot_joint_angle_command_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_joint_angle_command_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t joint_index,
        clink_float_t* p_angle);

/**
@brief      로봇 Joint 의 현재 실제 위치 (즉, 엔코더에서 읽은 값) 를 반환한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  joint_index
            해당 하는 Joint 의 index \n
            6축 로봇의 경우, 첫 번째 축은 0 \n
            마지막 축은 5
@param[out] p_angle
            로봇 Joint 의 현재 위치 (단위 : degree)
@return     @ref clink_robot_joint_angle_actual_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_joint_angle_actual_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t joint_index,
        clink_float_t* p_angle);

/**
@brief      로봇 Joint 의 현재 실제 torque (즉, 엔코더에서 읽은 값) 를 반환한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  joint_index
            해당 하는 Joint 의 index \n
            6축 로봇의 경우, 첫 번째 축은 0 \n
            마지막 축은 5
@param[out] p_torque
            로봇 Joint 의 현재 torque (단위 : Nm)
@return     @ref clink_robot_joint_torque_actual_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_joint_torque_actual_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t joint_index,
        clink_float_t* p_torque);

/** @} */


//------------------------------------------------------------------
// 304_ROBOT_SAFETY
//------------------------------------------------------------------
/** \addtogroup 304_ROBOT_SAFETY 304_ROBOT_SAFETY
*  @{
*/

/**
@brief      현재 설정되어 있는 로봇의 안전설정의 유효화 혹은 무효화 상태를 반환한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_on_off 
            CLINK_SWITCH_ON : 안전설정 유효화 \n
            CLINK_SWITCH_OFF : 안전설정 무효화
@return     @ref clink_robot_safety_limit_switch_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_safety_limit_switch_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH* p_on_off);

/**
@brief      로봇의 안전설정의 유효화 혹은 무효화한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  on_off 
            CLINK_SWITCH_ON : 안전설정 유효화 \n
            CLINK_SWITCH_OFF : 안전설정 무효화
@remark     안전설정 무효화는 안전설정을 위반하여 \n
            로봇을 안전영역으로 복귀시킬 때 사용하는 것을 \n
            목적으로 한다. 일반적인 상황에서 안전설정을 \n
            무효화하면, 안전설정을 위반해도 로봇은 동작을 \n
            계속하기 때문에 위험할 수 있다.
@return     @ref clink_robot_safety_limit_switch_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_safety_limit_switch_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH on_off);

/**
@brief      로봇의 안전설정 상의 해당 Joint 의 최대 가속도를 반환한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  joint_index
            해당 하는 Joint 의 index \n
            6축 로봇의 경우, 첫 번째 축은 0 \n
            마지막 축은 5 
@param[out] p_acc_max
            최대 가속도 값 (단위 : degree / second ^ 2)
@return     @ref clink_robot_safety_limit_joint_acc_max_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_safety_limit_joint_acc_max_get(
        uint32_t cbox_id,
        uint32_t robot_id, 
        uint32_t joint_index,
        clink_float_t* p_acc_max);

/**
@brief      로봇의 안전설정 상의 해당 Joint 의 최대 가속도를 설정한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  joint_index
            해당 하는 Joint 의 index \n
            6축 로봇의 경우, 첫 번째 축은 0 \n
            마지막 축은 5 
@param[in]  acc_max 
            최대 가속도 값 (단위 : degree / second^2)
@return     @ref clink_robot_safety_limit_joint_acc_max_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_safety_limit_joint_acc_max_set(
        uint32_t cbox_id,
        uint32_t robot_id, 
        uint32_t joint_index, 
        clink_float_t acc_max);

/**
@brief      로봇의 안전설정 상의 해당 Joint 의 최대 속도를 반환한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  joint_index
            해당 하는 Joint 의 index \n
            6축 로봇의 경우, 첫 번째 축은 0 \n
            마지막 축은 5 
@param[out] p_speed_max
            최대 속도 값 (단위 : degree / second)
@return     @ref clink_robot_safety_limit_joint_speed_max_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_safety_limit_joint_speed_max_get(
        uint32_t cbox_id,
        uint32_t robot_id, 
        uint32_t joint_index,
        clink_float_t* p_speed_max);

/**
@brief      로봇의 안전설정 상의 해당 Joint 의 최대 속도를 설정한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  joint_index
            해당 하는 Joint 의 index \n
            6축 로봇의 경우, 첫 번째 축은 0 \n
            마지막 축은 5 
@param[in]  speed_max 
            최대 속도 값 (단위 : degree / second)
@return     @ref clink_robot_safety_limit_joint_speed_max_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_safety_limit_joint_speed_max_set(
        uint32_t cbox_id,
        uint32_t robot_id, 
        uint32_t joint_index, 
        clink_float_t speed_max);

/**
@brief      로봇의 안전설정 상의 해당 Joint 의 구동 범위 중 \n
            최소 각도를 반환한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  joint_index
            해당 하는 Joint 의 index \n
            6축 로봇의 경우, 첫 번째 축은 0 \n
            마지막 축은 5 
@param[out] p_angle_min
            최소 각도 값 (단위 : degree)
@param[out] p_angle_max
            최대 각도 값 (단위 : degree)
@return     @ref clink_robot_safety_limit_joint_angle_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_safety_limit_joint_angle_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t joint_index,
        clink_float_t* p_angle_min,
        clink_float_t* p_angle_max);

/**
@brief      로봇의 안전설정 상의 해당 Joint 의 구동 범위 중 \n
            최소 각도를 설정한다. 
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  joint_index
            해당 하는 Joint 의 index \n
            6축 로봇의 경우, 첫 번째 축은 0 \n
            마지막 축은 5 
@param[in]  angle_min 
            최소 각도 값 (단위 : degree)
@param[in]  angle_max
            최대 각도 값 (단위 : degree)
@return     @ref clink_robot_safety_limit_joint_angle_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_safety_limit_joint_angle_set(
        uint32_t cbox_id,
        uint32_t robot_id, 
        uint32_t joint_index, 
        clink_float_t angle_min,
        clink_float_t angle_max);

/**
@brief      로봇의 안전설정 평면을 설정 및 생성한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  norm_x 
            평면의 방향 normal vector 중 X 성분
@param[in]  norm_y
            평면의 방향 normal vector 중 Y 성분
@param[in]  norm_z 
            평면의 방향 normal vector 중 Z 성분
@param[in]  origin_x 
            평면의 원점 중 X 성분
@param[in]  origin_y
            평면의 원점 중 Y 성분
@param[in]  origin_z 
            평면의 원점 중 Z 성분
@param[out] p_plane_id
            평면의 ID
@return     @ref clink_robot_safety_limit_plane_create_with_norm_origin_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_safety_limit_plane_create_with_norm_origin(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t norm_x,
        clink_float_t norm_y,
        clink_float_t norm_z,
        clink_float_t origin_x,
        clink_float_t origin_y,
        clink_float_t origin_z,
        uint32_t* p_plane_id);

/**
@brief      로봇의 안전설정 평면을 설정 및 생성한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  norm_x 
            평면의 방향 normal vector 중 X 성분
@param[in]  norm_y
            평면의 방향 normal vector 중 Y 성분
@param[in]  norm_z 
            평면의 방향 normal vector 중 Z 성분
@param[in]  distance 
            Base 좌표계의 원점으로 부터 normal vector \n
            방향으로의 거리. \n
            즉, clink_robot_safety_limit_plane_create_with_norm_origin() \n
            함수에서의 origin 은 \n
            origin = normal vector * distance \n
            로 표현할 수 있다.
@param[out] p_plane_id
            평면의 ID
@return     @ref clink_robot_safety_limit_plane_create_with_norm_distance_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_safety_limit_plane_create_with_norm_distance(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t norm_x,
        clink_float_t norm_y,
        clink_float_t norm_z,
        clink_float_t distance,
        uint32_t* p_plane_id);

/**
@brief      clink_robot_safety_limit_plane_create_with_norm_origin() 또는 \n
            clink_rpc_robot_safety_limit_plane_create_with_norm_distance() 으로 생성한 \n
            평면의 정보를 얻는다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  plane_id
            평면의 ID
@param[out] norm_x
            평면의 방향 normal vector 중 X 성분
@param[out] norm_y
            평면의 방향 normal vector 중 Y 성분
@param[out] norm_z
            평면의 방향 normal vector 중 Z 성분
@param[out] origin_x
            평면의 원점 중 X 성분
@param[out]  origin_y
            평면의 원점 중 Y 성분
@param[out] origin_z
            평면의 원점 중 Z 성분
@return     @ref clink_robot_safety_limit_plane_norm_origin_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_safety_limit_plane_norm_origin_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t plane_id,
        clink_float_t* norm_x,
        clink_float_t* norm_y,
        clink_float_t* norm_z,
        clink_float_t* origin_x,
        clink_float_t* origin_y,
        clink_float_t* origin_z);

/**
@brief      clink_robot_safety_limit_plane_create_with_norm_origin() 또는 \n
            clink_rpc_robot_safety_limit_plane_create_with_norm_distance() 으로 생성한 \n
            평면의 정보를 얻는다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  plane_id
            평면의 ID
@param[out] norm_x
            평면의 방향 normal vector 중 X 성분
@param[out] norm_y
            평면의 방향 normal vector 중 Y 성분
@param[out] norm_z
            평면의 방향 normal vector 중 Z 성분
@param[out] distance
            Base 좌표계의 원점으로 부터 normal vector \n
            방향으로의 거리. \n
            즉, clink_robot_safety_limit_plane_create_with_norm_origin() \n
            함수에서의 origin 은 \n
            origin = normal vector * distance \n
            로 표현할 수 있다.
@return     @ref clink_robot_safety_limit_plane_norm_distance_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_safety_limit_plane_norm_distance_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t plane_id,
        clink_float_t* norm_x,
        clink_float_t* norm_y,
        clink_float_t* norm_z,
        clink_float_t* distance);

/**
@brief      로봇에 안전설정된 안전평면을 삭제한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  plane_id 
            평면의 ID \n
            즉, clink_robot_safety_limit_plane_create_with_norm_origin() 또는 \n
            clink_rpc_robot_safety_limit_plane_create_with_norm_distance() \n
            을 통해 반환 받은 값
@remark     이 함수는 로봇에 설정되었던 안전평면을 \n
            삭제한다는 의미이다. 안전평면 자체를 삭제하는 것은 아니다. \n
            안전평면 자체를 삭제하기 위해서는 \n
            clink_rpc_robot_safety_limit_plane_destroy() \n
            를 사용해야 한다.
@return     @ref clink_robot_safety_limit_plane_remove_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_safety_limit_plane_remove(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t plane_id);

/**
@brief      로봇에 설정된 모든 안전 평면을 삭제한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@remark     이 함수는 로봇에 설정되었던 안전평면을 \n
            삭제한다는 의미이다. 안전평면 자체를 삭제하는 것은 아니다. \n
            안전평면 자체를 삭제하기 위해서는 \n
            clink_rpc_robot_safety_limit_plane_destroy() \n
            를 사용해야 한다.
@return     @ref clink_robot_safety_limit_plane_remove_all_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_safety_limit_plane_remove_all(
        uint32_t cbox_id,
        uint32_t robot_id);

/**
@brief      로봇의 안전설정 상의 TCP 의 최대 가속도를 반환한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_acc_max
            최대 가속도 값 (단위 : millimeter / second^2)
@return     @ref clink_robot_safety_limit_tcp_acc_max_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_safety_limit_tcp_acc_max_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t* p_acc_max);

/**
@brief      로봇의 안전설정 상의 TCP 의 최대 가속도를 설정한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  acc_max
            최대 가속도 값 (단위 : millimeter / second ^ 2)
@return     @ref clink_robot_safety_limit_tcp_acc_max_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_safety_limit_tcp_acc_max_set(
        uint32_t cbox_id,
        uint32_t robot_id, 
        clink_float_t acc_max);

/**
@brief      로봇의 안전설정 상의 TCP 의 최대 속도를 반환한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_speed_max
            최대 속도 값 (단위 : millimeter / second)
@return     @ref clink_robot_safety_limit_tcp_speed_max_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_safety_limit_tcp_speed_max_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t* p_speed_max);

/**
@brief      로봇의 안전설정 상의 TCP 의 최대 속도를 설정한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  speed_max
            최대 속도 값 (단위 : millimeter / second)
@return     @ref clink_robot_safety_limit_tcp_speed_max_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_safety_limit_tcp_speed_max_set(
        uint32_t cbox_id,
        uint32_t robot_id, 
        clink_float_t speed_max);

/**
@brief      로봇의 TCP (Tool Center Point) 의 안전 경계 영역을 표시하는 \n
            bounding box 를 설정한다. \n
            포인트 순서는 상관 없으나, 중복 포인트는 불가하다. \n
            clink_rpc_robot_safety_limit_tcp_bounding_shape_switch_set() \n
            을 통해 유효화했을 때, 유효하다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  ref_coord
            CLINK_REF_COORDINATE_BASE 또는, \n
            CLINK_REF_COORDINATE_TCP
@param[in]  pt1_x
            Bounding box 포인트 1의 X 성분
@param[in]  pt1_y
            Bounding box 포인트 1의 Y 성분
@param[in]  pt1_z
            Bounding box 포인트 1의 Z 성분
@param[in]  pt2_x
            Bounding box 포인트 2의 X 성분
@param[in]  pt2_y
            Bounding box 포인트 2의 Y 성분
@param[in]  pt2_z
            Bounding box 포인트 2의 Z 성분
@param[in]  pt3_x
            Bounding box 포인트 3의 X 성분
@param[in]  pt3_y
            Bounding box 포인트 3의 Y 성분
@param[in]  pt3_z
            Bounding box 포인트 3의 Z 성분
@param[in]  pt4_x
            Bounding box 포인트 4의 X 성분
@param[in]  pt4_y
            Bounding box 포인트 4의 Y 성분
@param[in]  pt4_z
            Bounding box 포인트 4의 Z 성분
@param[in]  pt5_x
            Bounding box 포인트 5의 X 성분
@param[in]  pt5_y
            Bounding box 포인트 5의 Y 성분
@param[in]  pt5_z
            Bounding box 포인트 5의 Z 성분
@param[in]  pt6_x
            Bounding box 포인트 6의 X 성분
@param[in]  pt6_y
            Bounding box 포인트 6의 Y 성분
@param[in]  pt6_z
            Bounding box 포인트 6의 Z 성분
@param[in]  pt7_x
            Bounding box 포인트 7의 X 성분
@param[in]  pt7_y
            Bounding box 포인트 7의 Y 성분
@param[in]  pt7_z
            Bounding box 포인트 7의 Z 성분
@param[in]  pt8_x
            Bounding box 포인트 8의 X 성분
@param[in]  pt8_y
            Bounding box 포인트 8의 Y 성분
@param[in]  pt8_z
            Bounding box 포인트 8의 Z 성분
@return     @ref clink_robot_safety_limit_tcp_bounding_box_set_return "returnable error code"
*/
/* 아래 그림 참조.
         pt2                   pt3
          /|------------------/|
         / |             pt4 / |
    pt1 /--|-pt6------------/  |
        |  /----------------|  / pt7
        | /                 | /
    pt5 |/------------------|/ pt8

*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_safety_limit_tcp_bounding_box_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_REF_COORDINATE ref_coord,
        clink_float_t pt1_x, 
        clink_float_t pt1_y, 
        clink_float_t pt1_z,
        clink_float_t pt2_x, 
        clink_float_t pt2_y, 
        clink_float_t pt2_z,
        clink_float_t pt3_x, 
        clink_float_t pt3_y, 
        clink_float_t pt3_z,
        clink_float_t pt4_x, 
        clink_float_t pt4_y, 
        clink_float_t pt4_z,
        clink_float_t pt5_x, 
        clink_float_t pt5_y, 
        clink_float_t pt5_z,
        clink_float_t pt6_x, 
        clink_float_t pt6_y, 
        clink_float_t pt6_z,
        clink_float_t pt7_x, 
        clink_float_t pt7_y, 
        clink_float_t pt7_z,
        clink_float_t pt8_x, 
        clink_float_t pt8_y, 
        clink_float_t pt8_z);

/**
@brief      TCP (Tool Center Point) 의 안전 경계 영역을 표시하는 \n
            bounding sphere (구) 를 설정한다. \n
            clink_rpc_robot_safety_limit_tcp_bounding_shape_switch_set() \n
            을 통해 유효화했을 때, 유효하다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  ref_coord
            CLINK_REF_COORDINATE_BASE 또는, \n
            CLINK_REF_COORDINATE_TCP
@param[in]  center_x
            Bounding Sphere 중심점의 X 성분
@param[in]  center_y
            Bounding Sphere 중심점의 Y 성분
@param[in]  center_z
            Bounding Sphere 중심점의 Z 성분
@param[in]  radius
            Bounding Sphere 의 반지름
@return     @ref clink_robot_safety_limit_tcp_bounding_sphere_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_safety_limit_tcp_bounding_sphere_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_REF_COORDINATE ref_coord,
        clink_float_t center_x, 
        clink_float_t center_y, 
        clink_float_t center_z,
        clink_float_t radius);

/**
@brief      TCP (Tool Center Point) 의 안전 경계 영역을 표시하는 \n
            bounding cylinder (원기둥) 를 설정한다. \n
            clink_rpc_robot_safety_limit_tcp_bounding_shape_switch_set() \n
            을 통해 유효화했을 때, 유효하다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  ref_coord
            CLINK_REF_COORDINATE_BASE 또는, \n
            CLINK_REF_COORDINATE_TCP
@param[in]  center_x
            Bounding cylinder 중심점의 X 성분
@param[in]  center_y
            Bounding cylinder 중심점의 Y 성분
@param[in]  center_z
            Bounding cylinder 중심점의 Z 성분
@param[in]  normal_x
            Bounding cylinder 방향 벡터의 X 성분
@param[in]  normal_y
            Bounding cylinder 방향 벡터의 Y 성분
@param[in]  normal_z
            Bounding cylinder 방향 벡터의 Z 성분
@param[in]  height
            Bounding cylinder 의 높이
@param[in]  radius
            Bounding cylinder 의 반지름 
@return     @ref clink_robot_safety_limit_tcp_bounding_cylinder_set_return "returnable error code"
*/
/* 아래 그림 참조.
     ()##################################()----┐
    (  )                                (  ) (radius)
   (    )              *(center)       (    )--┘
    (  )       <-or->   (normal)        (  )
     ()##################################()
      └------------ (height)-------------┘
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_safety_limit_tcp_bounding_cylinder_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_REF_COORDINATE ref_coord,
        clink_float_t center_x, 
        clink_float_t center_y, 
        clink_float_t center_z,
        clink_float_t normal_x, 
        clink_float_t normal_y, 
        clink_float_t normal_z,
        clink_float_t height, 
        clink_float_t radius);

/**
@brief      TCP (Tool Center Point) 의 안전 경계 영역을 표시하는 \n
            bounding cone (원뿔) 를 설정한다. \n
            clink_rpc_robot_safety_limit_tcp_bounding_shape_switch_set() \n
            을 통해 유효화했을 때, 유효하다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  ref_coord
            CLINK_REF_COORDINATE_BASE 또는, \n
            CLINK_REF_COORDINATE_TCP
@param[in]  center_x
            Bounding cone 중심점의 X 성분
@param[in]  center_y
            Bounding cone 중심점의 Y 성분
@param[in]  center_z
            Bounding cone 중심점의 Z 성분
@param[in]  normal_x
            Bounding cone 방향 벡터의 X 성분
@param[in]  normal_y
            Bounding cone 방향 벡터의 Y 성분
@param[in]  normal_z
            Bounding cone 방향 벡터의 Z 성분
@param[in]  height
            Bounding cone 의 높이
@param[in]  radius
            Bounding cone 의 반지름
@return     @ref clink_robot_safety_limit_tcp_bounding_cone_set_return "returnable error code"
*/
/* 아래 그림 참조
                              #######()----------┐
                        #######     (  )         |
                  #######          (    )      (radius)
            #######               (      )       |
      #######                    (        )      |
#######    <-(normal) *(center) (          )-----┘
|     #######                    (        )
|           #######               (      )
|                 #######          (    )
|                       #######     (  )
|                             #######()
└-------------- (height)--------------┘
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_safety_limit_tcp_bounding_cone_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_REF_COORDINATE ref_coord,
        clink_float_t center_x, 
        clink_float_t center_y, 
        clink_float_t center_z,
        clink_float_t normal_x, 
        clink_float_t normal_y, 
        clink_float_t normal_z,
        clink_float_t height, 
        clink_float_t radius);

/**
@brief      이하의 함수를 통해 설정한 bounding shape 의 유효화 혹은 무효화 한다.
            clink_rpc_robot_safety_limit_tcp_bounding_box_set(), \n
            clink_rpc_robot_safety_limit_tcp_bounding_sphere_set(), \n
            clink_rpc_robot_safety_limit_tcp_bounding_cylinder_set(), \n
            clink_rpc_robot_safety_limit_tcp_bounding_cone_set() \n
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  on_off
            CLINK_SWITCH_ON : 유효화 \n
            CLINK_SWITCH_OFF : 무효화
@return     @ref clink_robot_safety_limit_tcp_bounding_shape_switch_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_safety_limit_tcp_bounding_shape_switch_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH on_off);

/**
@brief      Bounding shape 의 유효화 혹은 무효화 상태를 반환한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_on_off
            CLINK_SWITCH_ON : 유효화 \n
            CLINK_SWITCH_OFF : 무효화
@return     @ref clink_robot_safety_limit_tcp_bounding_shape_switch_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_safety_limit_tcp_bounding_shape_switch_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH* p_on_off);

/**
@brief      이하의 함수를 통해 설정한 bounding shape를 삭제한다.
            clink_rpc_robot_safety_limit_tcp_bounding_box_set(), \n
            clink_rpc_robot_safety_limit_tcp_bounding_sphere_set(), \n
            clink_rpc_robot_safety_limit_tcp_bounding_cylinder_set(), \n
            clink_rpc_robot_safety_limit_tcp_bounding_cone_set() \n
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@return     @ref clink_robot_safety_limit_tcp_bounding_shape_delete_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_safety_limit_tcp_bounding_shape_delete(
        uint32_t cbox_id,
        uint32_t robot_id);

/**
@brief      Reduced Mode에서 사용되는 제한 속도를 설정한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  vel_tcp
            TCP의 속도, \n
            최소 : 1 \n
            (단위 : millimeter / second)
@param[in]  vel_joint
            Joint의 속도, \n
            최소 : 1, 최대 : 180 \n
            (단위 : degree / second)
@return     @ref clink_robot_safety_limit_velocity_in_reduced_mode_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_safety_limit_velocity_in_reduced_mode_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t vel_tcp,
        clink_float_t vel_joint);

/**
@brief      설정된 Reduced Mode에서 사용 속도 값을 리턴한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_vel_tcp
            TCP의 속도
@param[out] p_vel_joint
            Joint의 속도
@return     @ref clink_robot_safety_limit_velocity_in_reduced_mode_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_safety_limit_velocity_in_reduced_mode_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t* p_vel_tcp,
        clink_float_t* p_vel_joint);

/**
@brief      Reduced Mode에서 사용되는 충돌 감지 축 임계치를 설정한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  num_of_joints
            로봇 관절 갯수
@param[in]  p_torque_threshold
            collision detection torque threshold 값 \n
            (단위 : N)
@return     @ref clink_robot_safety_limit_collision_detection_torque_in_reduced_mode_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_safety_limit_collision_detection_torque_in_reduced_mode_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t num_of_joints,
        const clink_float_t* p_torque_threshold);

/**
@brief      Reduced Mode에서 사용되는 충돌 감지 축 임계치 값을 받아온다
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  num_of_joints
            로봇 관절 갯수
@param[out] p_torque_threshold
            collision detection torque threshold 값 \n
            (단위 : N)
@return     @ref clink_robot_safety_limit_collision_detection_torque_in_reduced_mode_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_safety_limit_collision_detection_torque_in_reduced_mode_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t num_of_joints,
        clink_float_t* p_torque_threshold);

/**
@brief      Reduced Mode에서 사용되는 충돌 감지 tcp 임계치를 설정한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  force_threshold
            collision detection tcp resultant force threshold 값 \n
            (단위 : N)
@return     @ref clink_robot_safety_limit_collision_detection_tcp_force_in_reduced_mode_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_safety_limit_collision_detection_tcp_force_in_reduced_mode_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t force_threshold);

/**
@brief      Reduced Mode에서 사용되는 충돌 감지 tcp 임계치 값을 받아온다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_force_threshold
            collision detection tcp resultant force threshold 값 \n
            (단위 : N)
@return     @ref clink_robot_safety_limit_collision_detection_tcp_force_in_reduced_mode_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_safety_limit_collision_detection_tcp_force_in_reduced_mode_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t* p_force_threshold);

/**
@brief      로봇의 Safety 관련 변수로 계산한 CRC 값을 리턴한다.
            Safety 관련 설정 변경 시 CRC 값은 변경된다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_crc 
            계산된 CRC 값을 리턴한다.
@return     @ref clink_robot_safety_crc32_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_safety_crc32_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t* p_crc);

/** @} */


//------------------------------------------------------------------
// 305_ROBOT_JOG
//------------------------------------------------------------------
/** \addtogroup 305_ROBOT_JOG 305_ROBOT_JOG
*  @{ 
*/

/**
@brief      Robot TCP 를 임의의 방향으로 이동한다. (JOG 모드) \n
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  ref_coord
            이동 방향의 기준 좌표계  \n
            CLINK_REF_COORDINATE_BASE 또는, \n
            CLINK_REF_COORDINATE_TCP
@param[in]  dir_x
            JOG 방향 중 X 성분
@param[in]  dir_y
            JOG 방향 중 Y 성분
@param[in]  dir_z
            JOG 방향 중 Z 성분
@param[in]  speed
            로봇의 이동 속도 (단위 : millimeter / second)
@param[in]  distance
            이동 distance (단위 : millimeter) \n
            JOG 를 통해 이동할 거리를 지정한다. \n
            충분히 큰 거리 (예: 100 meter 에 해당하는 100000) \n
            를 입력하는 경우, 로봇은 workspace 의 한계점까지 이동한다. \n
            작은 거리를 입력하는 경우, 해당 거리만을 이동한다.
@return     @ref clink_robot_jog_tcp_position_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_jog_tcp_position(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_REF_COORDINATE ref_coord,
        clink_float_t dir_x,
        clink_float_t dir_y,
        clink_float_t dir_z,
        clink_float_t speed,
        clink_float_t distance);

/**
@brief      Robot TCP 를 X 축에 따라 이동시킨다. (JOG 모드)
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  ref_coord
            이동 방향의 기준 좌표계  \n
            CLINK_REF_COORDINATE_BASE 또는, \n
            CLINK_REF_COORDINATE_TCP
@param[in]  positive_or_negative
            양의 방향 혹은 음의 방향 지정 \n
            CLINK_DIRECTION_POSITIVE 또는 \n
            CLINK_DIRECTION_NEGATIVE
@param[in]  speed
            로봇의 이동 속도 (단위 : millimeter / second)
@param[in]  distance
            이동 distance (단위 : millimeter) \n
            JOG 를 통해 이동할 거리를 지정한다. \n
            충분히 큰 거리 (예: 100 meter 에 해당하는 100000) \n
            를 입력하는 경우, 로봇은 workspace 의 한계점까지 이동한다. \n
            작은 거리를 입력하는 경우, 해당 거리만을 이동한다.
@return     @ref clink_robot_jog_tcp_position_x_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_jog_tcp_position_x(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_REF_COORDINATE ref_coord,
        CLINK_DIRECTION positive_or_negative,
        clink_float_t speed,
        clink_float_t distance);

/**
@brief      Robot TCP 를 Y 축에 따라 이동시킨다. (JOG 모드) \n
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  ref_coord
            이동 방향의 기준 좌표계  \n
            CLINK_REF_COORDINATE_BASE 또는, \n
            CLINK_REF_COORDINATE_TCP
@param[in]  positive_or_negative
            양의 방향 혹은 음의 방향 지정 \n
            CLINK_DIRECTION_POSITIVE 또는 \n
            CLINK_DIRECTION_NEGATIVE
@param[in]  speed
            로봇의 이동 속도 (단위 : millimeter / second)
@param[in]  distance
            이동 distance (단위 : millimeter) \n
            JOG 를 통해 이동할 거리를 지정한다. \n
            충분히 큰 거리 (예: 100 meter 에 해당하는 100000) \n
            를 입력하는 경우, 로봇은 workspace 의 한계점까지 이동한다. \n
            작은 거리를 입력하는 경우, 해당 거리만을 이동한다.
@return     @ref clink_robot_jog_tcp_position_y_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_jog_tcp_position_y(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_REF_COORDINATE ref_coord,
        CLINK_DIRECTION positive_or_negative,
        clink_float_t speed,
        clink_float_t distance);

/**
@brief      Robot TCP 를 Z 축에 따라 이동시킨다. (JOG 모드) \n
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  ref_coord
            이동 방향의 기준 좌표계  \n
            CLINK_REF_COORDINATE_BASE 또는, \n
            CLINK_REF_COORDINATE_TCP
@param[in]  positive_or_negative
            양의 방향 혹은 음의 방향 지정 \n
            CLINK_DIRECTION_POSITIVE 또는 \n
            CLINK_DIRECTION_NEGATIVE
@param[in]  speed
            로봇의 이동 속도 (단위 : millimeter / second)
@param[in]  distance
            이동 distance (단위 : millimeter) \n
            JOG 를 통해 이동할 거리를 지정한다. \n
            충분히 큰 거리 (예: 100 meter 에 해당하는 100000) \n
            를 입력하는 경우, 로봇은 workspace 의 한계점까지 이동한다. \n
            작은 거리를 입력하는 경우, 해당 거리만을 이동한다.
@return     @ref clink_robot_jog_tcp_position_z_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_jog_tcp_position_z(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_REF_COORDINATE ref_coord,
        CLINK_DIRECTION positive_or_negative,
        clink_float_t speed,
        clink_float_t distance);

/**
@brief      Robot TCP 의 orientation 을 \n
            입력받은 임의의 축을 회전축으로 하여 회전시킨다.  (JOG 모드) \n
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  ref_coord
            회전의 기준 좌표계  \n
            CLINK_REF_COORDINATE_BASE 또는, \n
            CLINK_REF_COORDINATE_TCP
@param[in]  rot_aixs_dir_x
            Rotation 의 회전축의 방향 중 X 성분
@param[in]  rot_aixs_dir_y
            Rotation 의 회전축의 방향 중 Y 성분
@param[in]  rot_aixs_dir_z
            Rotation 의 회전축의 방향 중 Z 성분
@param[in]  speed
            로봇의 회전 속도 (단위 : degree / second)
@param[in]  distance
            회전 distance (단위 : degree) \n
            JOG 를 통해 회전할 거리를 지정한다. \n
            충분히 큰 거리 (예: 10 바퀴에 해당하는 3600) \n
            를 입력하는 경우, 로봇은 workspace 의 한계점까지 이동한다. \n
            작은 거리를 입력하는 경우, 해당 거리만을 이동한다.
@return     @ref clink_robot_jog_tcp_orientation_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_jog_tcp_orientation(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_REF_COORDINATE ref_coord,
        clink_float_t rot_aixs_dir_x,
        clink_float_t rot_aixs_dir_y,
        clink_float_t rot_aixs_dir_z,
        clink_float_t speed,
        clink_float_t distance);

/**
@brief      Robot TCP 의 orientation 을 \n
            X 축을 회전축으로 하여 회전시킨다.  (JOG 모드) \n
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  ref_coord
            회전의 기준 좌표계  \n
            CLINK_REF_COORDINATE_BASE 또는, \n
            CLINK_REF_COORDINATE_TCP
@param[in]  positive_or_negative
            양의 방향 혹은 음의 방향 지정 \n
            CLINK_DIRECTION_POSITIVE 또는 \n
            CLINK_DIRECTION_NEGATIVE
@param[in]  speed
            로봇의 회전 속도 (단위 : degree / second)
@param[in]  rotation_distance
            회전 distance (단위 : degree) \n
            JOG 를 통해 회전할 거리를 지정한다. \n
            충분히 큰 거리 (예: 10 바퀴에 해당하는 3600) \n
            를 입력하는 경우, 로봇은 workspace 의 한계점까지 이동한다. \n
            작은 거리를 입력하는 경우, 해당 거리만을 이동한다.
@return     @ref clink_robot_jog_tcp_orientation_x_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_jog_tcp_orientation_x(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_REF_COORDINATE ref_coord,
        CLINK_DIRECTION positive_or_negative,
        clink_float_t speed,
        clink_float_t rotation_distance);
        
/**
@brief      Robot TCP 의 orientation 을 \n
            Y 축을 회전축으로 하여 회전시킨다.  (JOG 모드) \n
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  ref_coord
            회전의 기준 좌표계  \n
            CLINK_REF_COORDINATE_BASE 또는, \n
            CLINK_REF_COORDINATE_TCP
@param[in]  positive_or_negative
            양의 방향 혹은 음의 방향 지정 \n
            CLINK_DIRECTION_POSITIVE 또는 \n
            CLINK_DIRECTION_NEGATIVE
@param[in]  speed
            로봇의 회전 속도 (단위 : degree / second)
@param[in]  rotation_distance
            회전 distance (단위 : degree) \n
            JOG 를 통해 회전할 거리를 지정한다. \n
            충분히 큰 거리 (예: 10 바퀴에 해당하는 3600) \n
            를 입력하는 경우, 로봇은 workspace 의 한계점까지 이동한다. \n
            작은 거리를 입력하는 경우, 해당 거리만을 이동한다.
@return     @ref clink_robot_jog_tcp_orientation_y_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_jog_tcp_orientation_y(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_REF_COORDINATE ref_coord,
        CLINK_DIRECTION positive_or_negative,
        clink_float_t speed,
        clink_float_t rotation_distance);

/**
@brief      Robot TCP 의 orientation 을 \n
            Z 축을 회전축으로 하여 회전시킨다.  (JOG 모드) \n
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  ref_coord
            회전의 기준 좌표계  \n
            CLINK_REF_COORDINATE_BASE 또는, \n
            CLINK_REF_COORDINATE_TCP
@param[in]  positive_or_negative
            양의 방향 혹은 음의 방향 지정 \n
            CLINK_DIRECTION_POSITIVE 또는 \n
            CLINK_DIRECTION_NEGATIVE
@param[in]  speed
            로봇의 회전 속도 (단위 : degree / second)
@param[in]  rotation_distance
            회전 distance (단위 : degree) \n
            JOG 를 통해 회전할 거리를 지정한다. \n
            충분히 큰 거리 (예: 10 바퀴에 해당하는 3600) \n
            를 입력하는 경우, 로봇은 workspace 의 한계점까지 이동한다. \n
            작은 거리를 입력하는 경우, 해당 거리만을 이동한다.
@return     @ref clink_robot_jog_tcp_orientation_z_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_jog_tcp_orientation_z(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_REF_COORDINATE ref_coord,
        CLINK_DIRECTION positive_or_negative,
        clink_float_t speed,
        clink_float_t rotation_distance);

/**
@brief      Robot Joint 의 Jog 를 실행한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  joint_index
            해당 하는 Joint 의 index \n
            6축 로봇의 경우, 첫 번째 축은 0 \n
            마지막 축은 5 
@param[in]  positive_or_negative
            양의 방향 혹은 음의 방향 지정 \n
            CLINK_DIRECTION_POSITIVE 또는 \n
            CLINK_DIRECTION_NEGATIVE
@param[in]  speed
            Joint 의 이동 속도 (단위 : degree / second)
@param[in]  distance
            Joint 의 이동거리 (단위 : degree) \n
            JOG 를 통해 Joint 가 이동할 거리를 지정한다. \n
            충분히 큰 거리 (예: 10 바퀴에 해당하는 3600) \n
            를 입력하는 경우, 로봇은 Joint 의 Specification 상 \n
            구동범위까지 이동한다. 
@return     @ref clink_robot_jog_joint_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_jog_joint(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t joint_index,
        CLINK_DIRECTION positive_or_negative,
        clink_float_t speed,
        clink_float_t distance);

/** @} */


//------------------------------------------------------------------
// 306_ROBOT_TOOL_IO
//------------------------------------------------------------------
/** \addtogroup 306_ROBOT_TOOL_IO 306_ROBOT_TOOL_IO
*  @{
*/

/**
@brief      Robot 의 Tool IO 의 Digital IO 의 특정 포트 (Input) 의 값을 읽는다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  io_number 
            Digital IO 번호
@param[out] p_value 
            현재 해당 IO 를 통해 입력되고 있는 값
@return     @ref clink_robot_tool_io_digital_input_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_tool_io_digital_input_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t io_number,
        uint32_t* p_value);

/**
@brief      clink_robot_tool_io_digital_output_set() 를 통해 설정한 값을 반환한다. \n
            즉, 현재 해당 포트를 통해 출력되는 값을 반환한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  io_number 
            Digital IO 번호
@param[out] p_value
            현재 io_number 에 해당하는 Digital IO 를 통해 출력되는 값
@return     @ref clink_robot_tool_io_digital_output_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_tool_io_digital_output_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t io_number,
        uint32_t* p_value);

/**
@brief      Robot 의 Tool IO 의 Digital IO 의 특정 포트 (Output) 의 값을 설정한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  io_number 
            Digital IO 번호
@param[in]  io_value  
            Digital IO 에 설정할 값
@return     @ref clink_robot_tool_io_digital_output_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_tool_io_digital_output_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t io_number, 
        uint32_t io_value);

/**
@brief      Robot 의 Tool IO 의 Digital IO 의 특정 포트 (Output) 의 값을 설정한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  num_of_ios
            설정할 IO 갯수
@param[in]  p_io_number
            Digital IO 번호
@param[in]  p_io_value
            Digital IO 에 설정할 값
@return     @ref clink_robot_tool_io_digital_output_multiple_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_tool_io_digital_output_multiple_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t num_of_ios,
        const uint32_t* p_io_number,
        const uint32_t* p_io_value);

/**
@brief      Robot 의 Tool IO 의 Analog IO 의 특정 포트 (Input) 값을 반환한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  io_number 
            Analog IO 번호
@param[out] p_value
            현재 io_number 에 해당하는 Analog IO 를 통해 입력되는 값
@return     @ref clink_robot_tool_io_analog_input_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_tool_io_analog_input_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t io_number,
        clink_float_t* p_value);

/**
@brief      clink_robot_tool_io_analog_output_set() 를 통해 설정한 값을 반환한다. \n
            즉, 현재 해당 포트를 통해 출력되는 값을 반환한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  io_number
            Analog IO 번호
@param[out] p_value
            현재 io_number 에 해당하는 Analog IO 를 통해 출력되는 값
@return     @ref clink_robot_tool_io_analog_output_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_tool_io_analog_output_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t io_number,
        clink_float_t* p_value);

/**
@brief      Robot 의 Tool IO 의 Analog IO 의 특정 포트 (Output) 의 값을 설정한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  io_number
            Analog IO 번호
@param[in]  io_value
            Analog IO 에 설정할 값
@return     @ref clink_robot_tool_io_analog_output_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_tool_io_analog_output_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t io_number, 
        clink_float_t io_value);

/**
@brief      Robot 의 Tool IO 의 Analog IO 의 특정 포트 (Input) 의 모드를 반환한다. \n
            현재, Voltage (전압), Current (전류) 모드가 선택가능하다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  io_number 
            Analog IO 번호
@param[out] p_mode
            CLINK_ANALOG_IO_MODE_VOLTAGE : 전압 모드 \n
            CLINK_ANALOG_IO_MODE_CURRENT : 전류 모드
@param[out] p_i_reserved
            Reserved Option \n
            각 사용법은 각 로봇의 문서 참조.
@param[out] p_f_reserved
            Reserved Option \n
            각 사용법은 각 로봇의 문서 참조.
@return     @ref clink_robot_tool_io_analog_input_mode_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_tool_io_analog_input_mode_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t io_number,
        CLINK_ANALOG_IO_MODE* p_mode,
        int32_t* p_i_reserved,
        clink_float_t* p_f_reserved);

/**
@brief      Robot 의 Tool IO 의 Analog IO 의 특정 포트 (Input) 의 모드를 설정한다. \n
            현재, Voltage (전압), Current (전류) 모드가 선택가능하다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  io_number 
            Analog IO 번호
@param[in]  io_mode
            전류 / 전압 선택
@param[in]  i_reserved
            Reserved Option \n
            각 사용법은 각 로봇의 문서 참조.
@param[in]  f_reserved
            Reserved Option \n
            각 사용법은 각 로봇의 문서 참조.
@return     @ref clink_robot_tool_io_analog_input_mode_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_tool_io_analog_input_mode_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t io_number, 
        CLINK_ANALOG_IO_MODE io_mode,
        int32_t i_reserved,
        clink_float_t f_reserved);

/**
@brief      셋팅한 Tool I/O의 Power 전압 및 Tool이 소비하는 전류를 받아온다. \n
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_voltage_option
            CLINK_TOOL_IO_POWER_VOLTAGE_OPTION 참조: \n
            CLINK_TOOL_POWER_VOLTAGE_OPTION_0, \n
            CLINK_TOOL_POWER_VOLTAGE_OPTION_12, \n
            CLINK_TOOL_POWER_VOLTAGE_OPTION_24 입력가능
@param[out] p_current
            전류 값을 리턴한다 (Ampere NOT mA)
@remark     Offline Simulator 및 simulation robot 지원불가
@return     @ref clink_robot_tool_io_power_voltage_current_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_tool_io_power_voltage_current_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_TOOL_IO_POWER_VOLTAGE_OPTION* p_voltage_option,
        clink_float_t* p_current);

/**
@brief      Tool I/O의 Power 전압을 셋팅한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  voltage_option
            CLINK_TOOL_IO_POWER_VOLTAGE_OPTION 참조: \n
            CLINK_TOOL_POWER_VOLTAGE_OPTION_0, \n
            CLINK_TOOL_POWER_VOLTAGE_OPTION_12, \n
            CLINK_TOOL_POWER_VOLTAGE_OPTION_24 입력가능
@param[in]  b_sync
            함수의 synchronous 동작 유무를 선택. \n
            0 인 경우 함수는 값을 셋팅하는 명령을 제어SW 에 전송한 뒤 \n
            <STRONG>바로 리턴. 즉, delay없음 </STRONG>\n
            그 외의 값인 경우 함수는 제어SW에 전송한 명령의 \n
            <STRONG>반영을 확인하고 리턴, 즉 100 - 200 ms delay 가능 </STRONG> \n
@remark     셋팅된 voltageOption의 voltage 옵션은 Tool Digital IO 전압의 기준이 된다. \n
            Offline Simulator 및 simulation robot 지원불가.
@return     @ref clink_robot_tool_io_power_voltage_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_tool_io_power_voltage_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_TOOL_IO_POWER_VOLTAGE_OPTION voltage_option,
        char_t b_sync);

/**
@brief      현재 로봇 Tool LED 색상 정보를 받아온다. \n
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_red
            Red 색상 Value [0 ~ 255]
@param[out] p_green
            Green 색상 Value [0 ~ 255]
@param[out] p_blue
            Blue 색상 Value [0 ~ 255]
@return     @ref clink_robot_tool_io_led_color_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_tool_io_led_color_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t* p_red,
        uint32_t* p_green,
        uint32_t* p_blue);

/**
@brief      현재 로봇 Tool LED 색상 값을 설정한다. \n
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  red
            Red 색상 Value [0 ~ 255]
@param[in]  green
            Green 색상 Value [0 ~ 255]
@param[in]  blue
            Blue 색상 Value [0 ~ 255]
@return     @ref clink_robot_tool_io_led_color_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_tool_io_led_color_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t red,
        uint32_t green,
        uint32_t blue);

/**
@brief      Tool IO 모듈의 Software 버전을 리턴한다. \n
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@return     tool io sw version
*/
CLINK_EXPORT const char_t*
    clink_rpc_robot_tool_io_sw_version_get(
        uint32_t cbox_id,
        uint32_t robot_id);

/**
@brief      Tool IO 모듈의 Hardware 버전을 리턴한다. \n
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@return     tool io hw version
*/
CLINK_EXPORT const char_t*
    clink_rpc_robot_tool_io_hw_version_get(
        uint32_t cbox_id,
        uint32_t robot_id);

/**
@brief      Tool IO 모듈의 Hardware 이름을 리턴한다. \n
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@return     tool io hw name
*/
CLINK_EXPORT const char_t*
    clink_rpc_robot_tool_io_hw_name_get(
        uint32_t cbox_id,
        uint32_t robot_id);

/**
@brief      RS485 디바이스 통신을 설정한다. \n
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  serial_interface
            RS485 Serial Interface 설정
@param[in]  baudrate
            RS485 Baudrate 설정
@param[in]  data_frame
            RS485 Data frame 설정
@param[in]  protocol
            RS485 Protocol 설정
@param[in]  tx_continuous_enable
            RS485 Tx Continuous Enable 설정
@param[in]  tx_scan_rate
            RS485 Tx Scan Rate 설정
@return     @ref clink_robot_tool_io_rs485_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_tool_io_rs485_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_RS485_SERIAL_INTERFACE serial_interface,
        CLINK_RS485_BAUDRATE baudrate,
        CLINK_RS485_DATA_FRAME data_frame,
        CLINK_RS485_PROTOCOL protocol,
        CLINK_SWITCH tx_continuous_enable,
        int32_t tx_scan_rate);

/**
@brief      RS485 디바이스 통신을 설정을 가져온다. \n
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out]  serial_interface
            RS485 Serial Interface 설정 값 리턴
@param[out]  baudrate
            RS485 Baudrate 설정 값 리턴
@param[out]  data_frame
            RS485 Data frame 설정 값 리턴
@param[out]  protocol
            RS485 Protocol 설정 값 리턴
@param[out]  tx_continuous_enable
            RS485 Tx Continuous Enable 설정 값 리턴
@param[out]  tx_scan_rate
            RS485 Tx Scan Rate 설정 값 리턴
@return     @ref clink_robot_tool_io_rs485_setup_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_tool_io_rs485_setup_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_RS485_SERIAL_INTERFACE* serial_interface,
        CLINK_RS485_BAUDRATE* baudrate,
        CLINK_RS485_DATA_FRAME* data_frame,
        CLINK_RS485_PROTOCOL* protocol,
        CLINK_SWITCH* tx_continuous_enable,
        int32_t* tx_scan_rate);

/**
@brief      RS485 디바이스 통신 상태값을 확인한다. \n
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out]  p_tx_accepted
            RS485 Tx Request 결과를 확인
@param[out]  p_rx_request
            수신된 데이터 64 byte 초과 시 set (최대 2,048byte까지 연속적인 수신)
@param[out]  p_init_accepted
            RS485 Init Accepted 값 리턴
@param[out]  p_rx_buffer_full
            RX Buffer full 시 set
@param[out]  p_tx_scan_rate_accepted
            RS485 Tx Scan Rate Accepted 값 리턴
@param[out]  p_rx_pause_accepted
            RS485 Rx Pause Accepted 값 리턴
@param[out]  p_rx_done
            RS485 Rx 수신 완료
@param[out]  p_frame_error
            시리얼 통신 프레임 에러 발생 시 set
@param[out]  p_rx_input_length
            수신(입력)된 데이터의 길이 표시(최대 64byte)
@return     @ref clink_robot_tool_io_rs485_status_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_tool_io_rs485_status_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH* p_tx_accepted,
        CLINK_SWITCH* p_rx_request,
        CLINK_SWITCH* p_init_accepted,
        CLINK_SWITCH* p_rx_buffer_full,
        CLINK_SWITCH* p_tx_scan_rate_accepted,
        CLINK_SWITCH* p_rx_pause_accepted,
        CLINK_SWITCH* p_rx_done,
        CLINK_SWITCH* p_frame_error,
        uint8_t* p_rx_input_length);

/**
@brief      RS485 디바이스 통신 상태값을 확인한다. \n
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out]  p_tx_request
            tx_request 설정 값 리턴
@param[out]  p_rx_accepted
            rx_accepted 설정 값 리턴
@param[out]  p_init_request
            init_request 설정 값 리턴
@param[out]  p_tx_send_continuous
            tx_send_continuous 설정 값 리턴
@param[out]  p_tx_scan_rate_request
            tx_scan_rate_request 설정 값 리턴
@param[out]  p_rx_pause_request
            rx_pause_request 설정 값 리턴
@return     @ref clink_robot_tool_io_rs485_control_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_tool_io_rs485_control_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH* p_tx_request,
        CLINK_SWITCH* p_rx_accepted,
        CLINK_SWITCH* p_init_request,
        CLINK_SWITCH* p_tx_send_continuous,
        CLINK_SWITCH* p_tx_scan_rate_request,
        CLINK_SWITCH* p_rx_pause_request);

/**
@brief      RS485 디바이스 Command를 설정한다.  \n
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  cmd_request
            RS485 디바이스 Command
@return     @ref clink_robot_tool_io_rs485_control_cmd_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_tool_io_rs485_control_cmd(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_RS485_CONTROL_CMD cmd_request);

/**
@brief      RS485 디바이스의 Tx 데이터를 가져온다.  \n
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  size_of_data
            TX 데이터를 저장 버퍼 사이즈를 입력
@param[out]  p_val
            TX 데이터 저장 버퍼 포인터
@return     @ref clink_robot_tool_io_rs485_io_tx_data_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_tool_io_rs485_io_tx_data_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint8_t size_of_data,
        uint8_t* p_val);

/**
@brief      RS485 디바이스의 Tx 데이터를 설정한다.  \n
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  size_of_data
            TX 데이터를 저장한 버퍼 사이즈를 입력 (최대 64bytes)
@param[in]  p_val
            TX 데이터를 가지고 있는 버퍼 포인터
@return     @ref clink_robot_tool_io_rs485_io_tx_data_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_tool_io_rs485_io_tx_data_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint8_t size_of_data,
        const uint8_t* p_val);

/**
@brief      RS485 디바이스의 Rx 데이터를 가져온다.  \n
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  size_of_data
            RX 데이터를 저장 버퍼 사이즈를 입력한다. (최대 64bytes)
@param[out]  p_val
            RX 데이터 저장 버퍼 포인터
@return     @ref clink_robot_tool_io_rs485_io_rx_data_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_tool_io_rs485_io_rx_data_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint8_t size_of_data,
        uint8_t* p_val);

/**
@brief      RS485 디바이스 Modbus RTU read를 요청한다.  \n
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  slave_address
            Slave Address 설정
@param[in]  func_code
            Modbus function code 설정
@param[in]  start_address
            Start Address 설정
@param[in]  number_of_data
            Quantity 설정
@param[in]  size_of_data
            Read 할 byte  (최대 64bytes)
@param[out] p_val
            Read 버퍼 포인터
@return     @ref clink_robot_tool_io_rs485_modbus_rtu_read_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_tool_io_rs485_modbus_rtu_read(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint8_t slave_address,
        uint8_t func_code,
        uint16_t start_address,
        uint16_t number_of_data,
        uint8_t size_of_data,
        uint8_t* p_val);

/**
@brief      RS485 디바이스 Modbus RTU write를 요청한다.  \n
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  slave_address
            Slave Address 설정
@param[in]  func_code
            Modbus function code 설정
@param[in]  start_address
            Start Address 설정
@param[in]  number_of_data
            Quantity 설정
@param[in]  size_of_data
            Write 할 byte  (최대 64bytes)
@param[in]  p_val
            Write 할 데이터를 담고 있는 버퍼 포인터
@remark     write 요청 정보 (Address, function Code, Size 등) \n
            write data를 \n
            p_val(메모리)를 통해 전달한다. \n
@return     @ref clink_robot_tool_io_rs485_modbus_rtu_write_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_tool_io_rs485_modbus_rtu_write(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint8_t slave_address,
        uint8_t func_code,
        uint16_t start_address,
        uint16_t number_of_data,
        uint8_t size_of_data,
        const uint8_t* p_val);

/** @} */


//------------------------------------------------------------------
// 320_ROBOT_RECORDING
//------------------------------------------------------------------
/** \addtogroup 320_ROBOT_RECORDING 320_ROBOT_RECORDING
*  @{
*/
/**
@brief      Recording & Playback 을 위한 \n
            Buffer 를 생성한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_buffer_id
            Buffer ID
@remark     기본적으로 30분간 Recording 이 가능한 양의 buffer 를 생성한다.
@return     @ref clink_robot_recording_create_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_recording_create(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t* p_buffer_id);
/**
@brief      Recording & Playback 을 위한 \n
            Buffer 를 이미 저장된 file 로 부터 생성한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  p_file_name
            clink_rpc_robot_recording_file_save() 를 통해 \n
            이전에 저장한 파일명
@param[out] p_buffer_id
            Buffer ID
@return     @ref clink_robot_recording_create_from_file_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_recording_create_from_file(
        uint32_t cbox_id,
        uint32_t robot_id,
        const char_t* p_file_name,
        uint32_t* p_buffer_id);

/**
@brief      Recording & Playback 을 위한 \n
            Buffer 에 할당된 메모리를 해제한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  buffer_id
            clink_rpc_robot_recording_create() 혹은 \n
            clink_rpc_robot_recording_create_from_file() \n
            을 통해 반환받은 값.
@return     @ref clink_robot_recording_destroy_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_recording_destroy(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t buffer_id);

/**
@brief      호출하는 시점부터 로봇의 움직임을 Recording 한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@return     @ref clink_robot_recording_start_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_recording_start(
        uint32_t cbox_id,
        uint32_t robot_id);

/**
@brief      clink_robot_recording_start() 을 통해 시작한 \n
            Recording 을 종료한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  buffer_id
            clink_rpc_robot_recording_create() 혹은 \n
            clink_rpc_robot_recording_create_from_file() \n
            을 통해 반환받은 값.
@return     @ref clink_robot_recording_end_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_recording_end(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t buffer_id);

/**
@brief      에러 체크 등 없이 강제로 Recording 을 종료한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  buffer_id
            clink_rpc_robot_recording_create() 혹은 \n
            clink_rpc_robot_recording_create_from_file() \n
            을 통해 반환받은 값.
@return     @ref clink_robot_recording_end_without_error_check_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_recording_end_without_error_check(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t buffer_id);

/**
@brief      clink_robot_recording_create() 함수는 \n
            30분간 데이터가 저장가능한 사이즈의 메모리를 확보한다. \n
            해당 메모리를 \n
            clink_rpc_robot_recording_start() 함수가 호출된 시점부터 \n
            clink_rpc_robot_recording_end() 함수가 호출된 시점까지의 \n
            데이터만을 저장 수 있게끔 resize 한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  buffer_id
            clink_rpc_robot_recording_create() 혹은 \n
            clink_rpc_robot_recording_create_from_file() \n
            을 통해 반환받은 값.
@return     @ref clink_robot_recording_buffer_memory_optimize_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_recording_buffer_memory_optimize(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t buffer_id);

/**
@brief      로봇 Recording 시, 의도치 않게 \n
            로봇을 움직이지 않은 처음 및 마지막의 \n
            데이터를 불필요한 데이터로 간주하고, \n
            이에 해당하는 데이터를 삭제한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  buffer_id
            clink_rpc_robot_recording_create() 혹은 \n
            clink_rpc_robot_recording_create_from_file() \n
            을 통해 반환받은 값.
@remark     내부적으로 모든 joint 가 0.1 degree 이상 \n
            움직이지 않은 경우, 멈추어 있는 것으로 판단한다.
@return     @ref clink_robot_recording_trim_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_recording_trim(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t buffer_id);

/**
@brief      clink_robot_recording_start() 및 \n
            clink_rpc_robot_recording_end() 를 통해 \n
            Recording 한 데이터를 interpolation 한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  buffer_id
            clink_rpc_robot_recording_create() 혹은 \n
            clink_rpc_robot_recording_create_from_file() \n
            을 통해 반환받은 값.
@param[in]  interpolation_name
            interpolation algorithm을 선택한다.\n
            CLINK_RECORDING_INTERPOLATION_TCP 혹은 \n
            CLINK_RECORDING_INTERPOLATION_JOINT \n
            현재 <STRONG>CLINK_RECORDING_INTERPOLATION_JOINT 지원하지 않음 </STRONG>
@param[in]  sharpness
            interpolation algorithm parameter \n
            <STRONG>현재는 2 만 지원된다. \n
            사용자 입력 파라메터는 무시한다. </STRONG>
@param[in]  speed_factor
            recording된 data를 기준으로 속도비율에 따라 interpolation을 실행한다.\n
            범위는 0.1 ~ 10.0, 아래의 값 중 하나를 입력 \n
            [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
@param[out] p_interpolated_buffer
            interpolation 결과가 저장되는 버퍼 \n
            사용을 종료한 후, clink_robot_recording_destroy() \n
            를 통해 삭제해 주어야 한다.
@return     @ref clink_robot_recording_interpolate_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_recording_interpolate(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t buffer_id,
        CLINK_RECORDING_INTERPOLATION interpolation_name,
        uint32_t sharpness,
        clink_float_t speed_factor,
        uint32_t* p_interpolated_buffer);

/**
@brief      Buffer 에 저장된 데이터를 Playback 한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  buffer_id
            clink_rpc_robot_recording_interpolate() 를 통해 \n
            interpolation 을 한 buffer 의 ID
@param[out] p_command_id
            path recording motion의 command id
@remark     기본적으로 interpolation 하지 않은 buffer \n
            도 playback 은 가능하다. 단, 이 경우 \n
            Record 된 데이터는 매우 거칠기 때문에 \n
            로봇 기구부에 손상, \n
            혹은 의도치 않은 로봇 드라이브 에러, \n
            충돌감지 기능의 오동작이 발생할 수 있다.
@return     @ref clink_robot_recording_playback_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_recording_playback(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t buffer_id,
        uint32_t* p_command_id);

/**
@brief      Buffer 의 데이터를 파일로 저장한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  buffer_id
            clink_rpc_robot_recording_create(), \n
            clink_rpc_robot_recording_create_from_file(), \n
            clink_rpc_robot_recording_interpolate() 등을 통해 \n
            반환받은 값.
@param[in]  p_file_name
            저장 파일명
@return     @ref clink_robot_recording_file_save_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_recording_file_save(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t buffer_id,
        const char_t* p_file_name);

/**
@brief      Record 된 데이터의 시작 위치의 각 Joint Angle 을 반환한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  buffer_id
            clink_rpc_robot_recording_create(), \n
            clink_rpc_robot_recording_create_from_file(), \n
            clink_rpc_robot_recording_interpolate() 등을 통해 \n
            반환받은 값.
@param[in]  num_of_joints
            p_angles 배열의 크기 \n
            로봇의 joint 개수 이상이어야 한다. \n
@param[out] p_angles
            각 Joint 의 Angle
@return     @ref clink_robot_recording_angle_start_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_recording_angle_start_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t buffer_id,
        uint32_t num_of_joints,
        clink_float_t* p_angles);

/**
@brief      Record 된 데이터의 종료 위치의 각 Joint Angle 을 반환한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  buffer_id
            clink_rpc_robot_recording_create(), \n
            clink_rpc_robot_recording_create_from_file(), \n
            clink_rpc_robot_recording_interpolate() 등을 통해 \n
            반환받은 값.
@param[in]  num_of_joints
            p_angles 배열의 크기 \n
            로봇의 joint 개수 이상이어야 한다. \n
@param[out] p_angles
            각 Joint 의 Angle
@return     @ref clink_robot_recording_angle_end_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_recording_angle_end_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t buffer_id,
        uint32_t num_of_joints,
        clink_float_t* p_angles);

/**
@brief      Record 된 데이터의 시작 위치의 각 Joint Angle 을 반환한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  p_file_name
            clink_rpc_robot_recording_file_save() \n
            를 통해 저장한 파일명
@param[in]  num_of_joints
            p_angles 배열의 크기 \n
            로봇의 joint 개수 이상이어야 한다. \n
@param[out] p_angles
            각 Joint 의 Angle
@return     @ref clink_robot_recording_angle_start_from_file_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_recording_angle_start_from_file_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        const char_t* p_file_name,
        uint32_t num_of_joints,
        clink_float_t* p_angles);

/**
@brief      Record 된 데이터의 종료 위치의 각 Joint Angle 을 반환한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  p_file_name
            clink_rpc_robot_recording_file_save() \n
            를 통해 저장한 파일명
@param[in]  num_of_joints
            p_angles 배열의 크기 \n
            로봇의 joint 개수 이상이어야 한다. \n
@param[out] p_angles
            각 Joint 의 Angle
@return     @ref clink_robot_recording_angle_end_from_file_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_recording_angle_end_from_file_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        const char_t* p_file_name,
        uint32_t num_of_joints,
        clink_float_t* p_angles);

/** @} */


//------------------------------------------------------------------
// 321_ROBOT_EXT
//------------------------------------------------------------------
/** \addtogroup 321_ROBOT_EXT 321_ROBOT_EXT
*  @{
*/

/**
@brief      External Profiler 현재 버퍼 수를 반환한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_size
            버퍼 사이즈
@remark     버퍼는 3세대 기준 500us 마다 하나씩 소모된다.
@return     @ref clink_robot_ext_profiler_buffer_size_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_ext_profiler_buffer_size_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t* p_size);

/**
@brief      External Profiler 현재 전달 주기를 반환한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_period
            전달 주기 [ms]
@return     @ref clink_robot_ext_profiler_period_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_ext_profiler_period_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t* p_period);

/**
@brief      External Profiler 현재 전달 주기를 설정한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  period
            전달 주기 [ms] 초기 값은 5ms
@return     @ref clink_robot_ext_profiler_period_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_ext_profiler_period_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t period);

/**
@brief      External Profiler 로 입력하는 좌표가 Flange 인지 TCP 단인지 설정 상태를 받아온다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_type
            FLANGE or TCP 설정 값
@return     @ref clink_robot_ext_profiler_tcp_type_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_ext_profiler_tcp_type_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_POSE_TYPE* p_type);

/**
@brief      External Profiler 로 입력하는 좌표가 Flange 인지 TCP 단인지 설정한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  type
            FLANGE or TCP 설정
@remark     기본 값은 CLINK_POSE_TYPE_FLANGE
@return     @ref clink_robot_ext_profiler_tcp_type_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_ext_profiler_tcp_type_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_POSE_TYPE type);

/**
@brief      로봇의 현재 설정된 External Profiler 모드를 반환한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_on_off
            CLINK_SWITCH_ON : External Profiler 모드 ON \n
            CLINK_SWITCH_OFF : External Profiler OFF
@return     @ref clink_robot_ext_profiler_switch_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_ext_profiler_switch_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH* p_on_off);

/**
@brief      로봇의 External Profiler  모드를 enable / disable 한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  on_off
            CLINK_SWITCH_ON : External Profiler  모드 ON \n
            CLINK_SWITCH_OFF : External Profiler OFF
@remark     External Profiler 기능은 이미 어떤 다른 SW 등을 통해 \n
            명확히 profile 된 데이터를 전송해야 한다. \n
            현재 제어주기는 10 ms 로 되어 있다. \n
            따라서, 10 ms 를 제어주기로 하는 profile 데이터를 생성한 후, \n
            이 함수를 호출한 후에는 \n
            clink_rpc_robot_ext_profiler_pos_ort_euler_add() 또는 \n
            clink_rpc_robot_ext_profiler_pos_ort_quat_add() 를 \n
            매 10 ms 마다 반드시 호출해줘야 한다.
@return     @ref clink_robot_ext_profiler_switch_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_ext_profiler_switch_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH on_off);

/**
@brief      External Profiler 모드로서, 로봇에 전달할 command position 및 \n
            orientation 성분을 전송한다. \n
            <STRONG> 이 함수는 정해진 시간 (5 ms) 에 정확히 맞추어 호출되어야 한다. </STRONG>
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  pos_x
            Position 데이터의 X 성분, base 좌표계에서 TCP 위치 기준
@param[in]  pos_y
            Position 데이터의 Y 성분, base 좌표계에서 TCP 위치 기준
@param[in]  pos_z
            Position 데이터의 Z 성분, base 좌표계에서 TCP 위치 기준
@param[in]  ort_x
            Orientation 의 X 성분, base 좌표계에서 TCP 기준, degree 값 (즉, radian 아님)
@param[in]  ort_y
            Orientation 의 Y 성분, base 좌표계에서 TCP 기준, degree 값 (즉, radian 아님)
@param[in]  ort_z
            Orientation 의 Z 성분, base 좌표계에서 TCP 기준, degree 값 (즉, radian 아님)
@remark     같은 위치를 계속 추가할 경우 로봇 이동하지 않고, \n
            해당 위치를 지속적으로 유지한다.  \n
            5ms 간격으로 추가되는 로봇의 위치는 속도 및 가속도가 \n
            적용되는 자체 프로파일러를 사용하여 생성할 것을 권장한다. \n
            로봇은 각 Joint마다 속도 및 가속도 제한이 있으며(제한 위반시 stop), \n
            모두 불연속한 부분없이 부드럽게 이어져야 한다. \n
            3개 이상의 point 를 입력한 순간부터 모션이 시작되며, \n
            시작 시 ADD한 모든 point는 반영된다. \n
            (5ms 간격의 point들을 내부 제어 주기에 맞게 \n
            interpolation 하는 과정에 최소 3개 이상의 point가 필요)
@return     @ref clink_robot_ext_profiler_pos_ort_euler_add_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_ext_profiler_pos_ort_euler_add(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t pos_x,
        clink_float_t pos_y,
        clink_float_t pos_z,
        clink_float_t ort_x,
        clink_float_t ort_y,
        clink_float_t ort_z);

/**
@brief      clink_robot_ext_profiler_pos_ort_euler_add() 와 동일 \n
            단, Orientation 입력 시 euler angle 이 아닌 \n
            Quaternion 사용.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  pos_x
            position x 좌표
@param[in]  pos_y
            position y 좌표
@param[in]  pos_z
            position z 좌표
@param[in]  quat_w
            quaternion w 성분
@param[in]  quat_x
            quaternion x 성분
@param[in]  quat_y
            quaternion y 성분
@param[in]  quat_z
            quaternion z 성분
@remark     clink_robot_ext_profiler_pos_ort_euler_add() remark 참조.
@return     @ref clink_robot_ext_profiler_pos_ort_quat_add_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_ext_profiler_pos_ort_quat_add(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t pos_x,
        clink_float_t pos_y,
        clink_float_t pos_z,
        clink_float_t quat_w,
        clink_float_t quat_x,
        clink_float_t quat_y,
        clink_float_t quat_z);

/**
@brief      External Profiler 모드로서, 로봇에 전달할 joint angles를 전송한다.
            <STRONG> 이 함수는 정해진 시간 (5 ms) 에 정확히 맞추어 호출되어야 한다. </STRONG>
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  num_of_joints
            robot의 joint 갯수
@param[in]  angles
            로봇의 angle 배열
@remark     clink_robot_ext_profiler_pos_ort_euler_add() remark 참조.
@return     @ref clink_robot_ext_profiler_joint_add_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_ext_profiler_joint_add(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t num_of_joints,
        const clink_float_t* angles);

/**
@brief      로봇의 현재 설정된 External Move 모드를 반환한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_on_off
            CLINK_SWITCH_ON : External Move 모드 ON \n
            CLINK_SWITCH_OFF : External Move OFF
@return     @ref clink_robot_ext_move_switch_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_ext_move_switch_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH* p_on_off);

/**
@brief      로봇의 External Move 모드를 enable / disable 한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  on_off
            CLINK_SWITCH_ON : External Move 모드 ON \n
            CLINK_SWITCH_OFF : External Move 모드 OFF
@remark     External Move 는 주어진 target position 으로 이동한다. \n
            사용자는 profile 등에 대한 고려 없이 원하는 위치의 target 만을 \n
            전송하면 로봇 제어기가 알아서 해당 위치로 이동하는 것이 \n
            external profiler 와 다르다. \n
            Profile 된 데이터가 입수 가능하고 세밀한 제어를 원하는 경우 External Profiler 를 사용하고, \n
            Path 속도 등이 중요하지 않고, 편하게 어떤 위치로 이동하고 싶은 경우에는 External Move 를 사용한다.
@return     @ref clink_robot_ext_move_switch_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_robot_ext_move_switch_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH on_off);

/**
@brief      External Move 기능에서 설정된 stop mode를 반환한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_on_off
            CLINK_SWITCH_ON : stop mode ON \n
            CLINK_SWITCH_OFF : stop mode OFF
@remark     blending motion의 stop mode와 동일한 기능이다(default ON).
@return     @ref clink_robot_ext_move_stop_mode_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_ext_move_stop_mode_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH* p_on_off);

/**
@brief      External Move 기능에서 stop mode를 설정한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  on_off
            CLINK_SWITCH_ON : stop mode ON \n
            CLINK_SWITCH_OFF : stop mode OFF
@remark     blending motion의 stop mode와 동일한 기능이다(default ON).
@return     @ref clink_robot_ext_move_stop_mode_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_ext_move_stop_mode_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH on_off);

/**
@brief      주어진 위치로 로봇을 이동시킨다. (external move)
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  type
            입력 pose 가 flange 위치인지 tcp 위치 인지 결정
@param[in]  pos_x
            position x 좌표
@param[in]  pos_y
            position y 좌표
@param[in]  pos_z
            position z 좌표
@param[in]  ort_x
            orientation x 좌표
@param[in]  ort_y
            orientation y 좌표
@param[in]  ort_z
            orientation z 좌표
@param[in]  vel
            속도 [mm/s]
@return     @ref clink_robot_ext_move_pos_ort_euler_add_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_ext_move_pos_ort_euler_add(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_POSE_TYPE type,
        clink_float_t pos_x,
        clink_float_t pos_y,
        clink_float_t pos_z,
        clink_float_t ort_x,
        clink_float_t ort_y,
        clink_float_t ort_z,
        clink_float_t vel);

/**
@brief      주어진 위치로 로봇을 이동시킨다. (external move)
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  type
            입력 pose 가 flange 위치인지 tcp 위치 인지 결정
@param[in]  pos_x
            position x 좌표
@param[in]  pos_y
            position y 좌표
@param[in]  pos_z
            position z 좌표
@param[in]  quat_w
            quaternion w
@param[in]  quat_x
            quaternion x
@param[in]  quat_y
            quaternion y
@param[in]  quat_z
            quaternion z
@param[in]  vel
            속도 [mm/s]
@return     @ref clink_robot_ext_move_pos_ort_quat_add_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_ext_move_pos_ort_quat_add(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_POSE_TYPE type,
        clink_float_t pos_x,
        clink_float_t pos_y,
        clink_float_t pos_z,
        clink_float_t quat_w,
        clink_float_t quat_x,
        clink_float_t quat_y,
        clink_float_t quat_z,
        clink_float_t vel);

/**
@brief      주어진 위치로 로봇을 이동시킨다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  num_of_joints
            p_angles 의 배열 크기. robot 의 조인트 개수와 동일해야 한다.
@param[in]  p_angles
            각 joint 의 target angle 을 저장한 배열
@param[in]  vel
            Joint 의 각속도
@return     @ref clink_robot_ext_move_joint_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_ext_move_joint(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t num_of_joints,
        const clink_float_t* p_angles,
        clink_float_t vel);

/** @} */


//------------------------------------------------------------------
// 322_ROBOT_FORCE_CTRL
//------------------------------------------------------------------
/** \addtogroup 322_ROBOT_FORCE_CTRL 322_ROBOT_FORCE_CTRL
*  @{
*/

/**
@brief      로봇의 현재 Force Control 기능의 enable / disable 상태를 반환한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_on_off
            CLINK_SWITCH_ON : Force Control 모드 ON \n
            CLINK_SWITCH_OFF : Force Control 모드 OFF
@return     @ref clink_robot_force_ctrl_switch_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_ctrl_switch_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH* p_on_off);

/**
@brief      로봇의 Force Control 기능을 enable / disable 한다.
@param[in]  robot_id	
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  on_off
            CLINK_SWITCH_ON : Force Control 모드 ON  \n
            CLINK_SWITCH_OFF : Force Control 모드 OFF
@return     @ref clink_robot_force_ctrl_switch_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_ctrl_switch_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH on_off);

/**
@brief      로봇에 설정된 Force Control 을 적용하는 기준 좌표계를 반환한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_ref_coord
            Force Control 을 적용하는 기준 좌표계  \n
            CLINK_REF_COORDINATE_BASE 또는, \n
            CLINK_REF_COORDINATE_TCP
@return     @ref clink_robot_force_ctrl_coordinate_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_ctrl_coordinate_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_REF_COORDINATE* p_ref_coord);

/**
@brief      로봇에 설정된 Force Control 을 적용하는 기준 좌표계를 설정한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  ref_coord
            Force Control 을 적용하는 기준 좌표계  \n
            CLINK_REF_COORDINATE_BASE 또는, \n
            CLINK_REF_COORDINATE_TCP  \n
@remark		해당 설정 값은 모두 base 좌표계 기준 \n
            clink_rpc_robot_force_ctrl_coordinate_set()에서 TCP 좌표계 기준으로 변경 시, TCP기준으로 변경  \n
@return     @ref clink_robot_force_ctrl_coordinate_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_ctrl_coordinate_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_REF_COORDINATE ref_coord);

/**
@brief      로봇에 설정된 Force Control 기능의 Force 및 Torque 를 반환한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_fx 
            X 축 에 대한 force 값 (단위 : Newton [N])
@param[out] p_fy 
            Y 축 에 대한 force 값 (단위 : Newton [N])
@param[out] p_fz 
            Z 축 에 대한 force 값 (단위 : Newton [N])
@param[out] p_tx 
            X 축 에 대한 torque 값 (단위 : Newton-meter [Nm])
@param[out] p_ty 
            Y 축 에 대한 torque 값 (단위 : Newton-meter [Nm])
@param[out] p_tz 
            Z 축 에 대한 torque 값 (단위 : Newton-meter [Nm])
@return     @ref clink_robot_force_ctrl_force_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_ctrl_force_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t* p_fx,
        clink_float_t* p_fy,
        clink_float_t* p_fz,
        clink_float_t* p_tx,
        clink_float_t* p_ty,
        clink_float_t* p_tz);

/**
@brief      로봇에 Force Control 기능의 Force 및 Torque 를 설정한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  fx
            X 축 에 대한 force 값 (단위 : Newton [N])
@param[in]  fy
            Y 축 에 대한 force 값 (단위 : Newton [N])
@param[in]  fz
            Z 축 에 대한 force 값 (단위 : Newton [N])
@param[in]  tx
            X 축 에 대한 torque 값 (단위 : Newton-meter [Nm])
@param[in]  ty
            Y 축 에 대한 torque 값 (단위 : Newton-meter [Nm])
@param[in]  tz
            Z 축 에 대한 torque 값 (단위 : Newton-meter [Nm])
@remark     해당 설정 값은 모두 base 좌표계 기준 \n
            clink_rpc_robot_force_ctrl_coordinate_set()에서 TCP 좌표계 기준으로 변경 시, TCP기준으로 변경  \n
            사용 가능한 force 범위: -120N<force<+120N  \n
            사용 가능한 torque 범위: -50Nm<torque<+50Nm  \n
            +/- 극성 설정으로 force/torque 방향 전환 가능  \n
            기본 설정 값 fx:10N  \n
            기본 설정 값 fy:10N  \n
            기본 설정 값 fz:10N  \n
            기본 설정 값 tx:5Nm  \n
            기본 설정 값 ty:5Nm  \n
            기본 설정 값 tz:5Nm  \n
@return     @ref clink_robot_force_ctrl_force_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_ctrl_force_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t fx,
        clink_float_t fy,
        clink_float_t fz,
        clink_float_t tx,
        clink_float_t ty,
        clink_float_t tz);

/**
@brief      로봇에 Force Control 기능의 Velocity 를 반환한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_vx
            X 축 에 대한 velocity 값 (단위 : millimeter / second)
@param[out] p_vy 
            Y 축 에 대한 velocity 값 (단위 : millimeter / second)
@param[out] p_vz 
            Z 축 에 대한 velocity 값 (단위 : millimeter / second)
@param[out] p_rx 
            X 축 에 대한 angular velocity 값 (단위 : degree / second)
@param[out] p_ry 
            Y 축 에 대한 angular velocity 값 (단위 : degree / second)
@param[out] p_rz 
            Z 축 에 대한 angular velocity 값 (단위 : degree / second)
@return     @ref clink_robot_force_ctrl_velocity_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_ctrl_velocity_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t* p_vx,
        clink_float_t* p_vy,
        clink_float_t* p_vz,
        clink_float_t* p_rx,
        clink_float_t* p_ry,
        clink_float_t* p_rz);

/**
@brief      로봇에 Force Control 기능의 Velocity 를 설정한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  vx
            X 축 에 대한 velocity 값 (단위 : millimeter / second)
@param[in]  vy 
            Y 축 에 대한 velocity 값 (단위 : millimeter / second)
@param[in]  vz
            Z 축 에 대한 velocity 값 (단위 : millimeter / second)
@param[in]  rx 
            X 축 에 대한 angular velocity 값 (단위 : degree / second)
@param[in]  ry
            Y 축 에 대한 angular velocity 값 (단위 : degree / second)
@param[in]  rz 
            Z 축 에 대한 angular velocity 값 (단위 : degree / second)
@remark     해당 설정 값은 모두 base 좌표계 기준 \n
            clink_rpc_robot_force_ctrl_coordinate_set()에서 TCP 좌표계 기준으로 변경 시, TCP기준으로 변경 \n
            사용 가능한 velocity(vx, vy, vz) 범위: 0<velocity<500 \n
            사용 가능한 velocity(rx, ry, rz) 범위: 0<velocity<180 \n
            기본 설정 값 vx, vy, vz: 50mm/s  \n
            기본 설정 값 rx, ry, rz: 5degree/s  \n
@return     @ref clink_robot_force_ctrl_velocity_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_ctrl_velocity_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t vx,
        clink_float_t vy,
        clink_float_t vz,
        clink_float_t rx,
        clink_float_t ry,
        clink_float_t rz);

/**
@brief      로봇에 Force Control 기능의 각 Direction 에 대한 on / off 를 반환한다. 
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_x_on_off 
            X 축 방향에 대한 설정
@param[out] p_y_on_off 
            Y 축 방향에 대한 설정
@param[out] p_z_on_off
            Z 축 방향에 대한 설정
@param[out] p_rx_on_off 
            X 축 방향의 rotation 에 대한 설정
@param[out] p_ry_on_off 
            Y 축 방향의 rotation 에 대한 설정
@param[out] p_rz_on_off 
            Z 축 방향의 rotation 에 대한 설정
@return     @ref clink_robot_force_ctrl_direction_switch_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_ctrl_direction_switch_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH* p_x_on_off,
        CLINK_SWITCH* p_y_on_off,
        CLINK_SWITCH* p_z_on_off,
        CLINK_SWITCH* p_rx_on_off,
        CLINK_SWITCH* p_ry_on_off,
        CLINK_SWITCH* p_rz_on_off);

/**
@brief      로봇에 Force Control 기능의 각 Direction 에 대한 on / off 를 설정한다. 
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  x_on_off
            X 축 방향에 대한 설정
@param[in]  y_on_off
            Y 축 방향에 대한 설정
@param[in]  z_on_off
            Z 축 방향에 대한 설정
@param[in]  rx_on_off
            X 축 방향의 rotation 에 대한 설정
@param[in]  ry_on_off
            Y 축 방향의 rotation 에 대한 설정
@param[in]  rz_on_off
            Z 축 방향의 rotation 에 대한 설정
@remark     해당 설정 값은 모두 base 좌표계 기준 \n
            clink_rpc_robot_force_ctrl_coordinate_set()에서 TCP 좌표계 기준으로 변경 시, TCP기준으로 변경  \n
@return     @ref clink_robot_force_ctrl_direction_switch_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_ctrl_direction_switch_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH x_on_off,
        CLINK_SWITCH y_on_off,
        CLINK_SWITCH z_on_off,
        CLINK_SWITCH rx_on_off,
        CLINK_SWITCH ry_on_off,
        CLINK_SWITCH rz_on_off);

/**
@brief      로봇에 Force Control 기능의 각 Direction 에 acceleration 을 반환한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_x_acc 
            X 축 방향에 대한 설정, 단위 : millimeter / second^2
@param[out] p_y_acc 
            Y 축 방향에 대한 설정, 단위 : millimeter / second^2
@param[out] p_z_acc 
            Z 축 방향에 대한 설정, 단위 : millimeter / second^2
@param[out] p_rx_acc 
            X 축 방향의 rotation 에 대한 설정, 단위 : degree / second^2
@param[out] p_ry_acc 
            Y 축 방향의 rotation 에 대한 설정, 단위 : degree / second^2
@param[out] p_rz_acc 
            Z 축 방향의 rotation 에 대한 설정, 단위 : degree / second^2
@return     @ref clink_robot_force_ctrl_acc_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_ctrl_acc_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t* p_x_acc,
        clink_float_t* p_y_acc,
        clink_float_t* p_z_acc,
        clink_float_t* p_rx_acc,
        clink_float_t* p_ry_acc,
        clink_float_t* p_rz_acc);

/**
@brief      로봇에 Force Control 기능의 각 Direction 에 acceleration 을 설정한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  x_acc 
            X 축 방향에 대한 설정, 단위 : millimeter / second^2
@param[in]  y_acc 
            Y 축 방향에 대한 설정, 단위 : millimeter / second^2
@param[in]  z_acc 
            Z 축 방향에 대한 설정, 단위 : millimeter / second^2
@param[in]  rx_acc 
            X 축 방향의 rotation 에 대한 설정, 단위 : degree / second^2
@param[in]  ry_acc 
            Y 축 방향의 rotation 에 대한 설정, 단위 : degree / second^2
@param[in]  rz_acc 
            Z 축 방향의 rotation 에 대한 설정, 단위 : degree / second^2
@remark     해당 설정 값은 모두 base 좌표계 기준 \n
            clink_rpc_robot_force_ctrl_coordinate_set()에서 TCP 좌표계 기준으로 변경 시, TCP기준으로 변경  \n
            사용 가능한 acceleration(x_acc, y_acc, z_acc) 범위: 0<acceleration<10000  \n
            사용 가능한 acceleration(rx_acc, ry_acc, rz_acc) 범위: 0<acceleration<3600  \n
            기본 설정 값 x_acc, y_acc, z_acc: 100mm/s^2  \n
            기본 설정 값 rx_acc, ry_acc, rz_acc: 50degree/s^2  \n
            Force control velocity 설정의 10배로 가속도 설정 권장  \n
            ex)vx=vy=vz=10mm/s일 때, x_acc=y_acc=z_acc=100mm/s^2  \n
            rx=ry=rz=5degree/s일 때, rx_acc=ry_acc=rz_acc=50degree/s^2로 설정  \n
@return     @ref clink_robot_force_ctrl_acc_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_ctrl_acc_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t x_acc,
        clink_float_t y_acc,
        clink_float_t z_acc,
        clink_float_t rx_acc,
        clink_float_t ry_acc,
        clink_float_t rz_acc);

/**
@brief      로봇에 Force Control 기능의 각 Direction 에 deceleration 을 반환한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_x_dec 
            X 축 방향에 대한 설정, 단위 : millimeter / second^2
@param[out] p_y_dec 
            Y 축 방향에 대한 설정, 단위 : millimeter / second^2
@param[out] p_z_dec 
            Z 축 방향에 대한 설정, 단위 : millimeter / second^2
@param[out] p_rx_dec 
            X 축 방향의 rotation 에 대한 설정, 단위 : degree / second^2
@param[out] p_ry_dec 
            Y 축 방향의 rotation 에 대한 설정, 단위 : degree / second^2
@param[out] p_rz_dec 
            Z 축 방향의 rotation 에 대한 설정, 단위 : degree / second^2
@return     @ref clink_robot_force_ctrl_dec_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_ctrl_dec_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t* p_x_dec,
        clink_float_t* p_y_dec,
        clink_float_t* p_z_dec,
        clink_float_t* p_rx_dec,
        clink_float_t* p_ry_dec,
        clink_float_t* p_rz_dec);

/**
@brief      로봇에 Force Control 기능의 각 Direction 에 deceleration 을 설정한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  x_dec 
            X 축 방향에 대한 설정, 단위 : millimeter / second^2
@param[in]  y_dec 
            Y 축 방향에 대한 설정, 단위 : millimeter / second^2
@param[in]  z_dec 
            Z 축 방향에 대한 설정, 단위 : millimeter / second^2
@param[in]  rx_dec
            X 축 방향의 rotation 에 대한 설정, 단위 : degree / second^2
@param[in]  ry_dec 
            Y 축 방향의 rotation 에 대한 설정, 단위 : degree / second^2
@param[in]  rz_dec 
            Z 축 방향의 rotation 에 대한 설정, 단위 : degree / second^2
@remark     해당 설정 값은 모두 base 좌표계 기준 \n
            clink_rpc_robot_force_ctrl_coordinate_set()에서 TCP 좌표계 기준으로 변경 시, TCP기준으로 변경  \n
            사용 가능한 deceleration(x_dec, y_dec, z_dec) 범위: 0<deceleration<10000  \n
            사용 가능한 deceleration(rx_dec, ry_dec, rz_dec) 범위: 0<deceleration<3600  \n
            기본 설정 값 x_dec, y_dec, z_dec: 100mm/s^2  \n
            기본 설정 값 rx_dec, ry_dec, rz_dec: 50degree/s^2  \n
            force control velocity 설정의 10배로 가속도 설정 권장  \n
            ex)vx=vy=vz=10mm/s일 때, x_dec=y_dec=z_dec=100mm/s^2  \n
            rx=ry=rz=5degree/s일 때, rx_dec=ry_dec=rz_dec=50degree/s^2로 설정  \n
@return     @ref clink_robot_force_ctrl_dec_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_ctrl_dec_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t x_dec,
        clink_float_t y_dec,
        clink_float_t z_dec,
        clink_float_t rx_dec,
        clink_float_t ry_dec,
        clink_float_t rz_dec);

/**
@brief      로봇에 Force Control 기능의 damping ratio를 반환한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_x
			x축 damping ratio 값 [Ns/m]
@param[out] p_y
			y축 damping ratio 값 [Ns/m]
@param[out] p_z
			z축 damping ratio 값 [Ns/m]
@param[out] p_rx
			rx damping ratio 값 [Nms/degree]
@param[out] p_ry
			ry damping ratio 값 [Nms/degree]
@param[out] p_rz
			rz damping ratio 값 [Nms/degree]
@return     @ref clink_robot_force_ctrl_damping_ratio_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_ctrl_damping_ratio_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t* p_x,
        clink_float_t* p_y,
        clink_float_t* p_z,
        clink_float_t* p_rx,
        clink_float_t* p_ry,
        clink_float_t* p_rz);

/**
@brief      로봇에 Force Control 기능의 damping ratio를 설정한다. \n
            로봇이 사물에 접촉하는 순간 안정적으로 접촉하도록 응답 특징을 조정한다. \n
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  x
			x축 damping ratio 값 [Ns/m]
@param[in]  y
			y축 damping ratio 값 [Ns/m]
@param[in]  z
            z축 damping ratio 값 [Ns/m]
@param[in]  rx
			rx damping ratio 값 [Nms/degree]
@param[in]  ry
			ry damping ratio 값 [Nms/degree]
@param[in]  rz
			rz damping ratio 값 [Nms/degree]
@remark		해당 설정 값은 모두 base 좌표계 기준 \n
            clink_rpc_robot_force_ctrl_coordinate_set()에서 TCP 좌표계 기준으로 변경 시, TCP기준으로 변경  \n
            damping ratio 물리적 단위 없음  \n
            사용 가능한 damping ratio(x, y, z) 범위: 0<p_xyz<1000  \n
            사용 가능한 damping ratio(rx, ry, rz) 범위: 0<p_r_xyz<1000  \n
            damping ratio가 높아질 수록, 반응속도가 느려지나, 접촉안정성이 높아짐(강성이 높은 접촉면에 안정적으로   \n
            접촉 가능. 하지만, 너무 높이게 되면 반응속도가 느려져 원하는 모션을 하는데 한계가 있음)  \n
            damping ratio가 낮아질 수록, 반응속도가 빨라지나, 접촉안정성이 낮아짐(조금 더 빠른 반응속도를 원한다면,  \n
            damping ratio를 낮출 수 있으나, 강성이 높은 물체와 접촉할 때, 진동 할 수 있는 여지가 있음)  \n
@return     @ref clink_robot_force_ctrl_damping_ratio_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_ctrl_damping_ratio_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t x,
        clink_float_t y,
        clink_float_t z,
        clink_float_t rx,
        clink_float_t ry,
        clink_float_t rz);

/**
@brief      로봇에 Force Control 기능의 inertia 성분을 반환한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_x
            x축 inertia constant value [Ns^2/m]
@param[out] p_y
            y축 inertia constant value [Ns^2/m]
@param[out] p_z
            z축 inertia constant value [Ns^2/m]
@param[out] p_rx
            rx inertia constant value [Nms^2/degree]
@param[out] p_ry
            ry inertia constant value [Nms^2/degree]
@param[out] p_rz
            rz inertia constant value [Nms^2/degree]
@return     @ref clink_robot_force_ctrl_inertia_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_ctrl_inertia_get(
        uint32_t cbox_id,
	    uint32_t robot_id,
	    clink_float_t* p_x,
	    clink_float_t* p_y,
	    clink_float_t* p_z,
	    clink_float_t* p_rx,
	    clink_float_t* p_ry,
	    clink_float_t* p_rz);

/**
@brief      로봇에 Force Control 기능의 inertia constant 를 설정한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  x
            x축 inertia constant value [Ns^2/m]
@param[in]  y
            y축 inertia constant value [Ns^2/m]
@param[in]  z
            z축 inertia constant value [Ns^2/m]
@param[in]  rx
            rx inertia constant value [Nms^2/degree]
@param[in]  ry
            ry inertia constant value [Nms^2/degree]
@param[in]  rz
            rz inertia constant value [Nms^2/degree]
@return     @ref clink_robot_force_ctrl_inertia_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_ctrl_inertia_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t x,
        clink_float_t y,
        clink_float_t z,
        clink_float_t rx,
        clink_float_t ry,
        clink_float_t rz);

/**
@brief      로봇에 Force Control 기능의 approach velocity 성분을 반환한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_x
            x축 approach velocity [mm/s]
@param[out] p_y
            y축 approach velocity [mm/s]
@param[out] p_z
            z축 approach velocity [mm/s]
@param[out] p_rx
            rx approach velocity [degree/s]
@param[out] p_ry
            ry approach velocity [degree/s]
@param[out] p_rz
            rz approach velocity [degree/s]
@return     @ref clink_robot_force_ctrl_approach_velocity_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_ctrl_approach_velocity_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t* p_x,
        clink_float_t* p_y,
        clink_float_t* p_z,
        clink_float_t* p_rx,
        clink_float_t* p_ry,
        clink_float_t* p_rz);

/**
@brief      로봇에 Force Control 기능의 approach velocity 를 설정한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  x
            x축 approach velocity [mm/s]
@param[in]  y
            y축 approach velocity [mm/s]
@param[in]  z
            z축 approach velocity [mm/s]
@param[in]  rx
            rx approach velocity [degree/s]
@param[in]  ry
            ry approach velocity [degree/s]
@param[in]  rz
            rz approach velocity [degree/s]
@return     @ref clink_robot_force_ctrl_approach_velocity_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_ctrl_approach_velocity_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t x,
        clink_float_t y,
        clink_float_t z,
        clink_float_t rx,
        clink_float_t ry,
        clink_float_t rz);

/**
@brief      로봇에 Force Control 기능의 stiffness를 반환한다. \n
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_x
            X축 stiffness 값, 단위 : Newton / meter
@param[out] p_y
            Y축 stiffness 값, 단위 : Newton / meter
@param[out] p_z
            Z축 stiffness 값, 단위 : Newton / meter
@param[out] p_rx
            X축 Rotation 의 stiffness 값, 단위 : Newton-meter / degree
@param[out] p_ry
            Y축 Rotation 의 stiffness 값, 단위 : Newton-meter / degree
@param[out] p_rz
            Z축 Rotation 의 stiffness 값, 단위 : Newton-meter / degree
@return     @ref clink_robot_force_ctrl_stiffness_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_ctrl_stiffness_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t* p_x,
        clink_float_t* p_y,
        clink_float_t* p_z,
        clink_float_t* p_rx,
        clink_float_t* p_ry,
        clink_float_t* p_rz);

/**
@brief      로봇에 Force Control 기능의 stiffness를 설정한다. \n
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  x
            X축 stiffness 값, 단위 : Newton / meter
@param[in]  y
            Y축 stiffness 값, 단위 : Newton / meter
@param[in]  z
            Z축 stiffness 값, 단위 : Newton / meter
@param[in]  rx
            X축 Rotation 의 stiffness 값, 단위 : Newton-meter / degree
@param[in]  ry
            Y축 Rotation 의 stiffness 값, 단위 : Newton-meter / degree
@param[in]  rz
            Z축 Rotation 의 stiffness 값, 단위 : Newton-meter / degree
@remark     해당 설정 값은 모두 base 좌표계 기준 \n
            clink_rpc_robot_force_ctrl_coordinate_set()에서 TCP 좌표계 기준으로 변경 시, TCP기준으로 변경  \n
            사용 가능한 stiffness(x, y, z) 범위: 0<stiffness<10000  \n
            사용 가능한 stiffness(rx, ry, rz) 범위: 0<stiffness<10000  \n
            기본 설정 값 stiffness(x, y, z): 200N/m  \n
            기본 설정 값 stiffness(rx, ry, rz): 100Nm/degree  \n
            Stiffness 값이 커지면, Robot의 유연성은 낮아지나 반응속도는 빨라짐. \n
            Stiffness 값이 작아지면, Robot의 유연성은 높아지나 반응속도가 느려짐.
@return     @ref clink_robot_force_ctrl_stiffness_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_ctrl_stiffness_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t x,
        clink_float_t y,
        clink_float_t z,
        clink_float_t rx,
        clink_float_t ry,
        clink_float_t rz);

/**
@brief      로봇에 Force Control 기능의 LPF cut off frequency를 반환한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] cut_off_freq
            주파수 [Hz]
@return     @ref clink_robot_force_ctrl_low_pass_filter_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_ctrl_low_pass_filter_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t* cut_off_freq);

/**
@brief      로봇에 Force Control 기능의 LPF cut off frequency를 설정한다. \n
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  cuf_off_freq
            cut off 주파수 [Hz], 0 ~ 1000Hz 이내 설정
@return     @ref clink_robot_force_ctrl_low_pass_filter_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_ctrl_low_pass_filter_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t cuf_off_freq);

/**
@brief      로봇에 Force Control 기능의 Sensorless bias 여부를 반환한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_on_off
            설정/해제 여부
@return     @ref clink_robot_force_ctrl_sensorless_bias_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_ctrl_sensorless_bias_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH* p_on_off);

/**
@brief      로봇에 Force Control 기능의 Sensorless bias 여부를 설정한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  on_off
            설정/해제 여부
@return     @ref clink_robot_force_ctrl_sensorless_bias_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_ctrl_sensorless_bias_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH on_off);

/**
@brief      로봇에 Force Control 기능의 각 Pose 별 Force value bias를 받아온다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_x
            x축 설정/해제 여부
@param[out] p_y
            y축 설정/해제 여부
@param[out] p_z
            z축 설정/해제 여부
@param[out] p_rx
            rx축 설정/해제 여부
@param[out] p_ry
            ry축 설정/해제 여부
@param[out] p_rz
            rz축 설정/해제 여부
@return     @ref clink_robot_force_ctrl_force_value_bias_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_ctrl_force_value_bias_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH* p_x,
        CLINK_SWITCH* p_y,
        CLINK_SWITCH* p_z,
        CLINK_SWITCH* p_rx,
        CLINK_SWITCH* p_ry,
        CLINK_SWITCH* p_rz);

/**
@brief      로봇에 Force Control 기능의 각 Pose 별 Force value bias를 설정한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  x
            x축 설정/해제 여부
@param[in]  y
            y축 설정/해제 여부
@param[in]  z
            z축 설정/해제 여부
@param[in]  rx
            rx축 설정/해제 여부
@param[in]  ry
            ry축 설정/해제 여부
@param[in]  rz
            rz축 설정/해제 여부
@return     @ref clink_robot_force_ctrl_force_value_bias_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_ctrl_force_value_bias_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH x,
        CLINK_SWITCH y,
        CLINK_SWITCH z,
        CLINK_SWITCH rx,
        CLINK_SWITCH ry,
        CLINK_SWITCH rz);

/**
@brief      로봇에 Force Control 기능의 actual force 값을 반환한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_x 
            End Effector 의 X 축 방향 Force 값, 단위 : Newton
@param[out] p_y 
            End Effector 의 Y 축 방향 Force 값, 단위 : Newton
@param[out] p_z 
            End Effector 의 Z 축 방향 Force 값, 단위 : Newton
@return     @ref clink_robot_force_ctrl_actual_force_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_ctrl_actual_force_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t* p_x,
        clink_float_t* p_y,
        clink_float_t* p_z);

/**
@brief      로봇에 Force Control 기능의 actual torque 값을 반환한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_x 
            End Effector 의 X 축 방향 Torque 값, 단위 : Newton-meter
@param[out] p_y 
            End Effector 의 Y 축 방향 Torque 값, 단위 : Newton-meter
@param[out] p_z 
            End Effector 의 Z 축 방향 Torque 값, 단위 : Newton-meter
@return     @ref clink_robot_force_ctrl_actual_torq_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_ctrl_actual_torq_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t* p_x,
        clink_float_t* p_y,
        clink_float_t* p_z);

/** @} */


//------------------------------------------------------------------
// 323_ROBOT_FORCE_TORQUE_SENSOR
//------------------------------------------------------------------
/** \addtogroup 323_ROBOT_FORCE_TORQUE_SENSOR 323_ROBOT_FORCE_TORQUE_SENSOR
*  @{
*/

/**
@brief      로봇의 Force Control 기능을 사용하는데 있어 Force Torque Sensor 값 사용 / 미사용 상태를 반환한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_on_off
            CLINK_SWITCH_ON : Force Control에 Force Torque Sensor 값을 사용  \n
            CLINK_SWITCH_OFF : Force Control에 Force Torque Sensor 값을 사용 안 함
@return     @ref clink_robot_force_torq_sensor_switch_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_torq_sensor_switch_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH* p_on_off);

/**
@brief      로봇의 Force Control 기능을 사용하는데 있어 Force Torque Sensor 값 사용 / 미사용 설정
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  on_off
            CLINK_SWITCH_ON : Force Control에 Force Torque Sensor 값을 사용  \n
            CLINK_SWITCH_OFF : Force Control에 Force Torque Sensor 값을 사용 안 함
@return     @ref clink_robot_force_torq_sensor_switch_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_torq_sensor_switch_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH on_off);

/**
@brief      Force Torque Sensor Actual Force 값을 반환한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_x Force Torque Sensor 의 X 축 방향 Force 값, 단위 : Newton
@param[out] p_y Force Torque Sensor 의 Y 축 방향 Force 값, 단위 : Newton
@param[out] p_z Force Torque Sensor 의 Z 축 방향 Force 값, 단위 : Newton
@return     @ref clink_robot_force_torq_sensor_actual_force_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_torq_sensor_actual_force_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t* p_x,
        clink_float_t* p_y,
        clink_float_t* p_z);

/**
@brief      Force Torque Sensor Actual Torque 값을 반환한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_x Force Torque Sensor 의 X 축 방향 Torque 값, 단위 : Newton-meter
@param[out] p_y Force Torque Sensor 의 Y 축 방향 Torque 값, 단위 : Newton-meter
@param[out] p_z Force Torque Sensor 의 Z 축 방향 Torque 값, 단위 : Newton-meter
@return     @ref clink_robot_force_torq_sensor_actual_torq_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_torq_sensor_actual_torq_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t* p_x,
        clink_float_t* p_y,
        clink_float_t* p_z);

/**
@brief      Force Torque Sensor의 Bias 기능을 셋팅한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  on_off
            CLINK_SWITCH_ON : 현재 Load 값을 Bias로 셋팅한다.  \n
            CLINK_SWITCH_OFF : 최근에 셋팅된 Bias 값 사용
@return     @ref clink_robot_force_torq_sensor_bias_switch_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_torq_sensor_bias_switch_set(
        uint32_t cbox_id,
    uint32_t robot_id,
    CLINK_SWITCH on_off);

/**
@brief      Force Torque Sensor의 Bias를 Clear한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  on_off
            CLINK_SWITCH_ON : Bias값을 Clear 한다.  \n
            CLINK_SWITCH_OFF : Bias값을 변경하지 않고 그대로 둔다.
@return     @ref clink_robot_force_torq_sensor_bias_clear_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_torq_sensor_bias_clear(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH on_off);

/**
@brief      Force Torque Sensor Filter 종류를 셋팅한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  filter_value
            CLINK_FORCE_TORQUE_SENSOR_FILTER_NO_FILTER : 필터를 사용하지 않는다.  \n
            CLINK_FORCE_TORQUE_SENSOR_FILTER_1 ~ 8 : Low Pass Filter 사용 \n
            숫자가 높은 필터를 사용할 수록 Cut Off Frequency가 낮아진다.
@return     @ref clink_robot_force_torq_sensor_filter_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_torq_sensor_filter_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_FORCE_TORQUE_SENSOR_FILTER filter_value);

/**
@brief      Force Torque Sensor의 Sample Rate를 설정한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  sample_rate
            CLINK_FORCE_TORQUE_SENSOR_SAMPLE_RATE 참조
@return     @ref clink_robot_force_torq_sensor_sample_rate_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_torq_sensor_sample_rate_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_FORCE_TORQUE_SENSOR_SAMPLE_RATE sample_rate);

/**
@brief      Force Torque Sensor를 Calibration 한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  calib
            CLINK_FORCE_TORQUE_SENSOR_CALIBRATION 참조 \n
            CLINK_FORCE_TORQUE_SENSOR_CALIBRATION_RANGE_0 : (Fxy = 500[N], Fz = 900[N], Txyz = 20[Nm]) \n
            CLINK_FORCE_TORQUE_SENSOR_CALIBRATION_RANGE_1 : (Fxy = 200[N], Fz = 360[N], Txyz = 8[Nm])
@return     @ref clink_robot_force_torq_sensor_calibration_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_force_torq_sensor_calibration_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_FORCE_TORQUE_SENSOR_CALIBRATION calib);

/**
@brief      로봇의 Force Control 기능을 사용하는데 있어 ECAT 이 아닌 외부의 Force Torque Sensor 값 사용 / 미사용 상태를 반환한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_on_off
            CLINK_SWITCH_ON : Force Control에 외부 Force Torque Sensor 값을 사용  \n
            CLINK_SWITCH_OFF : Force Control에 외부 Force Torque Sensor 값을 사용 안 함
@return     @ref clink_robot_ext_force_torq_sensor_switch_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_ext_force_torq_sensor_switch_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH* p_on_off);

/**
@brief      로봇의 Force Control 기능을 사용하는데 있어 ECAT 이 아닌 외부의 Force Torque Sensor 값 사용 / 미사용 설정
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  on_off
            CLINK_SWITCH_ON : Force Control에 외부 Force Torque Sensor 값을 사용  \n
            CLINK_SWITCH_OFF : Force Control에 외부 Force Torque Sensor 값을 사용 안 함
@return     @ref clink_robot_ext_force_torq_sensor_switch_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_ext_force_torq_sensor_switch_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH on_off);

/**
@brief      로봇의 Force Control 기능을 사용하는데 있어 ECAT 이 아닌 외부의 Force Torque Sensor 값을 할당한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  fx
            x 성분 force 값 [N]
@param[in]  fy
            y 성분 force 값 [N]
@param[in]  fz
            z 성분 force 값 [N]
@param[in]  tx
            x 성분 torque 값 [Nm]
@param[in]  ty
            y 성분 torque 값 [Nm]
@param[in]  tz
            z 성분 torque 값 [Nm]
@return     @ref clink_robot_ext_force_torq_sensor_force_torque_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_ext_force_torq_sensor_force_torque_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t fx, 
        clink_float_t fy, 
        clink_float_t fz, 
        clink_float_t tx, 
        clink_float_t ty, 
        clink_float_t tz);

/** @} */


//------------------------------------------------------------------
// 324_ROBOT_CONVEYOR
//------------------------------------------------------------------
/** \addtogroup 324_ROBOT_CONVEYOR 324_ROBOT_CONVEYOR
*  @{
*/

/**
@brief      conveyor 모션에서 처음 가속 구간동안 생기는 거리 오프셋 값을 계산한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  velocity
            conveyor의 속도 (로봇의 속도) [mm/s]
@param[in]  acceleration
            로봇의 가속도 [mm/s^2]
@param[out] p_offset
            계산된 오프셋 값 [mm]
@remark		conveyor 속도는 일정한대 비해, 로봇은 첫 모션에서 가속구간이 있으므로 \n
            해당 구간에서 뒤 떨어지는 거리를 계산하여 미리 반영하기 위함
@return     @ref clink_robot_conveyor_offset_calculate_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_conveyor_offset_calculate(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t velocity,
        clink_float_t acceleration,
        clink_float_t* p_offset);

/**
@brief      conveyor 모션에서 처음 가속 구간동안 생기는 거리 오프셋 값을 계산한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  acceleration
            로봇의 가속도 [mm/s^2]
@param[out] p_offset
            계산된 오프셋 값 [mm]
@remark		clink_robot_conveyor_offset_calculate 과 동일하나, 속도는 엔코더 값을 이용한다. \n
            즉, 호출 당시 conveyor는 동작중이며 그에 따라 엔코데어 값이 들어오고 있어야 한다.
@return     @ref clink_robot_conveyor_offset_calculate_with_encoder_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_conveyor_offset_calculate_with_encoder(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t acceleration,
        clink_float_t* p_offset);

/**
@brief      conveyor 모션에서 이용되는 엔코더 parameter를 셋팅한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  encoder_type
            엔코더 타입
@param[in]  encoder_resolution
            엔코더 분해능
@param[in]  step_size
            스텝 사이즈
@param[in]  channel
            엔코더 사용 채널
@return     @ref clink_robot_conveyor_encoder_info_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_conveyor_encoder_info_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_CONVEYOR_ENCODER_TYPE encoder_type,
        clink_float_t encoder_resolution,
        clink_float_t step_size,
        int32_t channel);

/**
@brief      conveyor 모드에 진입한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@remark		clink_robot_conveyor_mode_end 와 항상 짝을 이뤄 사용한다.
@return     @ref clink_robot_conveyor_mode_start_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_conveyor_mode_start(
        uint32_t cbox_id,
        uint32_t robot_id);

/**
@brief      conveyor 모드를 종료한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@remark		clink_robot_conveyor_mode_start 와 항상 짝을 이뤄 사용한다.
@return     @ref clink_robot_conveyor_mode_end_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_conveyor_mode_end(
        uint32_t cbox_id,
        uint32_t robot_id);

/**
@brief      conveyor 모드 상태를 받아온다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_mode
            ON/OFF 유무 값 반환
@return     @ref clink_robot_conveyor_mode_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_conveyor_mode_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_CONVEYOR_MODE* p_mode);

/**
@brief      conveyor 모드에서 엔코더 사용 여부를 결정한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  on_off
            사용 여부 값
@remark		엔코더를 사용한다는 것은 엔코더 속도 값을 이용하여 로봇이 \n
            그 속도를 추종하도록 한다는 의미이다. (사용자 입력 속도 값 무시)
@return     @ref clink_robot_conveyor_encoder_switch_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_conveyor_encoder_switch_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH on_off);

/**
@brief      conveyor 모드에서 엔코더 사용 상태를 받아온다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_on_off
            ON/OFF 유무 값 반환
@return     @ref clink_robot_conveyor_encoder_switch_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_conveyor_encoder_switch_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH* p_on_off);

/**
@brief      conveyor 하위 모션에서 기준이되는 출발 위치를 지정한다.
            해당 위치 및 방향은 현재 tool 설정이 반영된 TCP 끝 단의 위치이다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  pos_x
            위치 중 X 성분
@param[in]  pos_y
            위치 중 Y 성분
@param[in]  pos_z
            위치 중 Z 성분
@param[in]  ort_x
            방향 중 X 성분
@param[in]  ort_y
            방향 중 Y 성분
@param[in]  ort_z
            방향 중 Z 성분
@remark		conveyor 모션은 컨베이어 방향을 따라 움직이는 기본 모션과 \n
            실 동작을 수행하는 위, 아래 등으로 움직이는 하위모션으로 구성된다.
@return     @ref clink_robot_conveyor_offset_start_pose_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_conveyor_offset_start_pose_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t pos_x,
        clink_float_t pos_y,
        clink_float_t pos_z,
        clink_float_t ort_x,
        clink_float_t ort_y,
        clink_float_t ort_z);

/** @} */


//------------------------------------------------------------------
// 380_ROBOT_DIRECT_TEACHING
//------------------------------------------------------------------
/** \addtogroup 380_ROBOT_DIRECT_TEACHING 380_ROBOT_DIRECT_TEACHING
*  @{
*/

/**
@brief      로봇의 현재 직접교시 상태를 반환한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_on_off
            CLINK_SWITCH_ON : 직접교시 실행 중 \n
            CLINK_SWITCH_OFF : 직접교시 실행하지 않음.
@return     @ref clink_robot_direct_teaching_switch_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_direct_teaching_switch_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH* p_on_off);

/**
@brief      로봇의 직접교시 모드를 설정한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  on_off
            CLINK_SWITCH_ON : 직접교시 실행 \n
            CLINK_SWITCH_OFF : 직접교시 종료 \n
            직접교시 종료 시점에 Actual 과 Command 를 동기화 해야한다.
@return     @ref clink_robot_direct_teaching_switch_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_direct_teaching_switch_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH on_off);

/**
@brief		direct teaching 모드로 강제적으로 전환한다. \n
            payload 검사를 하지 않는다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@remark		Offline Simulator 및 simulation robot 지원불가
@return     @ref clink_robot_direct_teaching_force_to_switch_on_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_direct_teaching_force_to_switch_on(
        uint32_t cbox_id,
        uint32_t robot_id);

/**
@brief		direct teaching 설정 함수 호출 시 payload 감도를 설정한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  sensitivity
            payload 검사 감도 \n
            min: 0(가장 민감), max: 5(가장 둔감), default: 0
@remark		clink_robot_direct_teaching_switch_set(CLINK_SWITCH_ON) 호출 시 실제 로봇에 \n
            걸린 하중과 설정된 payload의 차이가 감도보다 크다면 에러를 반환한다. \n
            해당 에러 반환 시, payload 설정을 재점검한다.
@return     @ref clink_robot_direct_teaching_payload_sensitivity_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_direct_teaching_payload_sensitivity_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        int32_t sensitivity);

/** @} */


//------------------------------------------------------------------
// 381_ROBOT_COLLISION_DETECTION
//------------------------------------------------------------------
/** \addtogroup 381_ROBOT_COLLISION_DETECTION 381_ROBOT_COLLISION_DETECTION
*  @{
*/

/**
@brief      로봇에 설정되어 있는 현재 충돌감지 기능을 상태를 반환한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_on_off
            CLINK_SWITCH_ON : 충돌감지 실행 \n
            CLINK_SWITCH_OFF : 충돌감지 실행하지 않음
@return     @ref clink_robot_collision_detection_switch_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_collision_detection_switch_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH* p_on_off);

/**
@brief      로봇의 충돌감지 기능을 on / off 한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  on_off
            CLINK_SWITCH_ON : 충돌감지 실행 \n
            CLINK_SWITCH_OFF : 충돌감지 실행하지 않음
@remark     Collision Detection 의 비활성화는 협동로봇으로 로봇을 사용하지 않는다는 \n
            의미로서 시스템 담당자는 해당 로봇이 협동로봇의 안전기능인 충돌감지를 \n
            사용하지 않는다는 것을 시각적 도구를 통해 제공해야할 의무가 있음.
@return     @ref clink_robot_collision_detection_switch_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_collision_detection_switch_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH on_off);

/**
@brief      현재 설정되어 있는 충돌감지 인지의 임계치를 반환한다. \n
            (단축 대상)
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  joint_index
            해당하는 Joint 의 Index
@param[out] p_threshold
            임계치 값 (단위 : Newton-meter[Nm])
@return     @ref clink_robot_collision_detection_threshold_joint_torque_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_collision_detection_threshold_joint_torque_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t joint_index,
        clink_float_t* p_threshold);

/**
@brief      충돌감지 인지의 임계치를 설정한다. \n
            (단축 대상)
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  joint_index
            해당하는 Joint 의 Index
@param[in]  threshold
            임계치 값 (단위 : Newton-meter[Nm])
@return     @ref clink_robot_collision_detection_threshold_joint_torque_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_collision_detection_threshold_joint_torque_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t joint_index,
        clink_float_t threshold);

/**
@brief      현재 설정되어 있는 TCP 의 합력 (resultant force) 에 대한 충돌감지 인지의 임계치를 리턴한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_threshold
            임계치 값 (단위 : Newton[N])
@return     @ref clink_robot_collision_detection_threshold_tcp_resultant_force_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_collision_detection_threshold_tcp_resultant_force_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t* p_threshold);

/**
@brief      TCP 의 합력 (resultant force) 에 대한 충돌감지 인지의 임계치를 설정한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  threshold
            임계치 값 (단위 : Newton[N])
@return     @ref clink_robot_collision_detection_threshold_tcp_resultant_force_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_collision_detection_threshold_tcp_resultant_force_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t threshold);

/**
@brief      현재 설정되어 있는 TCP에 대한 충돌감지 인지의 임계치를 리턴한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_threshold_fx
            x방향 TCP force 임계치 값 (단위 : Newton[N])
@param[out] p_threshold_fy
            y방향 TCP force 임계치 값 (단위 : Newton[N])
@param[out] p_threshold_fz
            z방향 TCP force 임계치 값 (단위 : Newton[N])
@param[out] p_threshold_rx
            x축 TCP torque 임계치 값 (단위 : Newton-Meter[Nm])
@param[out] p_threshold_ry
            y축 TCP torque 임계치 값 (단위 : Newton-Meter[Nm]))
@param[out] p_threshold_rz
            z축 TCP torque 임계치 값 (단위 : Newton-Meter[Nm]))
@return     @ref clink_robot_collision_detection_threshold_tcp_force_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_collision_detection_threshold_tcp_force_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t* p_threshold_fx,
        clink_float_t* p_threshold_fy,
        clink_float_t* p_threshold_fz,
        clink_float_t* p_threshold_rx,
        clink_float_t* p_threshold_ry,
        clink_float_t* p_threshold_rz);

/**
@brief      TCP에 대한 충돌감지 인지의 임계치를 설정한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  threshold_fx
            x방향 TCP force 임계치 값 (단위 : Newton[N])
@param[in]  threshold_fy
            y방향 TCP force 임계치 값 (단위 : Newton[N])
@param[in]  threshold_fz
            z방향 TCP force 임계치 값 (단위 : Newton[N])
@param[in]  threshold_rx
            x축 TCP torque 임계치 값 (단위 : Newton-Meter[Nm])
@param[in]  threshold_ry
            y축 TCP torque 임계치 값 (단위 : Newton-Meter[Nm]))
@param[in]  threshold_rz
            z축 TCP torque 임계치 값 (단위 : Newton-Meter[Nm]))
@return     @ref clink_robot_collision_detection_threshold_tcp_force_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_collision_detection_threshold_tcp_force_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t threshold_fx,
        clink_float_t threshold_fy,
        clink_float_t threshold_fz,
        clink_float_t threshold_rx,
        clink_float_t threshold_ry,
        clink_float_t threshold_rz);

/**
@brief      로봇에 설정되어 있는 현재 충돌감지 완화 기능을 상태를 반환한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_on_off
            CLINK_SWITCH_ON : 충돌감지 완화 기능 실행 \n
            CLINK_SWITCH_OFF : 충돌감지 완화 기능 실행하지 않음
@return     @ref clink_robot_collision_mitigation_switch_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_collision_mitigation_switch_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH* p_on_off);

/**
@brief      로봇의 충돌감지 완화 기능을 on / off 한다.
@param[in]  robot_id 
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  on_off
            CLINK_SWITCH_ON : 충돌감지 완화 기능 실행 \n
            CLINK_SWITCH_OFF : 충돌감지 완화 기능 실행하지 않음
@return     @ref clink_robot_collision_mitigation_switch_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_collision_mitigation_switch_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH on_off);

/**
@brief      로봇이 충돌 상태일 때, 충돌 상태를 초기화한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@remark     한 번 충돌 발생 후, 해당 API 호출 전까지는 다시 충돌이 발생하지 않는다.
@return     @ref clink_robot_collision_detection_clear_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_collision_detection_clear(
        uint32_t cbox_id,
        uint32_t robot_id);

/**
@brief      직접 교시 감도를 받아온다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  joint_idx
            joint의 index
@param[out] p_feed_rate
            직접 교시 감도 [0 ~ 1] 사이 값
@return     @ref clink_robot_dynamics_friction_feedrate_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_dynamics_friction_feedrate_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t joint_idx,
        clink_float_t* p_feed_rate);

/**
@brief      직접 교시 감도를 조절한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  joint_idx
            joint의 index
@param[in]  feed_rate
            직접 교시 감도 [0 ~ 1] 사이 값
@return     @ref clink_robot_dynamics_friction_feedrate_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_dynamics_friction_feedrate_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t joint_idx,
        clink_float_t feed_rate);

/**
@brief      모든 축의 friction modeling을 시작한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@return     @ref clink_robot_dynamics_calibration_start_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_dynamics_calibration_start(
        uint32_t cbox_id,
        uint32_t robot_id);

/**
@brief      한 축만 friction modeling을 시작한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  joint_idx
            joint의 index
@return     @ref clink_robot_joint_dynamics_calibration_start_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_joint_dynamics_calibration_start(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t joint_idx);

/**
@brief      모든 축의 friction modeling값을 로봇 내부에 저장한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@return     @ref clink_robot_dynamics_calibration_save_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_dynamics_calibration_save(
        uint32_t cbox_id,
        uint32_t robot_id);

/**
@brief      한 축만 friction modeling값을 로봇 내부에 저장한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  joint_idx
            joint의 index
@return     @ref clink_robot_joint_dynamics_calibration_save_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_joint_dynamics_calibration_save(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t joint_idx);

/**
@brief      dynamics calibration 전 warm up 동작을 수행하는 기능 시작
            clink_rpc_robot_dynamics_dry_run_end() 호출 시 까지 반복해서 동작
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  speed_factor
            motion을 수행할 속도 비율
@param[in]  sleep_time_millisec
            하나의 motion cycle 이 지난 후 sleep time
@return     @ref clink_robot_dynamics_dry_run_start_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_dynamics_dry_run_start(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t speed_factor,
        uint32_t sleep_time_millisec);

/**
@brief      dynamics calibration 전 warm up 동작을 수행하는 기능 종료
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@return     @ref clink_robot_dynamics_dry_run_end_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_dynamics_dry_run_end(
        uint32_t cbox_id,
        uint32_t robot_id);

/**
@brief      dynamics calibration 전 warm up 동작을 수행하는 기능에서
            해당 기능이 수행중인지 받아온다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] on_off
            dry run 상태
@return     @ref clink_robot_dynamics_dry_run_state_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_dynamics_dry_run_state_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH* on_off);

/** @} */


//------------------------------------------------------------------
// 382_ROBOT_UTIL
//------------------------------------------------------------------
/** \addtogroup 382_ROBOT_UTIL 382_ROBOT_UTIL
*  @{
*/
/**
@brief      주어진 Joint Angle 을 로봇의 Pose (Position + Orientation) 으로 변환한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  type
            출력되는 pos, ort의 위치가 flange인지 tcp 인지 정의
@param[in]  p_joint_angles
            조인트 각도 배열
@param[out] p_pos
            위치 X,Y,Z (크기 3이상 배열)
@param[out] p_ort
            방향 X,Y,Z (크기 3이상 배열)
@return     @ref clink_robot_util_joint_angle_to_pose_convert_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_util_joint_angle_to_pose_convert(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_POSE_TYPE type,
        const clink_float_t* p_joint_angles,
        clink_float_t* p_pos,
        clink_float_t* p_ort);

/**
@brief      주어진 Pose (Position + Orientation) 에 해당 하는 각도를 리턴한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  type
            출력되는 pos, ort의 위치가 flange인지 tcp 인지 정의
@param[in]  p_ref_angles
            Inverse Kinematics 에 사용할 reference angle.
@param[in]  p_pos
            위치 X,Y,Z (크기 3이상 배열)
@param[in]  p_ort
            방향 X,Y,Z (크기 3이상 배열)
@param[out] p_joint_angles
            조인트 각도 배열
@return     @ref clink_robot_util_pose_to_joint_angle_convert_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_util_pose_to_joint_angle_convert(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_POSE_TYPE type,
        const clink_float_t* p_ref_angles,
        const clink_float_t* p_pos,
        const clink_float_t* p_ort,
        clink_float_t* p_joint_angles);

/**
@brief      주어진 Pose (Position + Orientation) 에 해당 하는 각도를 리턴한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  type
            출력되는 pos, ort의 위치가 flange인지 tcp 인지 정의
@param[in]  p_ref_angles
            Inverse Kinematics 에 사용할 reference angle.
@param[in]  p_pos
            위치 X,Y,Z (크기 3이상 배열)
@param[in]  p_ort
            방향 X,Y,Z (크기 3이상 배열)
@param[in]  iter
            반복 계산 횟수
@param[out] p_joint_angles
            조인트 각도 배열
@return     @ref clink_robot_util_pose_to_joint_angle_convert_iterative_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_util_pose_to_joint_angle_convert_iterative(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_POSE_TYPE type,
        const clink_float_t* p_ref_angles,
        const clink_float_t* p_pos,
        const clink_float_t* p_ort,
        uint32_t iter,
        clink_float_t* p_joint_angles);

/**
@brief      주어진 Pose (Position + Orientation) 에 해당 하는 각도를 리턴한다. \n
            Inverse Kinematics 에서 가능한 8개의 모든 해를 반환한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  p_pos
            위치 X,Y,Z (크기 3이상 배열)
@param[in]  p_ort
            방향 X,Y,Z (크기 3이상 배열)
@param[out] p_joint_angles
            로봇의 각 조인트 angle 반환(단위: Degree) \n
            8개의 해가 차례대로 저장된다. \n
            즉, 배열의 크기는 조인트 개수 X 8 만큼 할당되어야 한다. \n
            해가 존재하지 않을 경우 해당 배열 구간의 값은 invalid 하다.
@param[out] p_validity
            8개의 해 중, 존재하는 경우 1, 존재하지 않는 경우 0을 반환한다. \n
            해당 배열의 크기는 8만큼 할당되어야 한다.
@return     @ref clink_robot_util_pose_to_joint_all_possible_angle_convert_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_util_pose_to_joint_all_possible_angle_convert(
        uint32_t cbox_id,
        uint32_t robot_id,
        const clink_float_t* p_pos,
        const clink_float_t* p_ort,
        clink_float_t* p_joint_angles,
        char_t* p_validity);

/**
@brief      주어진 Pose (Position + Orientation) 에 해당 하는 각도를 리턴한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  posture
            CLINK_ROBOT_POSTURE 참조
@param[in]  p_pos
            위치 X,Y,Z (크기 3이상 배열)
@param[in]  p_ort
            방향 X,Y,Z (크기 3이상 배열)
@param[out] p_joint_angles
            posture 에 해당하는 로봇의 각 조인트 angle 반환(단위: Degree)
@param[out] p_validity
            존재하는 경우 1, 존재하지 않는 경우 0을 반환한다.
@return     @ref clink_robot_util_pose_to_posture_joint_angle_convert_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_util_pose_to_posture_joint_angle_convert(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_ROBOT_POSTURE posture,
        const clink_float_t* p_pos,
        const clink_float_t* p_ort,
        clink_float_t* p_joint_angles,
        char_t* p_validity);

/**
@brief      시작 각도와 끝 각도를 받아서 로봇 끝 단의 position 궤적 정보를 얻는다.
            구해진 position 정보는 num_of_poins 가 2 이상일 경우 시작 각도와 끝 각도를 포함한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  num_of_joints
            로봇의 joint 갯수
@param[in]  start_angles
            시작 각도, num_of_joints 갯수 만큼의 배열 [degree]
@param[in]  end_angles
            끝 각도, num_of_joints 갯수 만큼의 배열 [degree]
@param[in]  num_of_points
            궤적을 이루는 point의 갯수 설정
@param[in]  pos_type
            계산되는 position 값이 flange 끝 단인지 tcp 끝 단인지 결정
@param[out] p_x
            계산된 좌표의 x 성분 배열, num_of_points 만큼의 크기 할당필요 [mm]
@param[out] p_y
            계산된 좌표의 y 성분 배열, num_of_points 만큼의 크기 할당필요 [mm]
@param[out] p_z
            계산된 좌표의 z 성분 배열, num_of_points 만큼의 크기 할당필요 [mm]
@return     @ref clink_robot_util_position_trajectory_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_util_position_trajectory_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t num_of_joints,
        const clink_float_t* start_angles,
        const clink_float_t* end_angles,
        uint32_t num_of_points,
        CLINK_POSE_TYPE pos_type,
        clink_float_t* p_x,
        clink_float_t* p_y,
        clink_float_t* p_z);

/**
@brief      reference 좌표계와 그 좌표계에 대항하는 좌표들을 입력받아 \n
            base 좌표계에서의 좌표를 구한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  ref_pos_x
            reference 좌표계에서 X 성분 [m]
@param[in]  ref_pos_y
            reference 좌표계에서 Y 성분 [m]
@param[in]  ref_pos_z
            reference 좌표계에서 Z 성분 [m]
@param[in]  ref_ort_x
            reference 좌표계에서 RX 성분 [degree]
@param[in]  ref_ort_y
            reference 좌표계에서 RY 성분 [degree]
@param[in]  ref_ort_z
            reference 좌표계에서 RZ 성분 [degree]
@param[in]  p_input_file_name
            reference 좌표계 기준 좌표 리스트가 저장된 파일 \n
            x, y, z, rx, ry, rz 순으로 나열되어 있다.
@param[out] p_output_file_name
            계산된 base 좌표계 기준 좌표 리스트. \n
            x, y, z, rx, ry, rz 순으로 나열되어 있다.
@return     @ref clink_robot_util_common_to_base_coordinate_convert_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_util_common_to_base_coordinate_convert(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t ref_pos_x,
        clink_float_t ref_pos_y,
        clink_float_t ref_pos_z,
        clink_float_t ref_ort_x,
        clink_float_t ref_ort_y,
        clink_float_t ref_ort_z,
        const char_t* p_input_file_name,
        char_t* p_output_file_name);

/** @} */


//------------------------------------------------------------------
// 400_MOTION_COMMAND
//------------------------------------------------------------------
/** \addtogroup 400_MOTION_COMMAND 400_MOTION_COMMAND
*  @{
*/
/**
@brief      모션커맨드에 할당된 메모리를 해제한다.
@param[in]  motion_command_id
            clink_rpc_motion_command_robot_tcp_welding_create(), \n
            clink_rpc_motion_command_robot_joint_create(), \n
            clink_rpc_motion_command_robot_tcp_create() 함수를 통해 \n
            반환받은 인스턴스 ID
@return     @ref clink_motion_command_destroy_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_destroy(
        uint32_t cbox_id,
        uint32_t motion_command_id);

/**
@brief      clink_motion_command_robot_tcp_welding_create(), \n
            clink_rpc_motion_command_robot_joint_create(), \n
            clink_rpc_motion_command_robot_tcp_create() 함수를 통해 \n
            생성된 모든 motion command를 삭제한다.
@return     @ref clink_motion_command_destroy_all_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_destroy_all(
        uint32_t cbox_id);

/**
@brief      모션커맨드의 복사본 (deepcopy) 를 반환한다.
@param[in]  src_motion_command_id
            clink_rpc_motion_command_robot_tcp_welding_create(), \n
            clink_rpc_motion_command_robot_joint_create(), \n
            clink_rpc_motion_command_robot_tcp_create() 함수를 통해 \n
            반환받은 인스턴스 ID
@param[out] p_dest_motion_command_id
            모션커맨드의 복사본 (deepcopy)
@return     @ref clink_motion_command_deep_copy_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_deep_copy(
        uint32_t cbox_id,
        uint32_t src_motion_command_id,
        uint32_t* p_dest_motion_command_id);

/**
@brief      속도 profiler 의 종류를 반환한다.
@param[in]  motion_command_id
            clink_rpc_motion_command_robot_tcp_welding_create(), \n
            clink_rpc_motion_command_robot_joint_create(), \n
            clink_rpc_motion_command_robot_tcp_create() 함수를 통해 \n
            반환받은 인스턴스 ID
@param[out] p_profile_name
            속도 profile 종류 이름
@return     @ref clink_motion_command_velocity_profiler_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_velocity_profiler_get(
        uint32_t cbox_id,
        uint32_t motion_command_id,
        CLINK_VELOCITY_PROFILE* p_profile_name);

/**
@brief      속도 profiler 의 종류를 설정한다.
@param[in]  motion_command_id
            clink_rpc_motion_command_robot_tcp_welding_create(), \n
            clink_rpc_motion_command_robot_joint_create(), \n
            clink_rpc_motion_command_robot_tcp_create() 함수를 통해 \n
            반환받은 인스턴스 ID
@param[in]  profiler_name
            CLINK_VELOCITY_PROFILE_ASYMMETRIC_S_CURVE : 비대칭형 S Curve 프로파일 \n
            CLINK_VELOCITY_PROFILE_TRAPEZODAL : 대칭형 사다리꼴 프로파일
@remark     <STRONG>현재, CLINK_VELOCITY_PROFILE_ASYMMETRIC_S_CURVE 만을 지원 </STRONG>
@return     @ref clink_motion_command_velocity_profiler_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_motion_command_velocity_profiler_set(
        uint32_t cbox_id,
        uint32_t motion_command_id, 
        CLINK_VELOCITY_PROFILE profiler_name);

/**
@brief      모션 커맨드의 최대 가속도를 반환한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 또는, \n
            clink_rpc_motion_command_robot_joint_create() 또는, \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환받은 값
@param[out] p_acc_max
            모션 커맨드의 최대 가속도, 이하 종류에 따른 단위 :  \n
            Motor       : count / second^2 \n
            Robot Joint : degree / second^2 \n
            Robot TCP   : millimeter / second^2
@return     @ref clink_motion_command_acc_max_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_motion_command_acc_max_get(
        uint32_t cbox_id,
        uint32_t motion_command_id,
        clink_float_t* p_acc_max);

/**
@brief      모션 커맨드의 최대 가속도를 설정한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 또는, \n
            clink_rpc_motion_command_robot_joint_create() 또는, \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환받은 값
@param[in]  acc_max
            모션 커맨드의 최대 가속도, 이하 종류에 따른 단위 :  \n
            Motor       : count / second^2 \n
            Robot Joint : degree / second^2 \n
            Robot TCP   : millimeter / second^2
@return     @ref clink_motion_command_acc_max_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_motion_command_acc_max_set(
        uint32_t cbox_id,
        uint32_t motion_command_id, 
        clink_float_t acc_max);

/**
@brief      모션 커맨드의 최대 감속도를 반환한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 또는, \n
            clink_rpc_motion_command_robot_joint_create() 또는, \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환받은 값
@param[out] p_dec_max
            모션 커맨드의 최대 감가속도, 이하 종류에 따른 단위 :  \n
            Motor       : count / second ^ 2 \n
            Robot Joint : degree / second ^ 2 \n
            Robot TCP   : millimeter / second ^ 2
@return     @ref clink_motion_command_dec_max_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_motion_command_dec_max_get(
        uint32_t cbox_id,
        uint32_t motion_command_id,
        clink_float_t* p_dec_max);

/**
@brief      모션 커맨드의 최대 감속도를 설정한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 또는, \n
            clink_rpc_motion_command_robot_joint_create() 또는, \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환받은 값
@param[in]  dec_max
            모션 커맨드의 최대 감속도, 이하 종류에 따른 단위 :  \n
            Motor       : count / second^2 \n
            Robot Joint : degree / second^2 \n
            Robot TCP   : millimeter / second^2
@return     @ref clink_motion_command_dec_max_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_motion_command_dec_max_set(
        uint32_t cbox_id,
        uint32_t motion_command_id, 
        clink_float_t dec_max);

/**
@brief      모션 커맨드의 최대 속도를 반환한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 또는, \n
            clink_rpc_motion_command_robot_joint_create() 또는, \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환받은 값
@param[out] p_speed_max
            모션 커맨드의 최대 속도, 이하 종류에 따른 단위 :  \n
            Motor       : count / second \n
            Robot Joint : degree / second \n
            Robot TCP   : millimeter / second 
@return     @ref clink_motion_command_speed_max_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_motion_command_speed_max_get(
        uint32_t cbox_id,
        uint32_t motion_command_id,
        clink_float_t* p_speed_max);

/**
@brief      모션 커맨드의 최대 속도를 설정한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 또는, \n
            clink_rpc_motion_command_robot_joint_create() 또는, \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환받은 값
@param[in]  speed_max
            모션 커맨드의 최대 속도, 이하 종류에 따른 단위 :  \n
            Motor       : count / second \n
            Robot Joint : degree / second \n
            Robot TCP   : millimeter / second
@return     @ref clink_motion_command_speed_max_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_motion_command_speed_max_set(
        uint32_t cbox_id,
        uint32_t motion_command_id,
        clink_float_t speed_max);

/**
@brief      모션 커맨드의 JERK 값을 반환한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 또는, \n
            clink_rpc_motion_command_robot_joint_create() 또는, \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환받은 값
@param[out] p_jerk_percentage
            모션 커맨드의 JERK 값 \n
            최소 : 0.01, 최대 : 1.0
@return     @ref clink_motion_command_jerk_percentage_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_jerk_percentage_get(
        uint32_t cbox_id,
        uint32_t motion_command_id,
        clink_float_t* p_jerk_percentage);

/**
@brief      모션 커맨드의 JERK 값
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 또는, \n
            clink_rpc_motion_command_robot_joint_create() 또는, \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환받은 값
@param[in]  jerk_percentage
            모션 커맨드의 JERK 값 \n
            최소 : 0.01, 최대 : 1.0
@remark     CLINK_VELOCITY_PROFILE_ASYMMETRIC_S_CURVE \n
            을 사용할때만이 유효하다.
@return     @ref clink_motion_command_jerk_percentage_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_motion_command_jerk_percentage_set(
        uint32_t cbox_id,
        uint32_t motion_command_id, 
        clink_float_t jerk_percentage);

/**
@brief      설정된 로봇의 Blending Motion Mode를 반환한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_joint_create() 혹은 \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환받은 값
@param[out] p_blending_mode
            현재 설정되어 있는 Blending Motion Mode \n
            CLINK_SWITCH_ON : Blending Motion Mode가 On \n
            CLINK_SWITCH_OFF : Blending Motion Mode가 Off
@return     @ref clink_motion_command_blending_motion_mode_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_blending_motion_mode_get(
        uint32_t cbox_id,
        uint32_t motion_command_id,
        CLINK_SWITCH* p_blending_mode);

/**
@brief      로봇의 Blending Motion Mode를 설정한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_joint_create() 혹은 \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환받은 값
@param[in]  blending_mode
            설정하는 Blending Motion Mode \n
            CLINK_SWITCH_ON : Blending Motion Mode를 On \n
            CLINK_SWITCH_OFF : Blending Motion Mode를 Off
@return     @ref clink_motion_command_blending_motion_mode_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_blending_motion_mode_set(
        uint32_t cbox_id,
        uint32_t motion_command_id,
        CLINK_SWITCH blending_mode);

/**
@brief      Blending motion시 시작조건으로서 설정된 바로 이전에 insert 된 모션의 남은 반지름 길이를 반환한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_joint_create() 혹은 \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환받은 값
@param[out] p_radius_for_blending
            모션의 남은 반지름 길이 (단위 : mm)
@return     @ref clink_motion_command_radius_for_blending_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_radius_for_blending_get(
        uint32_t cbox_id,
        uint32_t motion_command_id,
        clink_float_t* p_radius_for_blending);

/**
@brief      Blending motion시 시작조건으로서 바로 이전에 insert 된 모션의 남은 반지름 길이를 설정한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_joint_create() 혹은 \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환받은 값
@param[in]	radius_for_blending
            모션의 남은 반지름 길이 (단위 : mm)
@return     @ref clink_motion_command_radius_for_blending_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_radius_for_blending_set(
        uint32_t cbox_id,
        uint32_t motion_command_id,
        clink_float_t radius_for_blending);

/**
@brief      blending motion의 반지름 시작 조건 기능 ON/OFF 여부를 받아온다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_joint_create() 혹은 \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환받은 값
@param[out]	p_on_off
            blending radius 자동판단 기능 ON/OFF \n

@return     @ref clink_motion_command_blending_auto_start_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_blending_auto_start_get(
        uint32_t cbox_id,
        uint32_t motion_command_id,
        CLINK_SWITCH* p_on_off);

/**
@brief      blending motion의 반지름 시작 조건을 감속구간으로 자동 설정한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_joint_create() 혹은 \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환받은 값
@param[in]	on_off
            blending radius 자동판단 기능 ON/OFF
@return     @ref clink_motion_command_blending_auto_start_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_blending_auto_start_set(
        uint32_t cbox_id,
        uint32_t motion_command_id,
        CLINK_SWITCH on_off);

/**
@brief      blending motion의 stop mode 상태를 받아온다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_joint_create() 혹은 \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환받은 값
@param[out]	p_on_off
            stop 기능 ON/OFF
@return     @ref clink_motion_command_blending_stop_mode_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_blending_stop_mode_get(
        uint32_t cbox_id,
        uint32_t motion_command_id,
        CLINK_SWITCH* p_on_off);

/**
@brief      blending motion이 시작할 때 앞서 진행중인 모션을 stop 시킨다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_joint_create() 혹은 \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환받은 값
@param[in]	on_off
            stop 기능 ON/OFF
@return     @ref clink_motion_command_blending_stop_mode_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_blending_stop_mode_set(
        uint32_t cbox_id,
        uint32_t motion_command_id,
        CLINK_SWITCH on_off);

/**
@brief      삽입된 모션에 대해 시작 조건을 설정한다. \n
            시작 조건이 만족되면 모션이 실제로 실행된다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_joint_create() 혹은 \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환받은 값
@param[in]	io_type
            digital IO type
@param[in]	channel
            IO index
@param[in]	comparison_value
            비교 값(일치 시, 조건 만족)
@return     @ref clink_motion_command_start_condition_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_start_condition_set(
        uint32_t cbox_id,
        uint32_t motion_command_id,
        CLINK_IO_TYPE io_type,
        uint32_t channel,
        int32_t comparison_value);

/**
@brief      모션 커맨드에 설정된 이름을 받아온다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_joint_create() 혹은 \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환받은 값
@param[out] p_motion_name
            모션 커맨드 이름
@return     @ref clink_motion_command_name_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_name_get(
        uint32_t cbox_id,
        uint32_t motion_command_id,
        char_t* p_motion_name);

/**
@brief      모션 커맨드의 이름을 설정한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_joint_create() 혹은 \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환받은 값
@param[in]  p_motion_name
            모션의 이름
@return     @ref clink_motion_command_name_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_name_set(
        uint32_t cbox_id,
        uint32_t motion_command_id,
        const char_t* p_motion_name);

/**
@brief      모션 커맨드를 내부 Queue 에 추가한다.
@param[in]  motion_command_id
            clink_rpc_motion_command_robot_tcp_welding_create(), \n
            clink_rpc_motion_command_robot_joint_create(), \n
            clink_rpc_motion_command_robot_tcp_create() 함수를 통해 \n
            반환받은 인스턴스 ID
@remark     clink_motion_command_queue_flush() 가 호출되기 전까지 \n
            clink_rpc_motion_command_queue_add() 를 통해 추가된 \n
            모션 커맨드는 실행되지 않는다.
@return     @ref clink_motion_command_queue_add_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_motion_command_queue_add(
        uint32_t cbox_id,
        uint32_t motion_command_id);

/**
@brief      clink_motion_command_queue_add() 를 통해 추가된 모션 커맨드를 \n
            내부적으로 모두 삭제한다.
@remark     queue 에서 모션이 삭제되는 것이 해당 모션 instance 의 메모리가 \n
            해제되는 것은 아니다. 메모리 해제는 clink_motion_command_destroy() \n
            를 통해 실행해야 한다.
@return     @ref clink_motion_command_queue_clear_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_motion_command_queue_clear(
        uint32_t cbox_id);

/**
@brief      clink_motion_command_queue_add() 를 통해 추가된 \n
            모션 커맨드를 일괄 다운로드하여 실행한다.
@remark     clink_motion_command_queue_flush() 가 호출되기 전까지 \n
            clink_rpc_motion_command_queue_add() 를 통해 추가된 \n
            모션 커맨드는 실행되지 않는다.
@return     @ref clink_motion_command_queue_flush_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_motion_command_queue_flush(
        uint32_t cbox_id);

/**
@brief      모션 커맨드의 실행 시간을 리턴한다. (동작 X)
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_joint_create() 혹은 \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환받은 값
@param[out] p_time
            예상되는 실행 시간[second]
@return     @ref clink_motion_command_running_time_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_running_time_get(
        uint32_t cbox_id,
        uint32_t motion_command_id,
        clink_float_t* p_time);

/**
@brief      모션 커맨드가 실행 가능한지 (실행 중 에러는 없는지) 를 리턴한다. (동작X)
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_joint_create() 혹은 \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환받은 값
@return     @ref clink_motion_command_is_runnable_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_is_runnable(
        uint32_t cbox_id,
        uint32_t motion_command_id);

/**
@brief      모션 커맨드를 실행하며 특정 IO 출력의 실행이 필요한 경우 사용한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_joint_create() 혹은 \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환받은 값
@param[in]  cmd_type
            CLINK_IO_CONTROL_CONDITION_TIME : 모션 실행 시간이 threshold 값에 도달하는 순간 실행한다. (단위 : second) \n
            CLINK_IO_CONTROL_CONDITION_PERCENTAGE : 모션의 진행 비율이 threshold 값에 도달하는 순간 실행한다. (단위 : %)
@param[in]	threshold 
            cmd_type 에 맞추어 임계치를 설정한다. \n
            단위 : % \n
            입력 범위 : 0 ~ 1.0 \n
@param[in]	io_type 
            아래의 출력 IO 는 모두 사용 가능하다. 단, HW 적으로 해당 IO 포트가 존재하는지 \n
            HW 사양의 확인이 필요하다. \n
            CLINK_IO_TYPE_CBOX_DIGITAL_OUTPUT \n
            CLINK_IO_TYPE_CBOX_ANALOG_OUTPUT \n
            CLINK_IO_TYPE_CBOX_SAFETY_MODULE_DIGITAL_OUTPUT \n
            CLINK_IO_TYPE_CBOX_SAFETY_MODULE_ANALOG_OUTPUT \n
            CLINK_IO_TYPE_TOOL_DIGITAL_OUTPUT \n
            CLINK_IO_TYPE_TOOL_ANALOG_OUTPUT
@param[in]	io_channel
            출력할 io의 채널을 설정
@param[in]	io_val
            출력할 값을 설정한다. analog 의 경우 해당 값을 그대로 출력한다. \n
            digital 의 경우, 0.0 을 입력하면 LOW (0) 을 그 외의 경우에는 모두 HIGH (1) 을 출력한다.
@return     @ref clink_motion_command_io_command_type_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_io_command_type_set(
        uint32_t cbox_id,
        uint32_t motion_command_id,
        CLINK_IO_CONTROL_CONDITION cmd_type,
        clink_float_t threshold,
        CLINK_IO_TYPE io_type,
        uint32_t io_channel,
        clink_float_t io_val);

/** @} */


//------------------------------------------------------------------
// 401_MOTION_COMMAND_ROBOT
//------------------------------------------------------------------
/** \addtogroup 401_MOTION_COMMAND_ROBOT 401_MOTION_COMMAND_ROBOT
*  @{
*/

/**
@brief      설정된 로봇의 ID 를 반환한다. 
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_joint_create() 혹은 \n 
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환받은 값
@param[out] p_robot_id
            clink_rpc_motion_command_robot_robot_id_set() 를 \n
            통해 설정한 로봇 ID        
@return     @ref clink_motion_command_robot_robot_id_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_motion_command_robot_robot_id_get(
        uint32_t cbox_id,
        uint32_t motion_command_id, 
        uint32_t* p_robot_id);

/**
@brief      모션 커맨드를 실행할 로봇을 설정한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_joint_create() 혹은 \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환받은 값
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@remark     clink_motion_command_robot_tcp_welding_create() 를 통해 생성한 \n
            모터를 위한 모션 커맨드는 사용할 수 없다.
@return     @ref clink_motion_command_robot_robot_id_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_robot_id_set(
        uint32_t cbox_id,
        uint32_t motion_command_id,
        uint32_t robot_id);

/**
@brief      zigzag weaving motion 각 섹션의 변화에 따라 이벤트 발생 여부 상태를 받아온다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_on_off
            발생 여부 설정 값
@return     @ref clink_motion_command_robot_weaving_event_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_weaving_event_get(
        uint32_t cbox_id,
	    uint32_t robot_id,
	    CLINK_SWITCH* p_on_off);

/**
@brief      zigzag weaving motion 각 섹션의 변화에 따라 이벤트 발생 여부를 설정한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  on_off
            발생 여부 설정 값
@return     @ref clink_motion_command_robot_weaving_event_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_motion_command_robot_weaving_event_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH on_off);

/** @} */


//------------------------------------------------------------------
// 402_MOTION_COMMAND_ROBOT_JOINT
//------------------------------------------------------------------
/** \addtogroup 402_MOTION_COMMAND_ROBOT_JOINT 402_MOTION_COMMAND_ROBOT_JOINT 
*  @{
*/

/**
@brief      로봇 Joint 를 위한 모션 커맨드를 생성한다.
@param[out] p_command_id
            로봇 Joint 를 위한 모션 커맨드 인스턴스의 ID
@return     @ref clink_motion_command_robot_joint_create_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_joint_create(
        uint32_t cbox_id,
        uint32_t* p_command_id);

/**
@brief      모션 커맨드의 시작 각도를 반환한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_joint_create() 를 통해 반환 받은 값
@param[in]  joint_index
            해당 하는 Joint 의 index \n
            6축 로봇의 경우, 첫 번째 축은 0 \n
            마지막 축은 5 
@param[out] p_angle_start
            시작 각도 (단위 : degree)
@return     @ref clink_motion_command_robot_joint_angle_start_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_joint_angle_start_get(
        uint32_t cbox_id,
            uint32_t motion_command_id, 
            uint32_t joint_index,
            clink_float_t* p_angle_start);

/**
@brief      모션 커맨드의 시작 각도를 설정한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_joint_create() 를 통해 반환 받은 값
@param[in]  joint_index
            해당 하는 Joint 의 index \n
            6축 로봇의 경우, 첫 번째 축은 0 \n
            마지막 축은 5 
@param[in]  angle_start
            시작 각도 (단위 : degree)
@return     @ref clink_motion_command_robot_joint_angle_start_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_motion_command_robot_joint_angle_start_set(
        uint32_t cbox_id,
        uint32_t motion_command_id, 
        uint32_t joint_index, 
        clink_float_t angle_start);

/**
@brief      모션 커맨드의 종료 각도를 반환한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_joint_create() 를 통해 반환 받은 값
@param[in]  joint_index
            해당 하는 Joint 의 index \n
            6축 로봇의 경우, 첫 번째 축은 0 \n
            마지막 축은 5 
@param[out] p_angle_end
            종료 각도 (단위 : degree)
@return     @ref clink_motion_command_robot_joint_angle_end_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_motion_command_robot_joint_angle_end_get(
        uint32_t cbox_id,
        uint32_t motion_command_id, 
        uint32_t joint_index,
        clink_float_t* p_angle_end);

/**
@brief      모션 커맨드의 종료 각도를 설정한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_joint_create() 를 통해 반환 받은 값
@param[in]  joint_index
            해당 하는 Joint 의 index \n
            6축 로봇의 경우, 첫 번째 축은 0 \n
            마지막 축은 5 
@param[in]  angle_end
            종료 각도 (단위 : degree)
@return     @ref clink_motion_command_robot_joint_angle_end_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_motion_command_robot_joint_angle_end_set(
        uint32_t cbox_id,
        uint32_t motion_command_id, 
        uint32_t joint_index, 
        clink_float_t angle_end);

/** @} */


//------------------------------------------------------------------
// 403_MOTION_COMMAND_ROBOT_TCP
//------------------------------------------------------------------
/** \addtogroup 403_MOTION_COMMAND_ROBOT_TCP 403_MOTION_COMMAND_ROBOT_TCP
*  @{
*/

/**
@brief      로봇 TCP 를 위한 모션 커맨드를 생성한다.
@param[out] p_command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스의 ID
@return     @ref clink_motion_command_robot_tcp_create_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_create(
        uint32_t cbox_id,
        uint32_t* p_command_id);

/**
@brief      모션 커맨드의 시작 속도를 반환한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환 받은 값
@param[out] p_speed_start
            시작 속도 (단위 : millimeter / second)
@return     @ref clink_motion_command_robot_tcp_speed_start_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_speed_start_get(
        uint32_t cbox_id,
        uint32_t motion_command_id,
        clink_float_t* p_speed_start);

/**
@brief      모션 커맨드의 시작 속도를 설정한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환 받은 값
@param[in]  speed_start
            시작 속도, 단위 : millimeter / second
@remark     시작 속도가 0 이 아닌 경우, \n
            이전 모션은 종료 속도와 진행 방향이 \n
            현재 모션의 시작 속도와 진행 방향과 \n
            동일 해야 한다.
@return     @ref clink_motion_command_robot_tcp_speed_start_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_motion_command_robot_tcp_speed_start_set(
        uint32_t cbox_id,
        uint32_t motion_command_id, 
        clink_float_t speed_start);

/**
@brief      모션 커맨드의 종료 속도를 반환한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환 받은 값
@param[out] p_speed_end
            종료 속도 (단위 : millimeter / second)
@return     @ref clink_motion_command_robot_tcp_speed_end_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_motion_command_robot_tcp_speed_end_get(
        uint32_t cbox_id,
        uint32_t motion_command_id,
        clink_float_t* p_speed_end);

/**
@brief      모션 커맨드의 종료 속도를 설정한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환 받은 값
@param[in]  speed_end
            종료 속도, 단위 : millimeter / second
@remark     종료 속도가 0 이 아닌 경우, \n
            현재 모션의 종료 속도와 진행 방향은 \n
            다음 모션의 시작 속도와 진행 방향과 \n
            동일 해야 한다.
@return     @ref clink_motion_command_robot_tcp_speed_end_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_motion_command_robot_tcp_speed_end_set(
        uint32_t cbox_id,
        uint32_t motion_command_id, 
        clink_float_t speed_end);

/**
@brief      모션 커맨드의 시작 위치를 반환한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환 받은 값
@param[out] p_pos_x
            시작 위치의 X 성분
@param[out] p_pos_y
            시작 위치의 Y 성분
@param[out] p_pos_z
            시작 위치의 Z 성분
@return     @ref clink_motion_command_robot_tcp_position_start_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_motion_command_robot_tcp_position_start_get(
        uint32_t cbox_id,
        uint32_t motion_command_id, 
        clink_float_t* p_pos_x, 
        clink_float_t* p_pos_y, 
        clink_float_t* p_pos_z);

/**
@brief      모션 커맨드의 시작 위치를 설정한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환 받은 값
@param[in]  pos_x
            시작 위치의 X 성분
@param[in]  pos_y
            시작 위치의 Y 성분
@param[in]  pos_z
            시작 위치의 Z 성분
@return     @ref clink_motion_command_robot_tcp_position_start_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_motion_command_robot_tcp_position_start_set(
        uint32_t cbox_id,
        uint32_t motion_command_id, 
        clink_float_t pos_x,
        clink_float_t pos_y,
        clink_float_t pos_z);

/**
@brief      모션 커맨드의 중간 위치를 반환한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환 받은 값
@param[out] p_pos_x
            중간 위치의 X 성분
@param[out] p_pos_y
            중간 위치의 Y 성분
@param[out] p_pos_z
            중간 위치의 Z 성분
@remark     Interpolator 가 \n
            CLINK_INTERPOLATOR_CIRCULAR 또는, \n
            CLINK_INTERPOLATOR_CIRCLE 또는, \n
            CLINK_INTERPOLATOR_ARC 일 때 유효하다.
@return     @ref clink_motion_command_robot_tcp_position_mid_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_motion_command_robot_tcp_position_mid_get(
        uint32_t cbox_id,
        uint32_t motion_command_id, 
        clink_float_t* p_pos_x,
        clink_float_t* p_pos_y,
        clink_float_t* p_pos_z);

/**
@brief      모션 커맨드의 중간 위치를 설정한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환 받은 값
@param[in]  pos_x
            중간 위치의 X 성분
@param[in]  pos_y
            중간 위치의 Y 성분
@param[in]  pos_z
            중간 위치의 Z 성분
@remark     Interpolator 가 \n
            CLINK_INTERPOLATOR_CIRCULAR 또는, \n
            CLINK_INTERPOLATOR_CIRCLE 또는, \n
            CLINK_INTERPOLATOR_ARC 일 때 유효하다.
@return     @ref clink_motion_command_robot_tcp_position_mid_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_motion_command_robot_tcp_position_mid_set(
        uint32_t cbox_id,
        uint32_t motion_command_id, 
        clink_float_t pos_x,
        clink_float_t pos_y,
        clink_float_t pos_z);

/**
@brief      모션 커맨드의 종료 위치를 반환한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환 받은 값
@param[out] p_pos_x
            종료 위치의 X 성분
@param[out] p_pos_y
            종료 위치의 Y 성분
@param[out] p_pos_z
            종료 위치의 Z 성분
@return     @ref clink_motion_command_robot_tcp_position_end_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_position_end_get(
        uint32_t cbox_id,
        uint32_t motion_command_id,
        clink_float_t* p_pos_x,
        clink_float_t* p_pos_y,
        clink_float_t* p_pos_z);

/**
@brief      모션 커맨드의 종료 위치를 설정한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환 받은 값
@param[in]  pos_x
            종료 위치의 X 성분
@param[in]  pos_y
            종료 위치의 Y 성분
@param[in]  pos_z
            종료 위치의 Z 성분
@return     @ref clink_motion_command_robot_tcp_position_end_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_position_end_set(
        uint32_t cbox_id,
		uint32_t motion_command_id,
		clink_float_t pos_x,
		clink_float_t pos_y,
		clink_float_t pos_z);

/**
@brief      모션 커맨드의 b-spline 포인트들을 설정한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환 받은 값
@param[in]  pos_x
            포인트 X 성분
@param[in]  pos_y
            포인트 Y 성분
@param[in]  pos_z
            포인트 Z 성분
@param[in]  point_count
			포인트 수
@return     @ref clink_motion_command_robot_tcp_free_path_points_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_free_path_points_set(
        uint32_t cbox_id,
	    uint32_t motion_command_id,
        uint32_t point_count,
		const clink_float_t* pos_x,
		const clink_float_t* pos_y,
		const clink_float_t* pos_z);

/**
@brief      현재 로봇의 위치에서 direction 방향으로 로봇이 갈 수 있는 끝 지점의 위치를
            종료 위치로 설정한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환 받은 값
@param[in]  dir_x
            방향 벡터 x 성분
@param[in]  dir_y
            방향 벡터 y 성분
@param[in]  dir_z
            방향 벡터 z 성분
@remark     시작 위치는 현재 로봇의 position을 기준으로 계산한다.
@return     @ref clink_motion_command_robot_tcp_position_end_set_from_direction_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_position_end_set_from_direction(
        uint32_t cbox_id,
        uint32_t motion_command_id,
        clink_float_t dir_x,
        clink_float_t dir_y,
        clink_float_t dir_z);

/**
@brief      현재 로봇의 위치에서 direction 방향으로 로봇이 갈 수 있는 끝 지점의 위치와 방향을
            종료 위치로 설정한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환 받은 값
@param[in]  dir_x
            방향 벡터 x 성분
@param[in]  dir_y
            방향 벡터 y 성분
@param[in]  dir_z
            방향 벡터 z 성분
@remark     시작 위치는 현재 로봇의 position 및 orientation 기준으로 계산한다.
            이 때, orientation의 위치는 변화하지 않는다.
@return     @ref clink_motion_command_robot_tcp_position_and_orientation_end_set_from_direction_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_position_and_orientation_end_set_from_direction(
        uint32_t cbox_id,
        uint32_t motion_command_id,
        clink_float_t dir_x,
        clink_float_t dir_y,
        clink_float_t dir_z);

/**
@brief      모션 커맨드의 시작 방향 반환한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환 받은 값
@param[out] p_ort_x
            시작 방향의 X 성분
@param[out] p_ort_y
            시작 방향의 Y 성분
@param[out] p_ort_z
            시작 방향의 Z 성분
@return     @ref clink_motion_command_robot_tcp_orientation_start_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_orientation_start_get(
        uint32_t cbox_id,
        uint32_t motion_command_id, 
        clink_float_t* p_ort_x, 
        clink_float_t* p_ort_y, 
        clink_float_t* p_ort_z);

/**
@brief      모션 커맨드의 시작 방향을 설정한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환 받은 값
@param[in]  ort_x
            시작 방향의 X 성분
@param[in]  ort_y
            시작 방향의 Y 성분
@param[in]  ort_z
            시작 방향의 Z 성분
@return     @ref clink_motion_command_robot_tcp_orientation_start_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_orientation_start_set(
        uint32_t cbox_id,
        uint32_t motion_command_id, 
        clink_float_t ort_x, 
        clink_float_t ort_y, 
        clink_float_t ort_z);

/**
@brief      모션 커맨드의 종료 방향을 반환한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환 받은 값
@param[out] p_ort_x
            종료 방향의 X 성분
@param[out] p_ort_y
            종료 방향의 Y 성분
@param[out] p_ort_z
            종료 방향의 Z 성분
@return     @ref clink_motion_command_robot_tcp_orientation_end_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_orientation_end_get(
        uint32_t cbox_id,
        uint32_t motion_command_id, 
        clink_float_t* p_ort_x, 
        clink_float_t* p_ort_y, 
        clink_float_t* p_ort_z);

/**
@brief      모션 커맨드의 종료 방향 설정한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환 받은 값
@param[in]  ort_x
            종료 방향의 X 성분
@param[in]  ort_y
            종료 방향의 Y 성분
@param[in]  ort_z
            종료 방향의 Z 성분
@return     @ref clink_motion_command_robot_tcp_orientation_end_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_orientation_end_set(
        uint32_t cbox_id,
        uint32_t motion_command_id, 
        clink_float_t ort_x, 
        clink_float_t ort_y, 
        clink_float_t ort_z);

/**
@brief      tcp motion 중 offset 모션 설정을 받아온다
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환 받은 값
@param[out] p_on_off
            설정 여부
@return     @ref clink_motion_command_robot_tcp_offset_mode_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_offset_mode_get(
        uint32_t cbox_id,
        uint32_t motion_command_id,
        CLINK_SWITCH* p_on_off);

/**
@brief      tcp motion 중 offset 모션 설정을 한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환 받은 값
@param[in]  on_off
            설정 여부
@return     @ref clink_motion_command_robot_tcp_offset_mode_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_offset_mode_set(
        uint32_t cbox_id,
        uint32_t motion_command_id,
        CLINK_SWITCH on_off);

/**
@brief      모션 커맨드의 시작 위치부터 종료 위치까지의 \n
            Interpolation 방법에 대항하는 enum 을 반환한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환 받은 값
@param[out] p_interpolator_name
            CLINK_INTERPOLATOR_LINEAR 또는, \n
            CLINK_INTERPOLATOR_CIRCULAR 또는, \n
            CLINK_INTERPOLATOR_ARC, 또는 \n
            CLINK_INTERPOLATOR_CIRCLE 반환
@return     @ref clink_motion_command_robot_tcp_interpolator_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_interpolator_get(
        uint32_t cbox_id,
        uint32_t motion_command_id,
        CLINK_INTERPOLATOR* p_interpolator_name);

/**
@brief      모션 커맨드의 시작 위치부터 종료 위치까지의 \n
            Interpolation 방법을 설정한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환 받은 값
@param[in]  interpolator_name
            CLINK_INTERPOLATOR_LINEAR : \n
            직선 보간 \n
            CLINK_INTERPOLATOR_CIRCULAR : \n
            시작, 중간, 종료 위치에 내접원을 그려 보간 \n
            CLINK_INTERPOLATOR_ARC : \n
            시작, 중간, 종료 위치에 외접원을 그려 보간 \n
            CLINK_INTERPOLATOR_CIRCLE : \n
            시작, 종료 위치를 지나는 완벽한 원을 그려 보간 \n
            중간 위치는 평면을 구성하는데 사용 \n

@return     @ref clink_motion_command_robot_tcp_interpolator_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_interpolator_set(
        uint32_t cbox_id,
        uint32_t motion_command_id, 
        CLINK_INTERPOLATOR interpolator_name);

/**
@brief      clink_motion_command_robot_tcp_radius_set() 을 통해 설정한 \n
            반지름 값을 반환한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환 받은 값
@param[out] p_radius
            원형의 반지름 (단위 millimeter)
@remark     Interpolator 가 \n
            CLINK_INTERPOLATOR_CIRCULAR 또는, \n
            CLINK_INTERPOLATOR_CIRCLE 또는, \n
            CLINK_INTERPOLATOR_ARC 일 때 유효하다.
@return     @ref clink_motion_command_robot_tcp_radius_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_radius_get(
        uint32_t cbox_id,
        uint32_t motion_command_id,
        clink_float_t* p_radius);

/**
@brief      모션 커맨드의 interpolator 가 CLINK_INTERPOLATOR_CIRCULAR, \n
            CLINK_INTERPOLATOR_CIRCLE, CLINK_INTERPOLATOR_ARC 인 경우, \n
            해당 원형의 반지름 값을 설정한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환 받은 값
@param[in]  radius
            원형의 반지름 (단위 millimeter)
@remark     Interpolator 가 \n
            CLINK_INTERPOLATOR_CIRCULAR 또는, \n
            CLINK_INTERPOLATOR_CIRCLE 또는, \n
            CLINK_INTERPOLATOR_ARC 일 때 유효하다.
@return     @ref clink_motion_command_robot_tcp_radius_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_radius_set(
        uint32_t cbox_id,
        uint32_t motion_command_id,
        clink_float_t radius);

/**
@brief      모션 커맨드의 기준 좌표계를 반환한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환 받은 값
@param[out] p_ref_coord
            모션 커맨드의 기준 좌표계  \n
            CLINK_REF_COORDINATE_BASE 또는, \n
            CLINK_REF_COORDINATE_TCP
@return     @ref clink_motion_command_robot_tcp_coordinate_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_coordinate_get(
        uint32_t cbox_id,
        uint32_t motion_command_id,
        CLINK_REF_COORDINATE* p_ref_coord);

/**
@brief      모션 커맨드의 기준 좌표계를 설정한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환 받은 값
@param[in]  ref_coord
            CLINK_REF_COORDINATE_BASE 또는, \n
            CLINK_REF_COORDINATE_TCP
@return     @ref clink_motion_command_robot_tcp_coordinate_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_coordinate_set(
        uint32_t cbox_id,
        uint32_t motion_command_id,
        CLINK_REF_COORDINATE ref_coord);

/**
@brief      모션 커맨드의 pose type를 설정한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환 받은 값
@param[in]  pose_type
            CLINK_POSE_TYPE 참조
@return     @ref clink_motion_command_robot_tcp_pose_type_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_pose_type_set(
        uint32_t cbox_id,
        uint32_t motion_command_id,
        CLINK_POSE_TYPE pose_type);

/**
@brief      지정되어 있는 orientation 보간 방법을 반환한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환 받은 값
@param[out] p_mode
            orientation 보간 방법  \n
            CLINK_ORIENTATION_MODE_SLERP 또는, \n
            CLINK_ORIENTATION_MODE_AXIS
@return     @ref clink_motion_command_robot_orientation_mode_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_orientation_mode_get(
        uint32_t cbox_id,
        uint32_t motion_command_id,
        CLINK_ORIENTATION_MODE* p_mode);

/**
@brief      orientation 보간 방법을 설정한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환 받은 값
@param[in]	mode
            orientation 보간 방법  \n
            CLINK_ORIENTATION_MODE_SLERP 또는, \n
            CLINK_ORIENTATION_MODE_AXIS
@return     @ref clink_motion_command_robot_orientation_mode_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_orientation_mode_set(
        uint32_t cbox_id,
        uint32_t motion_command_id,
        CLINK_ORIENTATION_MODE mode);

/**
@brief      자동 속도 조절 결과 반영된 속도를 받아온다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환 받은 값
@param[out]	velocity
            조절된 속도 값
@remark     해당 API는 clink_motion_command_queue_add() 호출 이후
            유효한 값을 받아올 수 있다.
@return     @ref clink_motion_command_robot_tcp_adjusted_speed_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_adjusted_speed_get(
        uint32_t cbox_id,
        uint32_t motion_command_id,
        clink_float_t* velocity);

/**
@brief      하나의 command ID 내에 tcp 모션을 추가한다.
@param[in]  motion_command_id
            모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_create() 를 통해 반환 받은 값
@return     @ref clink_motion_command_robot_tcp_internal_add_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_internal_add(
        uint32_t cbox_id,
        uint32_t motion_command_id);

/** @} */


//------------------------------------------------------------------
// 800_JUST
//------------------------------------------------------------------
/** \addtogroup 800_JUST 800_JUST
*  @{
*/

/**
@brief      입력한 pose (위치 및 방향) 으로 로봇을 이동시킨다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  speed_factor
            구동 속도 설정 값 \n
            최소 : 0.1 \n
            최대 : 1.0 \n
            최대 (즉, 1.0) 으로 입력한 경우 로봇은 \n
            spec 상의 최대 속도로 동작한다.
@param[in]  pos_x
            로봇 TCP 의 모션 종료시의 위치 중 X 성분
@param[in]  pos_y
            로봇 TCP 의 모션 종료시의 위치 중 Y 성분
@param[in]  pos_z
            로봇 TCP 의 모션 종료시의 위치 중 Z 성분
@param[in]  ort_x
            로봇 TCP 의 모션 종료시의 방향 중 X 성분
@param[in]  ort_y
            로봇 TCP 의 모션 종료시의 방향 중 Y 성분
@param[in]  ort_z
            로봇 TCP 의 모션 종료시의 방향 중 Z 성분
@return     @ref clink_just_robot_tcp_move_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_just_robot_tcp_move(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t speed_factor,
        clink_float_t pos_x,
        clink_float_t pos_y,
        clink_float_t pos_z,
        clink_float_t ort_x,
        clink_float_t ort_y,
        clink_float_t ort_z);

/**
@brief      입력한 pose (위치 및 방향) 으로 로봇을 이동시킨다. \n
            blending mode로 동작한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  speed_factor
            구동 속도 설정 값 \n
            최소 : 0.1 \n
            최대 : 1.0 \n
            최대 (즉, 1.0) 으로 입력한 경우 로봇은 \n
            spec 상의 최대 속도로 동작한다.
@param[in]  pos_x
            로봇 TCP 의 모션 종료시의 위치 중 X 성분
@param[in]  pos_y
            로봇 TCP 의 모션 종료시의 위치 중 Y 성분
@param[in]  pos_z
            로봇 TCP 의 모션 종료시의 위치 중 Z 성분
@param[in]  ort_x
            로봇 TCP 의 모션 종료시의 방향 중 X 성분
@param[in]  ort_y
            로봇 TCP 의 모션 종료시의 방향 중 Y 성분
@param[in]  ort_z
            로봇 TCP 의 모션 종료시의 방향 중 Z 성분
@return     @ref clink_just_robot_blending_tcp_move_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_just_robot_blending_tcp_move(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t speed_factor,
        clink_float_t pos_x,
        clink_float_t pos_y,
        clink_float_t pos_z,
        clink_float_t ort_x,
        clink_float_t ort_y,
        clink_float_t ort_z);

/**
@brief      입력한 joint angle 로 로봇을 이동시킨다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  speed_factor
            구동 속도 설정 값 \n
            최소 : 0.1 \n
            최대 : 1.0 \n
            최대 (즉, 1.0) 으로 입력한 경우 로봇은 \n
            spec 상의 최대 속도로 동작한다.
@param[in]  num_of_joints
            joint_angles 배열의 크기 \n
            로봇의 joint 개수 이상이어야 한다. \n
@param[in]  joint_angles
            각 joint 의 모션 종료 각도 \n
            (단위 : degree) \n
            움직이지 않고 싶은 joint 의 경우에는 \n
            현재 joint 의 각도를 그대로 입력한다. \n
            joint_angles 배열의 size 는 로봇의 joint 개수 \n
            이상이어야 한다. 즉 6축 로봇인 경우 6개 이상의 \n
            배열 size 를 보장해야 한다.
@remark     joint_angles 배열의 size 가 로봇 joint 의 개수 \n
            이하인 경우, segmentation fault 를 발생시킬 수 있다.
@return     @ref clink_just_robot_joint_move_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_just_robot_joint_move(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t speed_factor,
        uint32_t num_of_joints,
        const clink_float_t* joint_angles);

/**
@brief      입력한 joint angle 로 로봇을 이동시킨다. \n
            blending mode로 동작한다.
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  speed_factor
            구동 속도 설정 값 \n
            최소 : 0.1 \n
            최대 : 1.0 \n
            최대 (즉, 1.0) 으로 입력한 경우 로봇은 \n
            spec 상의 최대 속도로 동작한다.
@param[in]  num_of_joints
            joint_angles 배열의 크기 \n
            로봇의 joint 개수 이상이어야 한다. \n
@param[in]  joint_angles
            각 joint 의 모션 종료 각도 \n
            (단위 : degree) \n
            움직이지 않고 싶은 joint 의 경우에는 \n
            현재 joint 의 각도를 그대로 입력한다. \n
            joint_angles 배열의 size 는 로봇의 joint 개수 \n
            이상이어야 한다. 즉 6축 로봇인 경우 6개 이상의 \n
            배열 size 를 보장해야 한다.
@remark     joint_angles 배열의 size 가 로봇 joint 의 개수 \n
            이하인 경우, segmentation fault 를 발생시킬 수 있다.
@return     @ref clink_just_robot_blending_joint_move_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_just_robot_blending_joint_move(
        uint32_t cbox_id,
        uint32_t robot_id,
        clink_float_t speed_factor,
        uint32_t num_of_joints,
        const clink_float_t* joint_angles);

/** @} */


//------------------------------------------------------------------
// 910_WELDING
//------------------------------------------------------------------
/** \addtogroup 910_WELDING 910_WELDING
*  @{
*/

/**
@brief      로봇 TCP 를 활용 용접을 위한 모션 커맨드를 생성한다.
@param[out] p_command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스의 ID
@return     @ref clink_motion_command_robot_tcp_welding_create_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_welding_create(
        uint32_t cbox_id,
        uint32_t* p_command_id);

/**
@brief      로봇 TCP 를 활용한 용접을 위해 설정된 weaving motion 사용 상태를 반환한다.
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[out] p_on_off
            CLINK_SWITCH_ON : weaving motion 사용이 On 인 상태 \n
            CLINK_SWITCH_OFF: weaving motion 사용이 Off 인 상태
@return     @ref clink_motion_command_robot_tcp_welding_weaving_motion_switch_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_welding_weaving_motion_switch_get(
        uint32_t cbox_id,
        uint32_t command_id,
        CLINK_SWITCH* p_on_off);

/**
@brief      로봇 TCP 를 활용한 용접을 위한 weaving motion 사용 상태를 설정한다.
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[in]  on_off
            CLINK_SWITCH_ON : weaving motion 사용을 On 상태 \n
            CLINK_SWITCH_OFF: weaving motion 사용을 Off 상태
@return     @ref clink_motion_command_robot_tcp_welding_weaving_motion_switch_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_welding_weaving_motion_switch_set(
        uint32_t cbox_id,
        uint32_t command_id,
        CLINK_SWITCH on_off);

/**
@brief      로봇 TCP 를 활용한 용접을 위한 모션 type 설정을 반환한다.
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[out] p_pattern
            설정되어 있는 용접을 위한 모션 type (enum CLINK_WELDING_WEAVING_PATTERN 참조)
@return     @ref clink_motion_command_robot_tcp_welding_weaving_pattern_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_welding_weaving_pattern_get(
        uint32_t cbox_id,
        uint32_t command_id,
        CLINK_WELDING_WEAVING_PATTERN* p_pattern);

/**
@brief      로봇 TCP 를 활용한 용접을 위한 모션 type을 설정한다.
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[in]  pattern
            용접을 위한 모션 type (enum CLINK_WELDING_WEAVING_PATTERN 참조)
@return     @ref clink_motion_command_robot_tcp_welding_weaving_pattern_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_welding_weaving_pattern_set(
        uint32_t cbox_id,
        uint32_t command_id,
        CLINK_WELDING_WEAVING_PATTERN pattern);

/**
@brief      로봇 TCP 를 활용한 용접을 위해 설정된 왼쪽 진폭 값을 반환한다.
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[out] p_value
            설정되어 있는 왼쪽 진폭 값 (mm)
@return     @ref clink_motion_command_robot_tcp_welding_left_amplitude_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_welding_left_amplitude_get(
        uint32_t cbox_id,
        uint32_t command_id,
        clink_float_t* p_value);

/**
@brief      로봇 TCP 를 활용한 용접을 위한 왼쪽 진폭 값을 설정한다.
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[in]  value
            설정하는 왼쪽 진폭 값 (mm)
@return     @ref clink_motion_command_robot_tcp_welding_left_amplitude_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_welding_left_amplitude_set(
        uint32_t cbox_id,
        uint32_t command_id,
        clink_float_t value);

/**
@brief      로봇 TCP 를 활용한 용접을 위해 설정된 오른쪽 진폭 값을 반환한다.
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[out] p_value
            설정되어 있는 오른쪽 진폭 값 (mm)
@return     @ref clink_motion_command_robot_tcp_welding_right_amplitude_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_welding_right_amplitude_get(
        uint32_t cbox_id,
        uint32_t command_id,
        clink_float_t* p_value);

/**
@brief      로봇 TCP 를 활용한 용접을 위한 오른쪽 진폭 값을 설정한다.
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[in]  value
            설정하는 오른쪽 진폭 값 (mm)
@return     @ref clink_motion_command_robot_tcp_welding_right_amplitude_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_welding_right_amplitude_set(
        uint32_t cbox_id,
        uint32_t command_id,
        clink_float_t value);

/**
@brief      로봇 TCP 를 활용한 용접을 위해 설정된 max speed 에서 sine파 또는 zigzag파형의 주파수 값을 반환한다.
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[out] p_value
            설정되어 있는 주파수 값 (Hz)
@return     @ref clink_motion_command_robot_tcp_welding_frequency_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_welding_frequency_get(
        uint32_t cbox_id,
        uint32_t command_id,
        clink_float_t* p_value);

/**
@brief      로봇 TCP 를 활용한 용접을 위한 max speed 에서 sine파 또는 zigzag파형의 주파수 값을 설정한다.
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[in]  value
            설정하는 주파수 값 (Hz)
@return     @ref clink_motion_command_robot_tcp_welding_frequency_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_welding_frequency_set(
        uint32_t cbox_id,
        uint32_t command_id,
        clink_float_t value);

/**
@brief      zigzag weaving motion 에서 한 주기당 사다리꼴 윗변의 비율을 받아온다.
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[out] p_rise
            한 파장에서 올라가는 구간의 비율
@param[out] p_upper_body
            한 파장에서 윗변이 차지하는 비율
@param[out] p_fall
            한 파장에서 내려가는 구간의 비율
@param[out] p_below_body
            한 파장에서 아랫변이 차지하는 비율
@return     @ref clink_motion_command_robot_tcp_welding_zigzag_ratio_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_welding_zigzag_ratio_get(
        uint32_t cbox_id,
        uint32_t command_id,
        clink_float_t* p_rise,
        clink_float_t* p_upper_body,
        clink_float_t* p_fall,
        clink_float_t* p_below_body);

/**
@brief      zigzag weaving motion 에서 한 주기당 사다리꼴 윗변의 비율을 설정한다.
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[in]  rise
            한 파장에서 올라가는 구간의 비율
@param[in]  upper_body
            한 파장에서 윗변이 차지하는 비율
@param[in]  fall
            한 파장에서 내려가는 구간의 비율
@param[in]  below_body
            한 파장에서 아랫변이 차지하는 비율
@remark     rise, upper_body, fall, below_body 값의 총 합은 1 이 되어야 한다.
@return     @ref clink_motion_command_robot_tcp_welding_zigzag_ratio_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_welding_zigzag_ratio_set(
        uint32_t cbox_id,
        uint32_t command_id,
        clink_float_t rise,
        clink_float_t upper_body,
        clink_float_t fall,
        clink_float_t below_body);

/**
@brief      로봇 TCP 를 활용한 용접을 위해 설정된 왼쪽 진폭의 끝에서 머무는 시간을 반환한다.
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[out] p_value
            설정되어 있는 왼쪽 진폭의 끝에서 머무는 시간 (second)
@return     @ref clink_motion_command_robot_tcp_welding_left_stop_settling_time_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_welding_left_stop_settling_time_get(
        uint32_t cbox_id,
        uint32_t command_id,
        clink_float_t* p_value);

/**
@brief      로봇 TCP 를 활용한 용접을 위한 왼쪽 진폭의 끝에서 머무는 시간을 설정한다.
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[in]  value
            왼쪽 진폭의 끝에서 머무는 시간 (second)
@return     @ref clink_motion_command_robot_tcp_welding_left_stop_settling_time_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_welding_left_stop_settling_time_set(
        uint32_t cbox_id,
        uint32_t command_id,
        clink_float_t value);

/**
@brief      로봇 TCP 를 활용한 용접을 위해 설정된 오른쪽 진폭의 끝에서 머무는 시간을 반환한다.
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[out] p_value
            설정되어 있는 오른쪽 진폭의 끝에서 머무는 시간 (second)
@return     @ref clink_motion_command_robot_tcp_welding_right_stop_settling_time_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_welding_right_stop_settling_time_get(
        uint32_t cbox_id,
        uint32_t command_id,
        clink_float_t* p_value);

/**
@brief      로봇 TCP 를 활용한 용접을 위한 오른쪽 진폭의 끝에서 머무는 시간을 설정한다.
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[in]  value
            오른쪽 진폭의 끝에서 머무는 시간 (second)
@return     @ref clink_motion_command_robot_tcp_welding_right_stop_settling_time_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_welding_right_stop_settling_time_set(
        uint32_t cbox_id,
        uint32_t command_id,
        clink_float_t value);

/**
@brief      로봇 TCP 를 활용한 용접을 위해 설정된 중간에서 머무는 시간을 반환한다.
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[out] p_value
            설정되어 있는 중간지점에서 머무는 시간 (second)
@return     @ref clink_motion_command_robot_tcp_welding_middle_stop_settling_time_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_welding_middle_stop_settling_time_get(
        uint32_t cbox_id,
        uint32_t command_id,
        clink_float_t* p_value);

/**
@brief      로봇 TCP 를 활용한 용접을 위한 중간에서 머무는 시간을 설정한다.
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[in]  value
            중간지점에서 머무는 시간 (second)
@return     @ref clink_motion_command_robot_tcp_welding_middle_stop_settling_time_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_welding_middle_stop_settling_time_set(
        uint32_t cbox_id,
        uint32_t command_id,
        clink_float_t value);

/**
@brief      제자리에서 진동하는 weaving 모션 옵션을 받아온다.
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[out] p_on_off
            제자리에서 진동하는 weaving 모션 옵션을 지정한다. \n
            기본 값은 OFF
@param[out] p_length_x
            제자리에서 진동하는 옵션을 이용할 경우 용접 대상의 x 길이 반환
@param[out] p_length_y
            제자리에서 진동하는 옵션을 이용할 경우 용접 대상의 y 길이 반환
@param[out] p_length_z
            제자리에서 진동하는 옵션을 이용할 경우 용접 대상의 z 길이 반환 \n
            (OFF 로 설정할 경우 length 값은 사용되지 않는다)
@return     @ref clink_motion_command_robot_tcp_welding_position_fix_swtich_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_welding_position_fix_swtich_get(
        uint32_t cbox_id,
        uint32_t command_id,
        CLINK_SWITCH* p_on_off,
        clink_float_t* p_length_x, 
        clink_float_t* p_length_y, 
        clink_float_t* p_length_z);

/**
@brief      제자리에서 진동하는 weaving 모션 기능을 이용한다.
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[in]  on_off
            제자리에서 진동하는 weaving 모션 옵션을 지정한다. \n
            기본 값은 OFF
@param[in]  length_x
            제자리에서 진동하는 옵션을 이용할 경우, 용접 대상의 x 방향 길이. \n
            (OFF 로 설정할 경우 length 값은 사용되지 않는다)
@param[in]  length_y
            제자리에서 진동하는 옵션을 이용할 경우, 용접 대상의 y 방향 길이. \n
            (OFF 로 설정할 경우 length 값은 사용되지 않는다)
@param[in]  length_z
            제자리에서 진동하는 옵션을 이용할 경우, 용접 대상의 z 방향 길이. \n
            (OFF 로 설정할 경우 length 값은 사용되지 않는다)
@return     @ref clink_motion_command_robot_tcp_welding_position_fix_swtich_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_welding_position_fix_swtich_set(
        uint32_t cbox_id,
        uint32_t command_id,
        CLINK_SWITCH on_off,
        clink_float_t length_x, 
        clink_float_t length_y, 
        clink_float_t length_z);

/**
@brief      용접이 일어나는 평면의 기울기를 받아온다.
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[out] p_value
            평면 기울기 값 [degree]
@return     @ref clink_motion_command_robot_tcp_welding_plane_tilt_angle_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_welding_plane_tilt_angle_get(
        uint32_t cbox_id,
        uint32_t command_id,
        clink_float_t* p_value);

/**
@brief      용접이 일어나는 평면의 기울기를 설정한다.
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[in]  value
            평면 기울기 값 [degree]
@return     @ref clink_motion_command_robot_tcp_welding_plane_tilt_angle_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_welding_plane_tilt_angle_set(
        uint32_t cbox_id,
        uint32_t command_id,
        clink_float_t value);

/**
@brief      로봇 TCP 를 활용한 용접을 위한 왼쪽 진폭 값을 변경한다.
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[in]  value
            설정하는 왼쪽 진폭 값 (mm)
@return     @ref clink_motion_command_robot_tcp_welding_left_amplitude_change_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_welding_left_amplitude_change(
        uint32_t cbox_id,
        uint32_t command_id,
        clink_float_t value);

/**
@brief      로봇 TCP 를 활용한 용접을 위한 오른쪽 진폭 값을 변경한다.
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[in]  value
            설정하는 오른쪽 진폭 값 (mm)
@return     @ref clink_motion_command_robot_tcp_welding_right_amplitude_change_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_welding_right_amplitude_change(
        uint32_t cbox_id,
        uint32_t command_id,
        clink_float_t value);

/**
@brief      로봇 TCP 를 활용한 용접을 위한 max speed 에서 sine파 또는 zigzag파형의 주파수 값을 변경한다.
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[in]  value
            설정하는 주파수 값 (Hz)
@return     @ref clink_motion_command_robot_tcp_welding_frequency_change_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_welding_frequency_change(
        uint32_t cbox_id,
        uint32_t command_id,
        clink_float_t value);

/**
@brief      로봇 TCP 를 활용한 용접을 위한 왼쪽 진폭의 끝에서 머무는 시간을 변경한다.
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[in]  value
            왼쪽 진폭의 끝에서 머무는 시간 (second)
@return     @ref clink_motion_command_robot_tcp_welding_left_stop_settling_time_change_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_welding_left_stop_settling_time_change(
        uint32_t cbox_id,
        uint32_t command_id,
        clink_float_t value);

/**
@brief      로봇 TCP 를 활용한 용접을 위한 오른쪽 진폭의 끝에서 머무는 시간을 변경한다.
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[in]  value
            오른쪽 진폭의 끝에서 머무는 시간 (second)
@return     @ref clink_motion_command_robot_tcp_welding_right_stop_settling_time_change_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_welding_right_stop_settling_time_change(
        uint32_t cbox_id,
        uint32_t command_id,
        clink_float_t value);

/**
@brief      로봇 TCP 를 활용한 용접을 위한 중간지점에서 머무는 시간을 변경한다.
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[in]  value
            진폭 중간에서 머무는 시간 (second)
@return     @ref clink_motion_command_robot_tcp_welding_middle_stop_settling_time_change_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_welding_middle_stop_settling_time_change(
        uint32_t cbox_id,
        uint32_t command_id,
        clink_float_t value);

/**
@brief      진행하고 있는 용접 모션에 한해 상하 방향으로 offset position을 더한다.
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[in]  offset_distance
            상하 방향 거리, 양의 방향은 z축(아래쪽) 단위: [mm]
@return     @ref clink_motion_command_robot_tcp_welding_vertical_offset_pos_change_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_welding_vertical_offset_pos_change(
        uint32_t cbox_id,
        uint32_t command_id,
        clink_float_t offset_distance);

/**
@brief      진행하고 있는 용접 모션에 한 해 좌우 방향으로 offset position을 더한다.
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[in]  offset_distance
            좌우 방향 거리, 양의 방향은 좌측(진행하고 있는 방향을 바라봤을 때) 단위: [mm]
@return     @ref clink_motion_command_robot_tcp_welding_horizontal_offset_pos_change_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_welding_horizontal_offset_pos_change(
        uint32_t cbox_id,
        uint32_t command_id,
        clink_float_t offset_distance);

/**
@brief      진행하고 있는 용접 모션에 한 해 임의의 방향으로 offset position을 더한다.
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[in]  dir_x
            방향 벡터 x 성분
@param[in]  dir_y
            방향 벡터 y 성분
@param[in]  dir_z
            방향 벡터 z 성분
@remark     시작 위치는 현재 로봇의 position을 기준으로 계산한다.
@return     @ref clink_motion_command_robot_tcp_welding_offset_pos_change_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_welding_offset_pos_change(
        uint32_t cbox_id,
        uint32_t command_id,
        clink_float_t dir_x,
        clink_float_t dir_y,
        clink_float_t dir_z);

/**
@brief      arc sensing을 위한 analog data source를 반환한다.
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[out] source
            현재 ECAT IO analog input 0, 1 지원
@return     @ref clink_motion_command_robot_tcp_arc_sensing_data_source_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_arc_sensing_data_source_get(
        uint32_t cbox_id,
        uint32_t command_id,
        CLINK_ARC_SENSING_SRC* source);

/**
@brief      arc sensing을 위한 analog data source를 지정한다.
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[in]  source
            현재 ECAT IO analog input 0, 1 지원
@return     @ref clink_motion_command_robot_tcp_arc_sensing_data_source_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_arc_sensing_data_source_set(
        uint32_t cbox_id,
        uint32_t command_id,
        CLINK_ARC_SENSING_SRC source);

/**
@brief      arc sensing을 위한 data 정렬 tuning factor
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[out] factor
            shift 만큼의 시간 입력 unit: [s]
@return     @ref clink_motion_command_robot_tcp_arc_sensing_reference_shift_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_arc_sensing_reference_shift_get(
        uint32_t cbox_id,
        uint32_t command_id,
        clink_float_t* factor);

/**
@brief      arc sensing을 위한 data 정렬 tuning factor
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[in]  factor
            shift 만큼의 시간 입력 unit: [s], default 값은 0
@remark     weaving 모션이 좌측, 우측을 번갈아가면서 진행할 때 data 취득은 느릴 수 있는 만큼 \n
            해당 delay를 보정한다.
@return     @ref clink_motion_command_robot_tcp_arc_sensing_reference_shift_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_arc_sensing_reference_shift_set(
        uint32_t cbox_id,
        uint32_t command_id,
        clink_float_t factor);

/**
@brief      arc sensing 보정 기능 ON/OFF
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[in]  direction
            좌우 또는 상하 선택
@param[out] on_off
            ON/OFF 값
@return     @ref clink_motion_command_robot_tcp_arc_sensing_switch_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_arc_sensing_switch_get(
        uint32_t cbox_id,
        uint32_t command_id,
        CLINK_ARC_SENSING_DIRECTION direction,
        CLINK_SWITCH* on_off);

/**
@brief      arc sensing 보정 기능 ON/OFF
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[in]  direction
            좌우 또는 상하 선택
@param[in]  on_off
            ON/OFF 설정
@return     @ref clink_motion_command_robot_tcp_arc_sensing_switch_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_arc_sensing_switch_set(
        uint32_t cbox_id,
        uint32_t command_id,
        CLINK_ARC_SENSING_DIRECTION direction,
        CLINK_SWITCH on_off);

/**
@brief      arc sensing에서 reference 신호를 잡기위한 시작 주기
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[in]  direction
            좌우 또는 상하 선택
@param[out] period
            reference 주기
@return     @ref clink_motion_command_robot_tcp_arc_sensing_start_ref_period_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_arc_sensing_start_ref_period_get(
        uint32_t cbox_id,
        uint32_t command_id,
        CLINK_ARC_SENSING_DIRECTION direction,
        uint32_t* period);

/**
@brief      arc sensing에서 reference 신호를 잡기위한 시작 주기
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[in]  direction
            좌우 또는 상하 선택
@param[in]  period
            reference 주기, default 값은 0
@remark     입력 주기 이전의 신호는 무시된다. 해당 신호부터 기준을 잡기 시작한다.
@return     @ref clink_motion_command_robot_tcp_arc_sensing_start_ref_period_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_arc_sensing_start_ref_period_set(
        uint32_t cbox_id,
        uint32_t command_id,
        CLINK_ARC_SENSING_DIRECTION direction,
        uint32_t period);

/**
@brief      arc sensing에서 보정 시작 주기
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[in]  direction
            좌우 또는 상하 선택
@param[out] period
            추적 시작 주기
@return     @ref clink_motion_command_robot_tcp_arc_sensing_start_tracking_period_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_arc_sensing_start_tracking_period_get(
        uint32_t cbox_id,
        uint32_t command_id,
        CLINK_ARC_SENSING_DIRECTION direction,
        uint32_t* period);

/**
@brief      arc sensing에서 보정 시작 주기
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[in]  direction
            좌우 또는 상하 선택
@param[in]  period
            추적 시작 주기, default 값은 5
@remark     실제 시작은 (start ref period + tracking period) 부터 시작된다.
@return     @ref clink_motion_command_robot_tcp_arc_sensing_start_tracking_period_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_arc_sensing_start_tracking_period_set(
        uint32_t cbox_id,
        uint32_t command_id,
        CLINK_ARC_SENSING_DIRECTION direction,
        uint32_t period);

/**
@brief      arc sensing total 보정 제한량
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[in]  direction
            좌우 또는 상하 선택
@param[out] distance
            보정이 가능한 max distance unit: [mm]
@return     @ref clink_motion_command_robot_tcp_arc_sensing_total_limit_distance_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_arc_sensing_total_limit_distance_get(
        uint32_t cbox_id,
        uint32_t command_id,
        CLINK_ARC_SENSING_DIRECTION direction,
        clink_float_t* distance);

/**
@brief      arc sensing total 보정 제한량
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[in]  direction
            좌우 또는 상하 선택
@param[in]  distance
            보정이 가능한 max distance unit: [mm], default 값은 100mm
@return     @ref clink_motion_command_robot_tcp_arc_sensing_total_limit_distance_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_arc_sensing_total_limit_distance_set(
        uint32_t cbox_id,
        uint32_t command_id,
        CLINK_ARC_SENSING_DIRECTION direction,
        clink_float_t distance);

/**
@brief      arc sensing 한 회당 보정 제한량
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[in]  direction
            좌우 또는 상하 선택
@param[out] distance
            한번 보정이 일어날 때마다 가능한 max distance unit: [mm]
@return     @ref clink_motion_command_robot_tcp_arc_sensing_limit_distance_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_arc_sensing_limit_distance_get(
        uint32_t cbox_id,
        uint32_t command_id,
        CLINK_ARC_SENSING_DIRECTION direction,
        clink_float_t* distance);

/**
@brief      arc sensing 한 회당 보정 제한량
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[in]  direction
            좌우 또는 상하 선택
@param[in]  distance
            한번 보정이 일어날 때마다 가능한 max distance unit: [mm], default 값은 5mm
@return     @ref clink_motion_command_robot_tcp_arc_sensing_limit_distance_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_arc_sensing_limit_distance_set(
        uint32_t cbox_id,
        uint32_t command_id,
        CLINK_ARC_SENSING_DIRECTION direction,
        clink_float_t distance);

/**
@brief      arc sensing 보정 비율 설정
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[in]  direction
            좌우 또는 상하 선택
@param[out] factor
            보정이 일어나는 비율
@return     @ref clink_motion_command_robot_tcp_arc_sensing_weighted_factor_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_arc_sensing_weighted_factor_get(
        uint32_t cbox_id,
        uint32_t command_id,
        CLINK_ARC_SENSING_DIRECTION direction,
        clink_float_t* factor);

/**
@brief      arc sensing 보정 비율 설정
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[in]  direction
            좌우 또는 상하 선택
@param[in]  factor
            보정이 일어나는 비율을 조절한다. default 값은 1.0
@return     @ref clink_motion_command_robot_tcp_arc_sensing_weighted_factor_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_arc_sensing_weighted_factor_set(
        uint32_t cbox_id,
        uint32_t command_id,
        CLINK_ARC_SENSING_DIRECTION direction,
        clink_float_t factor);

/**
@brief      arc sensing dead band 값 설정
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[in]  direction
            좌우 또는 상하 선택
@param[out] p_value
            보정량 bias [mm/s]
@return     @ref clink_motion_command_robot_tcp_arc_sensing_dead_band_get_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_arc_sensing_dead_band_get(
        uint32_t cbox_id,
        uint32_t command_id,
        CLINK_ARC_SENSING_DIRECTION direction,
        clink_float_t* p_value);

/**
@brief      arc sensing dead band 값 설정
@param[in]  command_id
            로봇 TCP 를 위한 모션 커맨드 인스턴스 ID \n
            clink_rpc_motion_command_robot_tcp_welding_create() 를 통해 반환받은 값
@param[in]  direction
            좌우 또는 상하 선택
@param[in]  value
            보정량 결과가 value 보다 작게 나오면 보정을 하지 않는다. default 값은 0. 단위 [mm/s]
@return     @ref clink_motion_command_robot_tcp_arc_sensing_dead_band_set_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_arc_sensing_dead_band_set(
        uint32_t cbox_id,
        uint32_t command_id,
        CLINK_ARC_SENSING_DIRECTION direction,
        clink_float_t value);

/**
@brief      가장 최근에 진행완료된 arc sensing 데이터를 파일로 저장한다.
@param[in]  p_path
            파일 저장 path
@return     @ref clink_motion_command_robot_tcp_arc_sensing_data_save_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_motion_command_robot_tcp_arc_sensing_data_save(
        uint32_t cbox_id,
        const char_t* p_path);

/** @} */


//------------------------------------------------------------------
// 920_USER_MEMORY
//------------------------------------------------------------------
/** \addtogroup 920_USER_MEMORY 920_USER_MEMORY
*  @{
*/

/**
@brief      key, value 으로 이루어진 사용자 데이터를 hash map 기반하여 저장한다.
@param[in]  key
            사용자 데이터 저장 KEY
@param[in]	type
            사용자 데이터 타입
@param[in]	int_value
            integer 형 값
@param[in]	str_value
            string 형 값
@param[in]	bool_value
            boolean 형 값
@param[in]	float_value
            double 형 값
@param[in]	longlong_value
            longlong 형 값
@return     @ref clink_set_user_memory_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_set_user_memory(
        uint32_t cbox_id,
        const char_t* key,
        CLINK_USER_DATA_TYPE type,
        int32_t int_value,
        const char_t* str_value,
        char_t bool_value,
        clink_float_t float_value,
        int64_t longlong_value);

/**
@brief      저장된 사용자 데이터를 key 값으로 검색하여 반환한다.
@param[in]  key
            검색할 사용자 데이터의 KEY
@param[out] p_type
            저장된 사용자 데이터 타입
@param[out]	p_int_value
            integer 형 값
@param[out]	p_str_value
            string 형 값
@param[out]	p_bool_value
            boolean 형 값
@param[out]	p_double_value
            double 형 값
@param[out]	p_longlong_value
            long long 형 값
@return     @ref clink_get_user_memory_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_get_user_memory(
        uint32_t cbox_id,
        const char_t* key,
        CLINK_USER_DATA_TYPE* p_type,
        int32_t* p_int_value,
        char_t* p_str_value,
        char_t* p_bool_value,
        clink_float_t* p_double_value,
        int64_t* p_longlong_value);

/**
@brief      사용자 데이터를 삭제한다.
@param[in]  key
            검색할 사용자 데이터의 KEY
@return     @ref clink_delete_user_memory_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_delete_user_memory(
        uint32_t cbox_id,
        const char_t* key);

/**
@brief      사용자 데이터 저장소를 초기화한다.
@return     @ref clink_clear_user_memory_return "returnable error code"
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_clear_user_memory(
        uint32_t cbox_id);

/** @} */


#ifdef __cplusplus
}
#endif

#endif // __CLINK_API_CALL_RPC_H__


