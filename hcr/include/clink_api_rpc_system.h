// ControLINK-API
// Copyright ⓒ Hanwha Robotics. All rights reserved.
// (https://www.hanwharobotics.co.kr/)
//
// Kiho     Cho    (kiho0520 at hanwha dot com)
// Incheol  Jeong  (ic.jeong at hanwha dot com)
// 
//------------------------------------------------------------------------------

/** \addtogroup 50_RPC_API 50_RPC_API
*  @{
*/

#ifndef __CLINK_API_RPC_CLIENT_SYSTEM_H__
#define __CLINK_API_RPC_CLIENT_SYSTEM_H__

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
@brief      특정 Control box와 TCP/IP 연결을 시도한다.
@param[in]  p_config_file_path
            설정 값 ini 파일
@param[in]  cbox_ip
            control box ip adress
@param[out] cbox_id
            return 되는 control box ID \n
            모든 API는 해당 cbox ID를 활용하여 특정 control box 에 명령을 내리거나 값을 받아온다.
@remark     rpc client API 활용 시 처츰으로 호출해야 한다.
*/
CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_system_cbox_connect(
        const char_t* p_config_file_path,
        const char_t* cbox_ip,
        uint32_t* cbox_id);

/** @} */


//------------------------------------------------------------------
// 390_ROBOT_JOINT_CHASER
//------------------------------------------------------------------
/** \addtogroup 390_ROBOT_JOINT_CHASER 390_ROBOT_JOINT_CHASER
*  @{
*/


/**
@brief      로봇의 Joint Position 추적 기능 상태를 받아온다.
@param[in]  cbox_id
			clink_rpc_system_cbox_connect() 를 통해 반환받은 Control Box ID
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[out] p_on_off
            설정 On/Off
@remark     RPC Client API 에만 존재하는 기능
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_joint_chaser_switch_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH* p_on_off);

/**
@brief      로봇의 Joint Position 추적 기능을 On/Off 한다.
@param[in]  cbox_id
			clink_rpc_system_cbox_connect() 를 통해 반환받은 Control Box ID
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  on_off
            설정 On/Off
@remark     RPC Client API 에만 존재하는 기능 \n
			route add 192.168.222.0 mask 255.255.255.0 [목적지 IP] \n
			ex) route add 192.168.222.0 mask 255.255.255.0 192.168.100.200 \n
            해당 기능은 네트워크 구조 상 하나의 Control Box 대상으로만 사용이 가능하다. \n
            2개 이상의 Control Box 접속 상태에서 사용 불가
            해당 기능 활성화 시 다른 모션 명령 삽입 금지
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_joint_chaser_switch_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        CLINK_SWITCH on_off); 

/**
@brief      로봇의 Joint Position 추적 기능을 수행할 로봇의 IP를 설정한다.
@param[in]  cbox_id
            clink_rpc_system_cbox_connect() 를 통해 반환받은 Control Box ID
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  ip
            Control Box 내부 INtime OS Virtual NIC 의 IP 입력
@remark     Default 값은 셋팅되어져 일반적인 환경에서 호출할 필요는 없다. \n
            다만 특수한 상황에서의 호출이 필요한 경우를 대비해 해당 API를 제공한다. \n
            ip 변경이 필요한 경우, 기능 ON 이전 최초 한 번 설정 필요
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_joint_chaser_ip_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        const char_t* ip);

/**
@brief      로봇의 Joint Position 추적을 실행한다. (해당 좌표로 로봇이 이동한다.)
@param[in]  cbox_id
            clink_rpc_system_cbox_connect() 를 통해 반환받은 Control Box ID
@param[in]  robot_id
            clink_rpc_robot_create() 를 통해 반환받은 로봇 인스턴스 ID
@param[in]  num_of_joints
            로봇의 총 Joint 갯수, 해당 갯수만큼 pos 배열을 입력한다.
@param[in]  p_pos
			이동 목적지 각 축의 joint position, num_of_joints 만큼의 배열을 입력한다.
@remark     RPC Client API 에만 존재하는 기능 \n
            실행 주기는 최소 2ms 이상 간격으로 호출하며 현재 로봇의 각도와의 차이를 최소한으로 하는 값을 할당해야 한다. \n
            급격한 각도 변화를 일으키는 입력은 드라이브 에러가 발생하거나 급격한 로봇의 움직임을 유발할 수 있으니 주의가 필요하다.
*/
CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_robot_joint_chaser_move(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t num_of_joints,
        const clink_float_t* p_pos);

/** @} */



#ifdef __cplusplus
}
#endif

#endif // __CLINK_API_RPC_CLIENT_SYSTEM_H__


/** @} */
