#------------------------------------------------------------------------------
#
# Clink RPC Client API
# Copyright © Hanwha. All rights reserved.
#
# Incheol Jeong (ic.jeong at hanwha.com)
# Kiho Jo (kiho0520 at hanwha.com)
# Yongjin Kim (kim.jy at hanwha.com)
# Hyeyeon Park (hyeyeon.park at hanwha.com)
# Sungpil Mun (sungpil.mun at hanwha.com)
#
#------------------------------------------------------------------------------

if __name__ == "__main__":
    exit()

import sys, os, time

abs_file_path = os.path.dirname(__file__)
bin_dir = os.path.abspath(os.path.join(abs_file_path, "..", ".."))
bin_dir = os.path.join(bin_dir, "bin")

if hasattr(os, "add_dll_directory"):
    os.add_dll_directory(bin_dir)

sys.path.insert(0, bin_dir)
import clink_rpc_api

#---------------------------------------------- User 환경에 맞게 설정 ----------------------------------------------

# 실제 로봇과 연결할 경우 SIMULATION_MODE = false
SIMULATION_MODE = True
CBOX_MODEL_NAME = clink_rpc_api.CLINK_CBOX_MODEL_3GEN_SIM
ROBOT_MODEL_NAME = clink_rpc_api.CLINK_ROBOT_MODEL_HCR14
if not SIMULATION_MODE:
    CBOX_MODEL_NAME = clink_rpc_api.CLINK_CBOX_MODEL_3GEN

TARGET_IPADDR	= "192.168.100.123"				   # 제어SW 및 RPC Server가 실행되는 Target PC의 IP Adress
CONFIG_FILE_NAME = "../config/config_rpc.ini"       # ${base_dir}/config/config_rpc.ini 의 경로

#-----------------------------------------------------------------------------------------------------------------

NUM_OF_JOINTS = 6
cbox_id = 0
robot_id = 0

def move_homing():
    clink_rpc_api.clink_rpc_system_event_queue_clear(cbox_id)

    # Home 자세로 이동한다.
    # Home 자세는 로봇의 가장 기본이 되는 자세로 각도가 정해져 있다.
    angles = clink_rpc_api.clink_float_array(NUM_OF_JOINTS)
    angles[0] = 0.0
    angles[1] = -90.0
    angles[2] = -90.0
    angles[3] = -90.0
    angles[4] = 90.0
    angles[5] = 0.0

    # 간단한 함수로 로봇 joint move를 실행해 볼 수 있는 API
    err_ret_val = clink_rpc_api.clink_rpc_just_robot_joint_move(
        cbox_id,
        robot_id,
        0.5,
        NUM_OF_JOINTS,
        angles.cast())

    # 로봇 모션이 끝날 때까지 wait
    # 해당 함수는 지정된 이벤트가 도착할 때까지 지정된 시간만큼 blocking
    valid_event = -1
    err, valid_event = clink_rpc_api.clink_rpc_system_wait_event_group_subgroup(
        cbox_id,                                                                          # control box ID
        clink_rpc_api.CLINK_EVENT_GRP_MOTION_COMMAND,                                     # event group
        clink_rpc_api.CLINK_EVENT_SUBGRP_MOTION_COMMAND_SETTLED_DOWN,                     # event subgroup
        10000,                                                                        # 대기 시간 [ms]
        1)                                                                            # 해당 이벤트를 내부 이벤트 큐에서 삭제한다.


def clink_rpc_event_callback_func(
    event_grp,
    event_subgrp,
    event_sender_id,
    reserve01,
    reserve02,
    reserve03,
    reserve04,
    reserve05):

    # 이벤트 수신시, 실행되는 callback 함수
    # 이벤트 종류에 따라 필요한 작업을 수행할 수 있다.
    # 이벤트에 대한 사항은 clink_rpc_api help 문서의 Modules/40_API/00CONSTANT 내
    # CLINK_EVENT_GRP 및 CLINK_EVENT_SUBGRP 항목을 참조한다.
    # error 관련 이벤트는 clink_rpc_error_event_desc.xml 문서도 참조 가능하다.

    print("[REGISTERD CALLBACK FUNC] evt grp: ",
       str(event_grp),
       ", subgrp: " ,
       str(event_subgrp),
       ", sender ID: ",
       str(event_sender_id))


#-- for every TEST binary
# 모든 예제에서 공통적으로 들어가는 루틴
# 해당 함수에서는 순서대로
# System 초기화, 제어권 획득, 로봇 생성, 이벤트 핸들러 등록
# 순으로 함수가 호출된다.
def setup():
    global cbox_id
    global robot_id

    # 모든 API는 기본적으로 error code를 반환한다.
    # error code 에 대한 사항은
    # clink_rpc_api help 문서의 Modules/40_API/CLINK_API_RESULT 항목 또는
    # clink_rpc_error_api_return_desc.xml 문서를 참조한다.
    err_ret_val = clink_rpc_api.CLINK_API_RESULT_OK

    ret = True
    # RPC Server 연결
    err_ret_val, cbox_id = clink_rpc_api.clink_rpc_system_cbox_connect(CONFIG_FILE_NAME, TARGET_IPADDR)
    time.sleep(1)

    # 제어SW 초기화 API 호출
    err_ret_val = clink_rpc_api.clink_rpc_gen_system_create(cbox_id, CONFIG_FILE_NAME, CBOX_MODEL_NAME)
    time.sleep(3)

    # CLINK_API_RESULT_WARNING_SYSTEM_ALREADY_CREATED WARNING은 RPC Server 에서
    # 이미 호출되었을 때 발생
    if clink_rpc_api.CLINK_API_RESULT_OK != err_ret_val and clink_rpc_api.CLINK_API_RESULT_WARNING_SYSTEM_ALREADY_CREATED != err_ret_val:
        ret = False
        print(f"제어SW 초기화 실패, code: {err_ret_val}")

    # 제어권 획득 API 호출
    if ret:
        err_ret_val = clink_rpc_api.clink_rpc_system_control_take(cbox_id)
        if clink_rpc_api.CLINK_API_RESULT_OK != err_ret_val and clink_rpc_api.CLINK_API_RESULT_WARNING_YOU_DONT_HAVE_THE_CONTROL != err_ret_val:
            ret = False
            print(f"제어권 획득 실패, code: {err_ret_val}")

    # 이벤트 핸들러 등록
    # 제어SW에서 발생하는 각종 정보는 이벤트를 통해 API 측으로 고지된다.
    # 사용자는 이벤트 핸들러 등록을 통해서 이벤트에 대한 정보를 받고 처리할 수 있다.
    if ret:
        err_ret_val = clink_rpc_api.clink_rpc_system_event_callback_add(cbox_id, clink_rpc_event_callback_func)
        time.sleep(1)
        if clink_rpc_api.CLINK_API_RESULT_OK != err_ret_val:
            ret = False
            print(f"이벤트 callback 등록 실패, code: {err_ret_val}")

    # 로봇 생성 API 호출
    if ret:
        err_ret_val, robot_id = clink_rpc_api.clink_rpc_robot_create(cbox_id, ROBOT_MODEL_NAME, " ", 0)
        err_ret_val, ecat_stat = clink_rpc_api.clink_rpc_cbox_ecat_connection_state_get(cbox_id)
        if clink_rpc_api.CLINK_ECAT_CONN_STATE_CONNECTED != ecat_stat:
            valid_event = -1
            err_ret_val, valid_event = clink_rpc_api.clink_rpc_system_wait_event_group_subgroup(
                cbox_id,
                clink_rpc_api.CLINK_EVENT_GRP_NOTIFICATION,
                clink_rpc_api.CLINK_EVENT_SUBGRP_NOTIFICATION_ECAT_CONNECTED,
                1000000,
                1)

        if clink_rpc_api.CLINK_API_RESULT_OK != err_ret_val:
            ret = False
            print(f"로봇 생성 실패, code: {err_ret_val}")

    # 로봇 모션에 있어 속도/가속도 위반 에러를 감지하여 자동 속도 조절해주는 기능 ON
    if ret:
        clink_rpc_api.clink_rpc_robot_motion_auto_adjust_swith_set(cbox_id, robot_id, clink_rpc_api.CLINK_SWITCH_ON)
    return ret

def tear_down():
    # 등록했던 이벤트 핸들러 제거
    clink_rpc_api.clink_rpc_system_event_callback_remove(cbox_id, clink_rpc_event_callback_func)

    # 안전 상 로봇 stop 및 servo off 실행 후 종료
    clink_rpc_api.clink_rpc_robot_stop(cbox_id, robot_id, 0.5)
    time.sleep(1)
    clink_rpc_api.clink_rpc_robot_servo_switch_set(cbox_id, robot_id, clink_rpc_api.CLINK_SWITCH_OFF)
