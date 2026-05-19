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

import sys, os, time

abs_file_path = os.path.dirname(__file__)
sys.path.append(os.path.join(abs_file_path, "..", "..", "..", "bin"))
sys.path.append(os.path.join(abs_file_path, ".."))

import example000_main, clink_rpc_api

NUM_OF_JOINTS = example000_main.NUM_OF_JOINTS

def CALL(f):
    err_val = f
    if clink_rpc_api.CLINK_API_RESULT_OK != err_val:
        print(f"error code: {err_val}")
    return err_val

def main():
    # 사용자가 지정한 조인트 각도[deg] 값들을 순차적으로 적용하여 진행한다.
    # 입력한 각도 값의 변화에 따라 조인트 단위의 동작을 확인할 수 있다.
    # 과도한 각도 변화는 에러로 처리될 수 있으며
    # 입력 값이 로봇 사양을 초과하는 경우, 각도 변화가 조정될 수 있다.
    # 본 예제는 J6 조인트를 약 30도 회전 시킨다.
    cbox_id = example000_main.cbox_id
    robot_id = example000_main.robot_id
    err_ret_val = CALL(clink_rpc_api.clink_rpc_robot_servo_switch_set(cbox_id, robot_id, clink_rpc_api.CLINK_SWITCH_ON))

    start_angles = clink_rpc_api.clink_float_array(NUM_OF_JOINTS)
    start_angles[0] = 0.0
    start_angles[1] = -90.0
    start_angles[2] = -90.0
    start_angles[3] = -90.0
    start_angles[4] = 90.0
    start_angles[5] = 0.0

    target_angle = clink_rpc_api.clink_float_array(NUM_OF_JOINTS)
    target_angle[0] = 0.0
    target_angle[1] = -90.0
    target_angle[2] = -90.0
    target_angle[3] = -90.0
    target_angle[4] = 90.0
    target_angle[5] = 0.0

    # 초기 각도로 이동
    CALL(clink_rpc_api.clink_rpc_just_robot_joint_move(
        cbox_id,
        robot_id,
        0.5,
        NUM_OF_JOINTS,
        start_angles.cast()))
    
    # 로봇 모션이 끝날 때까지 wait
    # 해당 함수는 지정된 이벤트가 도착할 때까지 지정된 시간만큼 blocking
    valid_event = -1
    err, valid_event = clink_rpc_api.clink_rpc_system_wait_event_group_subgroup(
        cbox_id,                                                                          # control box ID
        clink_rpc_api.CLINK_EVENT_GRP_MOTION_COMMAND,                                     # event group
        clink_rpc_api.CLINK_EVENT_SUBGRP_MOTION_COMMAND_SETTLED_DOWN,                     # event subgroup
        10000,                                                                        # 대기 시간 [ms]
        1)    

    # joint chaser 시작
    CALL(clink_rpc_api.clink_rpc_robot_joint_chaser_switch_set(cbox_id, robot_id, clink_rpc_api.CLINK_SWITCH_ON))

    for offset in range(0, 300, 2): # 약 30도까지
        target_angle[5] = start_angles[5] + offset * 0.1
        CALL(clink_rpc_api.clink_rpc_robot_joint_chaser_move(cbox_id, robot_id, NUM_OF_JOINTS, target_angle.cast()))
        time.sleep(0.015)

    # joint chaser 종료
    CALL(clink_rpc_api.clink_rpc_robot_joint_chaser_switch_set(cbox_id, robot_id, clink_rpc_api.CLINK_SWITCH_OFF))

    # home 으로 이동
    example000_main.move_homing()
    time.sleep(1)
    CALL(clink_rpc_api.clink_rpc_robot_servo_switch_set(cbox_id, robot_id, clink_rpc_api.CLINK_SWITCH_OFF))
    time.sleep(1)

if __name__ == "__main__":
    result = example000_main.setup()
    if result:
        main()