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

def main():
    # linear 계열 jog 명령
    # x, y, z, rx, ry, rz jog 이동 명령
    # 또는 일반적으로 진행 벡터(x, y, z) 또는 회전 벡터(rx, ry, rz) 입력 가능
    # stop 명령을 통해 멈춘다.
    cbox_id = example000_main.cbox_id
    robot_id = example000_main.robot_id
    err_ret_val = clink_rpc_api.clink_rpc_robot_servo_switch_set(cbox_id, robot_id, clink_rpc_api.CLINK_SWITCH_ON)

    # home 으로 이동
    example000_main.move_homing()
    time.sleep(1)

    # 입력 값
    # 기준 좌표계(base 또는 tcp 끝단 기준)
    # 정방향/역방향
    # 속도 [mm/s]
    # 거리 [mm]
    clink_rpc_api.clink_rpc_robot_jog_tcp_position_x(cbox_id, robot_id, clink_rpc_api.CLINK_REF_COORDINATE_BASE, clink_rpc_api.CLINK_DIRECTION_POSITIVE, 30, 10000)
    time.sleep(1)
    clink_rpc_api.clink_rpc_robot_stop(cbox_id, robot_id, 0.5)
    time.sleep(1)

    # 입력 값
    # 기준 좌표계(base 또는 tcp 끝단 기준)
    # 정방향/역방향
    # 속도 [deg/s]
    # 거리 [deg]
    clink_rpc_api.clink_rpc_robot_jog_tcp_orientation_x(cbox_id, robot_id, clink_rpc_api.CLINK_REF_COORDINATE_BASE, clink_rpc_api.CLINK_DIRECTION_POSITIVE, 30, 10000)
    time.sleep(1)
    clink_rpc_api.clink_rpc_robot_stop(cbox_id, robot_id, 0.5)
    time.sleep(1)

    clink_rpc_api.clink_rpc_robot_servo_switch_set(cbox_id, robot_id, clink_rpc_api.CLINK_SWITCH_OFF)
    time.sleep(1)

if __name__ == "__main__":
    result = example000_main.setup()
    if result:
        main()