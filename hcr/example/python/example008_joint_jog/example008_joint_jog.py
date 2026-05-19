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
    # joint 별 jog 명령
    # jog 호출 시 입력한 거리만큼 진행한다,
    # 각도 범위보다 큰 값을 입력했을 때 각도 범위 값까지 진행한다.
    # stop 명령을 통해 멈춘다.
    cbox_id = example000_main.cbox_id
    robot_id = example000_main.robot_id
    err_ret_val = clink_rpc_api.clink_rpc_robot_servo_switch_set(cbox_id, robot_id, clink_rpc_api.CLINK_SWITCH_ON)

    # home 으로 이동
    example000_main.move_homing()
    time.sleep(1)

    # 입력 값
    # joint 축 index [0~5] 범위
    # 정방향/역방향
    # 속도 [degree/s]
    # 거리 [degree]
    clink_rpc_api.clink_rpc_robot_jog_joint(cbox_id, robot_id, 4, clink_rpc_api.CLINK_DIRECTION_POSITIVE, 30, 1000)
    time.sleep(3)
    clink_rpc_api.clink_rpc_robot_stop(cbox_id, robot_id, 0.5)
    time.sleep(1)

    clink_rpc_api.clink_rpc_robot_servo_switch_set(cbox_id, robot_id, clink_rpc_api.CLINK_SWITCH_OFF)
    time.sleep(1)

if __name__ == "__main__":
    result = example000_main.setup()
    if result:
        main()