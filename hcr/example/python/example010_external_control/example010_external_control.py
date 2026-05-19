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

import sys, os, time, math

abs_file_path = os.path.dirname(__file__)
sys.path.append(os.path.join(abs_file_path, "..", "..", "..", "bin"))
sys.path.append(os.path.join(abs_file_path, ".."))

import example000_main, clink_rpc_api

def main():
    # 모션 명령을 실행하는 즉시 모션이 blending 되어가면서 진행한다.
    # (앞의 모션이 진행중이더라도 앞선 모션과 섞이면서 바로 진행됨)
    # external control은 함수 호출을 하는 시점과
    # 입력 속도 값에 따라 모션이 달라진다.
    # 두 factor를 잘 조절하여 사용
    # 예제는 spiral 형태의 모션을 그린다.
    cbox_id = example000_main.cbox_id
    robot_id = example000_main.robot_id
    err_ret_val = clink_rpc_api.clink_rpc_robot_servo_switch_set(cbox_id, robot_id, clink_rpc_api.CLINK_SWITCH_ON)

    # home 으로 이동
    example000_main.move_homing()
    time.sleep(1)

    err_ret_val, cmd_pos_x, cmd_pos_y, cmd_pos_z, cmd_ort_x, cmd_ort_y, cmd_ort_z = clink_rpc_api.clink_rpc_robot_tcp_pose_command_get(cbox_id, robot_id)

    # external control 시작
    err_ret_val = clink_rpc_api.clink_rpc_robot_ext_move_switch_set(cbox_id, robot_id, clink_rpc_api.CLINK_SWITCH_ON)

    NUM_OF_MOTION = 200								    # 삽입할 모션의 총 갯수
    D_THETA = 360.0 / NUM_OF_MOTION * 10.0				# 총 10바퀴
    D_RADIUS_LENGTH = 150.0 / NUM_OF_MOTION			    # 최종 150mm 까지 반지름 증가
    D_Z_LENGTH = 50.0 / NUM_OF_MOTION					# z축은 최종 50mm 까지 증가
    PI = 3.14159265358979323846

    theta = 0.0
    radius = 0.0

    for i in range(0, NUM_OF_MOTION):
        x = cmd_pos_x + (D_RADIUS_LENGTH * i) * math.cos((D_THETA * i) * PI / 180.0)
        y = cmd_pos_y + (D_RADIUS_LENGTH * i) * math.sin((D_THETA * i) * PI / 180.0)
        z = cmd_pos_z + (D_Z_LENGTH * i)
        err_ret_val = clink_rpc_api.clink_rpc_robot_ext_move_pos_ort_euler_add(
            cbox_id,
            robot_id,
            clink_rpc_api.CLINK_POSE_TYPE_FLANGE,
            x,
            y,
            z,
            cmd_ort_x,
            cmd_ort_y,
            cmd_ort_z,
            60.0)
        time.sleep(0.2)

    time.sleep(1)
    err_ret_val = clink_rpc_api.clink_rpc_robot_stop(cbox_id, robot_id, 0.5)
    time.sleep(1)

    # external control 종료
    err_ret_val = clink_rpc_api.clink_rpc_robot_ext_move_switch_set(cbox_id, robot_id, clink_rpc_api.CLINK_SWITCH_OFF)

    example000_main.move_homing()
    time.sleep(1)
    err_ret_val = clink_rpc_api.clink_rpc_robot_servo_switch_set(cbox_id, robot_id, clink_rpc_api.CLINK_SWITCH_OFF)
    time.sleep(1)

if __name__ == "__main__":
    result = example000_main.setup()
    if result:
        main()