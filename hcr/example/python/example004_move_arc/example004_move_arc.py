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
    # start -> mid -> end point 를 지나는 원호를 그리는 모션이다.
    # arc 모션은 circle 모션과 대부분 비슷하다.
    # circle 이 end pos를 지나 다시 start 위치까지 돌아온다면
    # arc는 end pos 에서 종료된다.
    # 예제는 반원 두개 명령으로 원을 그리는 모션이다.

    cbox_id = example000_main.cbox_id
    robot_id = example000_main.robot_id
    err_ret_val = clink_rpc_api.clink_rpc_robot_servo_switch_set(cbox_id, robot_id, clink_rpc_api.CLINK_SWITCH_ON)

    # home 으로 이동
    example000_main.move_homing()
    time.sleep(1)

    err_ret_val, cmd_pos_x, cmd_pos_y, cmd_pos_z, cmd_ort_x, cmd_ort_y, cmd_ort_z = clink_rpc_api.clink_rpc_robot_tcp_pose_command_get(cbox_id, robot_id)

    mid_pos1 = [0 for _ in range(3)]
    end_pos1 = [0 for _ in range(3)]
    mid_pos2 = [0 for _ in range(3)]
    end_pos2 = [0 for _ in range(3)]
    end_ort = [0 for _ in range(3)]

    mid_pos1[0] = cmd_pos_x + 100.0
    mid_pos1[1] = cmd_pos_y - 100.0
    mid_pos1[2] = cmd_pos_z

    end_pos1[0] = cmd_pos_x + 200.0
    end_pos1[1] = cmd_pos_y
    end_pos1[2] = cmd_pos_z

    mid_pos2[0] = cmd_pos_x + 100.0
    mid_pos2[1] = cmd_pos_y + 100.0
    mid_pos2[2] = cmd_pos_z

    end_pos2[0] = cmd_pos_x
    end_pos2[1] = cmd_pos_y
    end_pos2[2] = cmd_pos_z

    end_ort[0] = cmd_ort_x
    end_ort[1] = cmd_ort_y
    end_ort[2] = cmd_ort_z

    spd = 300           # [mm/s]
    acc = spd * 2.0     # [mm/s^2]
    jerk = 0.5          # 0 ~ 1 사이 값
    NUM_OF_ROTATION = 5
    for iter in range(0, NUM_OF_ROTATION):
        print(f"test loop: {str(iter)}")

        err_ret_val, cmd_id = clink_rpc_api.clink_rpc_motion_command_robot_tcp_create(cbox_id)
        err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_robot_id_set(cbox_id, cmd_id, robot_id)
        err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_speed_start_set(cbox_id, cmd_id, 0)
        err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_speed_end_set(cbox_id, cmd_id, 0)
        err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_position_mid_set(cbox_id, cmd_id, mid_pos1[0], mid_pos1[1], mid_pos1[2])
        err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_position_end_set(cbox_id, cmd_id, end_pos1[0], end_pos1[1], end_pos1[2])
        err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_orientation_end_set(cbox_id, cmd_id, end_ort[0], end_ort[1], end_ort[2])
        err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_interpolator_set(cbox_id, cmd_id, clink_rpc_api.CLINK_INTERPOLATOR_ARC)
        err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_coordinate_set(cbox_id, cmd_id, clink_rpc_api.CLINK_REF_COORDINATE_BASE)
        err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_orientation_mode_set(cbox_id, cmd_id, clink_rpc_api.CLINK_ORIENTATION_MODE_SLERP)
        err_ret_val = clink_rpc_api.clink_rpc_motion_command_velocity_profiler_set(cbox_id, cmd_id, clink_rpc_api.CLINK_VELOCITY_PROFILE_ASYMMETRIC_S_CURVE)
        err_ret_val = clink_rpc_api.clink_rpc_motion_command_acc_max_set(cbox_id, cmd_id, acc)
        err_ret_val = clink_rpc_api.clink_rpc_motion_command_dec_max_set(cbox_id, cmd_id, acc)
        err_ret_val = clink_rpc_api.clink_rpc_motion_command_speed_max_set(cbox_id, cmd_id, spd)
        err_ret_val = clink_rpc_api.clink_rpc_motion_command_jerk_percentage_set(cbox_id, cmd_id, jerk)
        err_ret_val = clink_rpc_api.clink_rpc_motion_command_blending_motion_mode_set(cbox_id, cmd_id, clink_rpc_api.CLINK_SWITCH_OFF)
        err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_pose_type_set(cbox_id, cmd_id, clink_rpc_api.CLINK_POSE_TYPE_FLANGE)
        err_ret_val = clink_rpc_api.clink_rpc_motion_command_queue_add(cbox_id, cmd_id)
        err_ret_val = clink_rpc_api.clink_rpc_motion_command_queue_flush(cbox_id)

        err_ret_val, valid_event = clink_rpc_api.clink_rpc_system_wait_event_group_subgroup_sender_id(
            cbox_id,
            clink_rpc_api.CLINK_EVENT_GRP_MOTION_COMMAND,
            clink_rpc_api.CLINK_EVENT_SUBGRP_MOTION_COMMAND_SETTLED_DOWN,
            cmd_id,
            1000000,
            1)

        # 아래와 같이 motion command create 이 후 재활용 가능
        # 하지만 같은 command ID를 쓰므로 command ID 관리를 위해 한번씩 쓰는 것을 추천
        err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_position_mid_set(cbox_id, cmd_id, mid_pos2[0], mid_pos2[1], mid_pos2[2])
        err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_position_end_set(cbox_id, cmd_id, end_pos2[0], end_pos2[1], end_pos2[2])
        err_ret_val = clink_rpc_api.clink_rpc_motion_command_queue_add(cbox_id, cmd_id)
        err_ret_val = clink_rpc_api.clink_rpc_motion_command_queue_flush(cbox_id)
        err_ret_val = clink_rpc_api.clink_rpc_motion_command_destroy(cbox_id, cmd_id)

        err_ret_val, cmd_id = clink_rpc_api.clink_rpc_system_wait_event_group_subgroup_sender_id(
            cbox_id,
            clink_rpc_api.CLINK_EVENT_GRP_MOTION_COMMAND,
            clink_rpc_api.CLINK_EVENT_SUBGRP_MOTION_COMMAND_SETTLED_DOWN,
            cmd_id,
            1000000,
            1)

    example000_main.move_homing()
    time.sleep(1)
    err_ret_val = clink_rpc_api.clink_rpc_robot_servo_switch_set(cbox_id, robot_id, clink_rpc_api.CLINK_SWITCH_OFF)
    time.sleep(1)

if __name__ == "__main__":
    result = example000_main.setup()
    if result:
        main()