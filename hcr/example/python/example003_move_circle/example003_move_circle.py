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
    # circle은 mid pos 및 end pos에 의해서 궤적이 결정된다.
    # 일직선 상에 있지 않은 세 점은 하나의 원을 만든다.
    # start -> mid -> end point 를 지나 다시 start 위치까지
    # 원을 그리며 돌아오는 모션이다.

    cbox_id = example000_main.cbox_id
    robot_id = example000_main.robot_id
    err_ret_val = clink_rpc_api.clink_rpc_robot_servo_switch_set(cbox_id, robot_id, clink_rpc_api.CLINK_SWITCH_ON)

    # home 으로 이동
    example000_main.move_homing()
    time.sleep(1)

    err_ret_val, cmd_pos_x, cmd_pos_y, cmd_pos_z, cmd_ort_x, cmd_ort_y, cmd_ort_z = clink_rpc_api.clink_rpc_robot_tcp_pose_command_get(cbox_id, robot_id)

    mid_pos = [0 for _ in range(3)]
    end_pos = [0 for _ in range(3)]
    end_ort = [0 for _ in range(3)]

    mid_pos[0] = cmd_pos_x + 100.0
    mid_pos[1] = cmd_pos_y - 100.0
    mid_pos[2] = cmd_pos_z

    end_pos[0] = cmd_pos_x + 100.0
    end_pos[1] = cmd_pos_y + 100.0
    end_pos[2] = cmd_pos_z

    end_ort[0] = cmd_ort_x
    end_ort[1] = cmd_ort_y
    end_ort[2] = cmd_ort_z

    spd = 300		   # [mm/s]
    acc = spd * 2.0    # [mm/s^2]
    jerk = 0.5		   # 0 ~ 1 사이 값
    NUM_OF_ROTATION = 5
    for iter in range(0, NUM_OF_ROTATION):
        print(f"test loop: {str(iter)}")
        for i in range(0, NUM_OF_ROTATION):
            err_ret_val, cmd_id = clink_rpc_api.clink_rpc_motion_command_robot_tcp_create(cbox_id)
            err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_robot_id_set(cbox_id, cmd_id, robot_id)
            err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_speed_start_set(cbox_id, cmd_id, 0)
            err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_speed_end_set(cbox_id, cmd_id, 0)
            err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_position_mid_set(cbox_id, cmd_id, mid_pos[0], mid_pos[1], mid_pos[2])
            err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_position_end_set(cbox_id, cmd_id, end_pos[0], end_pos[1], end_pos[2])
            err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_orientation_end_set(cbox_id, cmd_id, end_ort[0], end_ort[1], end_ort[2])
            err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_interpolator_set(cbox_id, cmd_id, clink_rpc_api.CLINK_INTERPOLATOR_CIRCLE)
            err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_coordinate_set(cbox_id, cmd_id, clink_rpc_api.CLINK_REF_COORDINATE_BASE)
            err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_orientation_mode_set(cbox_id, cmd_id, clink_rpc_api.CLINK_ORIENTATION_MODE_SLERP)
            err_ret_val = clink_rpc_api.clink_rpc_motion_command_velocity_profiler_set(cbox_id, cmd_id, clink_rpc_api.CLINK_VELOCITY_PROFILE_ASYMMETRIC_S_CURVE)
            err_ret_val = clink_rpc_api.clink_rpc_motion_command_acc_max_set(cbox_id, cmd_id, acc)
            err_ret_val = clink_rpc_api.clink_rpc_motion_command_dec_max_set(cbox_id, cmd_id, acc)
            err_ret_val = clink_rpc_api.clink_rpc_motion_command_speed_max_set(cbox_id, cmd_id, spd)
            if (0 == i):
                # 첫 모션은 끝 속도를 준다.
                err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_speed_start_set(cbox_id, cmd_id, 0)
                err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_speed_end_set(cbox_id, cmd_id, spd)
            elif ((NUM_OF_ROTATION - 1) == i):
                # 끝 모션은 시작 속도를 준다.
                err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_speed_start_set(cbox_id, cmd_id, spd)
                err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_speed_end_set(cbox_id, cmd_id, 0)
            else:
                # 그 외에는 시작, 끝 속도가 존재
                err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_speed_start_set(cbox_id, cmd_id, spd)
                err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_speed_end_set(cbox_id, cmd_id, spd)

            err_ret_val = clink_rpc_api.clink_rpc_motion_command_jerk_percentage_set(cbox_id, cmd_id, jerk)
            err_ret_val = clink_rpc_api.clink_rpc_motion_command_blending_motion_mode_set(cbox_id, cmd_id, clink_rpc_api.CLINK_SWITCH_OFF)
            err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_pose_type_set(cbox_id, cmd_id, clink_rpc_api.CLINK_POSE_TYPE_FLANGE)
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