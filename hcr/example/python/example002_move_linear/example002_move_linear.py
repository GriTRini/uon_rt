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
    cbox_id = example000_main.cbox_id
    robot_id = example000_main.robot_id
    err_ret_val = clink_rpc_api.clink_rpc_robot_servo_switch_set(cbox_id, robot_id, clink_rpc_api.CLINK_SWITCH_ON)

    # home 으로 이동
    example000_main.move_homing()
    time.sleep(1)

    err_ret_val, cmd_pos_x, cmd_pos_y, cmd_pos_z, cmd_ort_x, cmd_ort_y, cmd_ort_z = clink_rpc_api.clink_rpc_robot_tcp_pose_command_get(cbox_id, robot_id)

    NUM_OF_MOTIONS = 4
    target_pos = [[0 for _ in range(3)] for _ in range(NUM_OF_MOTIONS)]
    target_ort = [[0 for _ in range(3)] for _ in range(NUM_OF_MOTIONS)]

    target_pos[0][0] = cmd_pos_x + 100.0
    target_pos[0][1] = cmd_pos_y
    target_pos[0][2] = cmd_pos_z

    target_pos[1][0] = cmd_pos_x + 100.0
    target_pos[1][1] = cmd_pos_y + 100.0
    target_pos[1][2] = cmd_pos_z

    target_pos[2][0] = cmd_pos_x
    target_pos[2][1] = cmd_pos_y + 100.0
    target_pos[2][2] = cmd_pos_z

    target_pos[3][0] = cmd_pos_x
    target_pos[3][1] = cmd_pos_y
    target_pos[3][2] = cmd_pos_z

    target_ort[0][0] = cmd_ort_x
    target_ort[0][1] = cmd_ort_y
    target_ort[0][2] = cmd_ort_z

    target_ort[1][0] = cmd_ort_x
    target_ort[1][1] = cmd_ort_y
    target_ort[1][2] = cmd_ort_z

    target_ort[2][0] = cmd_ort_x
    target_ort[2][1] = cmd_ort_y
    target_ort[2][2] = cmd_ort_z

    target_ort[3][0] = cmd_ort_x
    target_ort[3][1] = cmd_ort_y
    target_ort[3][2] = cmd_ort_z

    spd = 300		   # [mm/s]
    acc = spd * 2.0    # [mm/s^2]
    jerk = 0.5		   # 0 ~ 1 사이 값

    for iter in range(0,10):
        print(f"test loop: {str(iter)}")
        for i in range(0,NUM_OF_MOTIONS):
            err_ret_val, cmd_id = clink_rpc_api.clink_rpc_motion_command_robot_tcp_create(cbox_id)
            err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_robot_id_set(cbox_id, cmd_id, robot_id)
            err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_speed_start_set(cbox_id, cmd_id, 0)
            err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_speed_end_set(cbox_id, cmd_id, 0)
            err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_position_end_set(cbox_id, cmd_id, target_pos[i][0], target_pos[i][1], target_pos[i][2])
            err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_orientation_end_set(cbox_id, cmd_id, target_ort[i][0], target_ort[i][1], target_ort[i][2])
            err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_interpolator_set(cbox_id, cmd_id, clink_rpc_api.CLINK_INTERPOLATOR_LINEAR)
            err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_coordinate_set(cbox_id, cmd_id, clink_rpc_api.CLINK_REF_COORDINATE_BASE)
            err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_orientation_mode_set(cbox_id, cmd_id, clink_rpc_api.CLINK_ORIENTATION_MODE_SLERP)
            err_ret_val = clink_rpc_api.clink_rpc_motion_command_velocity_profiler_set(cbox_id, cmd_id, clink_rpc_api.CLINK_VELOCITY_PROFILE_ASYMMETRIC_S_CURVE)
            err_ret_val = clink_rpc_api.clink_rpc_motion_command_acc_max_set(cbox_id, cmd_id, acc)
            err_ret_val = clink_rpc_api.clink_rpc_motion_command_dec_max_set(cbox_id, cmd_id, acc)
            err_ret_val = clink_rpc_api.clink_rpc_motion_command_speed_max_set(cbox_id, cmd_id, spd)
            err_ret_val = clink_rpc_api.clink_rpc_motion_command_jerk_percentage_set(cbox_id, cmd_id, jerk)
            err_ret_val = clink_rpc_api.clink_rpc_motion_command_blending_motion_mode_set(cbox_id, cmd_id, clink_rpc_api.CLINK_SWITCH_ON)
            err_ret_val = clink_rpc_api.clink_rpc_motion_command_radius_for_blending_set(cbox_id, cmd_id, 30)
            err_ret_val = clink_rpc_api.clink_rpc_motion_command_blending_auto_start_set(cbox_id, cmd_id, clink_rpc_api.CLINK_SWITCH_ON)
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