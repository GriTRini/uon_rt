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

    NUM_OF_MOTIONS = 4
    NUM_OF_JOINTS = 6

    angles = [
        [0.0, -90.0, -90.0, -90.0, 90.0, 0.0],
        [20.0, -100.0, -80.0, -90.0, 90.0, -70.0],
        [0.0, -30.0, -120.0, 40.0, 20.0, 70.0],
        [-20.0, -130.0, -50.0, -50.0, 90.0, -70.0]
    ]
    spd = 50			# [deg/s]
    acc = spd * 2.0	    # [deg/s^2]
    jerk = 0.5			# 0 ~ 1 사이 값

    for iter in range(0, 10):
        cmd_id = 0
        print(f"test loop: {iter}")
        for i in range(0, NUM_OF_MOTIONS):
            err_ret_val, cmd_id = clink_rpc_api.clink_rpc_motion_command_robot_joint_create(cbox_id)
            clink_rpc_api.clink_rpc_motion_command_robot_robot_id_set(cbox_id, cmd_id, robot_id)
            for j_idx in range(0, NUM_OF_JOINTS):
                clink_rpc_api.clink_rpc_motion_command_robot_joint_angle_end_set(cbox_id, cmd_id, j_idx, angles[i][j_idx])
            clink_rpc_api.clink_rpc_motion_command_velocity_profiler_set(cbox_id, cmd_id, clink_rpc_api.CLINK_VELOCITY_PROFILE_ASYMMETRIC_S_CURVE)
            clink_rpc_api.clink_rpc_motion_command_acc_max_set(cbox_id, cmd_id, acc)
            clink_rpc_api.clink_rpc_motion_command_dec_max_set(cbox_id, cmd_id, acc)
            clink_rpc_api.clink_rpc_motion_command_speed_max_set(cbox_id, cmd_id, spd)
            clink_rpc_api.clink_rpc_motion_command_jerk_percentage_set(cbox_id, cmd_id, jerk)
            clink_rpc_api.clink_rpc_motion_command_blending_motion_mode_set(cbox_id, cmd_id, clink_rpc_api.CLINK_SWITCH_OFF)
            err_ret_val = clink_rpc_api.clink_rpc_motion_command_queue_add(cbox_id, cmd_id)		# 모션을 queue 에 삽입한다.
            err_ret_val = clink_rpc_api.clink_rpc_motion_command_queue_flush(cbox_id)	        # queue 에 삽입되어 있는 모션들을 일괄적으로 전송한다(모션 시작).
            clink_rpc_api.clink_rpc_motion_command_destroy(cbox_id, cmd_id)

        err_ret_val, valid_event = clink_rpc_api.clink_rpc_system_wait_event_group_subgroup_sender_id(
            cbox_id,
            clink_rpc_api.CLINK_EVENT_GRP_MOTION_COMMAND,
            clink_rpc_api.CLINK_EVENT_SUBGRP_MOTION_COMMAND_SETTLED_DOWN,
            cmd_id,
            1000000,
            1)

    example000_main.move_homing()
    clink_rpc_api.clink_rpc_robot_servo_switch_set(cbox_id, robot_id, clink_rpc_api.CLINK_SWITCH_OFF)
    time.sleep(1)

if __name__ == "__main__":
    result = example000_main.setup()
    if result:
        main()