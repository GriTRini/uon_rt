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
    # 용접향 weaving 모션에 대한 설명이다.
    # 기본적으로 tcp motion 과 동일하게 호출하며
    # weaving과 관련한 추가적인 옵션이 붙는다.
    cbox_id = example000_main.cbox_id
    robot_id = example000_main.robot_id
    err_ret_val = clink_rpc_api.clink_rpc_robot_servo_switch_set(cbox_id, robot_id, clink_rpc_api.CLINK_SWITCH_ON)

    # home 으로 이동
    example000_main.move_homing()
    time.sleep(1)

    err_ret_val, cmd_pos_x, cmd_pos_y, cmd_pos_z, cmd_ort_x, cmd_ort_y, cmd_ort_z = clink_rpc_api.clink_rpc_robot_tcp_pose_command_get(cbox_id, robot_id)

    cmd_id = 0
    spd = 10
    acc = spd * 2
    jerk = 0.5

    # weaving motion create
    err_ret_val, cmd_id = clink_rpc_api.clink_rpc_motion_command_robot_tcp_welding_create(cbox_id)

    # tcp motion 설정과 기본적으로 동일
    err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_robot_id_set(cbox_id, cmd_id, robot_id)
    err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_speed_start_set(cbox_id, cmd_id, 0)
    err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_speed_end_set(cbox_id, cmd_id, 0)
    err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_position_end_set(cbox_id, cmd_id, cmd_pos_x, cmd_pos_y + 203, cmd_pos_z)
    err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_orientation_end_set(cbox_id, cmd_id, cmd_ort_x, cmd_ort_y, cmd_ort_z)
    err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_interpolator_set(cbox_id, cmd_id, clink_rpc_api.CLINK_INTERPOLATOR_LINEAR)
    err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_coordinate_set(cbox_id, cmd_id, clink_rpc_api.CLINK_REF_COORDINATE_BASE)
    err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_orientation_mode_set(cbox_id, cmd_id, clink_rpc_api.CLINK_ORIENTATION_MODE_AXIS)
    err_ret_val = clink_rpc_api.clink_rpc_motion_command_velocity_profiler_set(cbox_id, cmd_id, clink_rpc_api.CLINK_VELOCITY_PROFILE_ASYMMETRIC_S_CURVE)
    err_ret_val = clink_rpc_api.clink_rpc_motion_command_acc_max_set(cbox_id, cmd_id, acc)
    err_ret_val = clink_rpc_api.clink_rpc_motion_command_dec_max_set(cbox_id, cmd_id, acc)
    err_ret_val = clink_rpc_api.clink_rpc_motion_command_speed_max_set(cbox_id, cmd_id, spd)
    err_ret_val = clink_rpc_api.clink_rpc_motion_command_jerk_percentage_set(cbox_id, cmd_id, jerk)
    err_ret_val = clink_rpc_api.clink_rpc_motion_command_blending_motion_mode_set(cbox_id, cmd_id, clink_rpc_api.CLINK_SWITCH_ON)
    err_ret_val = clink_rpc_api.clink_rpc_motion_command_radius_for_blending_set(cbox_id, cmd_id, 50)
    err_ret_val = clink_rpc_api.clink_rpc_motion_command_blending_auto_start_set(cbox_id, cmd_id, clink_rpc_api.CLINK_SWITCH_ON)
    err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_pose_type_set(cbox_id, cmd_id, clink_rpc_api.CLINK_POSE_TYPE_FLANGE)

    # weaving 과 관련된 추가 파라미터
    # weaving motion enable
    err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_welding_weaving_motion_switch_set(cbox_id, cmd_id, clink_rpc_api.CLINK_SWITCH_ON)
    # 주파수 설정
    err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_welding_frequency_set(cbox_id, cmd_id, 1.0)
    # 좌측 진폭 설정
    err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_welding_left_amplitude_set(cbox_id, cmd_id, 4.0)
    # 우측 진폭 설정
    err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_welding_right_amplitude_set(cbox_id, cmd_id, 4.0)
    # 위빙 패턴 설정(zigzag, ellipse, ....)
    err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_welding_weaving_pattern_set(cbox_id, cmd_id, clink_rpc_api.CLINK_WELDING_WEAVING_PATTERN_ZIGZAG)
    # zigzag의 경우에는 R, U, F 비율을 설정한다.
    err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_welding_zigzag_ratio_set(cbox_id, cmd_id, 0.5, 0.0, 0.5, 0.0)

    err_ret_val = clink_rpc_api.clink_rpc_motion_command_queue_add(cbox_id, cmd_id)
    err_ret_val = clink_rpc_api.clink_rpc_motion_command_queue_flush(cbox_id)
    time.sleep(3)

    # 진행 도중 주파수를 변경한다.
    err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_welding_frequency_change(cbox_id, cmd_id, 2.0)
    time.sleep(3)

    # 진행 도중 진폭을 븐경한다.
    err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_welding_left_amplitude_change(cbox_id, cmd_id, 2.0)
    err_ret_val = clink_rpc_api.clink_rpc_motion_command_robot_tcp_welding_right_amplitude_change(cbox_id, cmd_id, 2.0)
    err_ret_val = clink_rpc_api.clink_rpc_motion_command_destroy(cbox_id, cmd_id)

    err_ret_val, valid_event = clink_rpc_api.clink_rpc_system_wait_event_group_subgroup_sender_id(
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
