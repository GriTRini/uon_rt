#ifndef __CLINK_API_CALL_TEST_SUPPORT_RPC_H__
#define __CLINK_API_CALL_TEST_SUPPORT_RPC_H__

#include "clink_def.h"
#include "clink_constant.h"


#ifdef __cplusplus
extern "C"
{
#endif

CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_ts_event_echo (
        uint32_t cbox_id,
        uint32_t event_grp,
        uint32_t event_subgrp,
        uint32_t event_sender_id,
        int32_t reserve01,
        int32_t reserve02,
        int32_t reserve03,
        int32_t reserve04,
        int32_t reserve05);

CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_ts_fault_inject(
        uint32_t cbox_id,
        uint32_t err_source, 
        uint32_t err_code, 
        uint32_t err_source_id);

CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_ts_cbox_io_digital_input_set(
        uint32_t cbox_id,
        uint32_t io_number,
        uint32_t io_value);

CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_ts_cbox_redundant_io_digital_input_set(
        uint32_t cbox_id,
        uint32_t io_number,
        uint32_t io_value);

CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_ts_cbox_redundant_io_digital_output_set(
        uint32_t cbox_id,
        uint32_t io_number,
        uint32_t io_value);

CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_ts_gen_cbox_io_digital_input_set(
        uint32_t cbox_id,
        uint32_t io_index,
        uint32_t io_number,
        uint32_t io_value);

CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_ts_gen_cbox_redundant_io_digital_input_set(
        uint32_t cbox_id,
        uint32_t io_index,
        uint32_t io_number,
        uint32_t io_value);

CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_ts_gen_cbox_redundant_io_digital_output_set(
        uint32_t cbox_id,
        uint32_t io_index,
        uint32_t io_number,
        uint32_t io_value);

CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_ts_safety_module_redundant_io_digital_input_set(
        uint32_t cbox_id,
        uint32_t io_number,
        uint32_t io_value);

CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_ts_robot_tool_io_digital_input_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t io_number,
        uint32_t io_value);

CLINK_EXPORT CLINK_API_RESULT 
    clink_rpc_ts_epc_io_digital_input_set(
        uint32_t cbox_id,
        uint32_t io_number,
        uint32_t io_value);

CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_ts_cbox_sdo_write(
        uint32_t cbox_id,
        uint32_t slave_id,
        uint32_t addr,
        uint32_t size,
        uint32_t sub_index,
        uint32_t value);

CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_ts_cbox_sdo_read(
        uint32_t cbox_id,
        uint32_t slave_id,
        uint32_t addr,
        uint32_t size,
        uint32_t sub_index,
        uint32_t* p_value);

CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_ts_robot_joint_velocity_feedforward_ratio_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t joint_index,
        clink_float_t* p_ratio);

CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_ts_robot_joint_velocity_feedforward_ratio_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t joint_index,
        clink_float_t ratio);

CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_ts_robot_joint_torque_feedforward_ratio_get(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t joint_index,
        clink_float_t* p_ratio);

CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_ts_robot_joint_torque_feedforward_ratio_set(
        uint32_t cbox_id,
        uint32_t robot_id,
        uint32_t joint_index,
        clink_float_t ratio);

CLINK_EXPORT CLINK_API_RESULT
    clink_rpc_ts_robot_joint_torque_feedforward_current_set(
        uint32_t cbox_id,
	    uint32_t robot_id,
	    uint32_t joint_index,
	    clink_float_t current);

#ifdef __cplusplus
}
#endif

#endif // __CLINK_API_CALL_TEST_SUPPORT_RPC_H__
