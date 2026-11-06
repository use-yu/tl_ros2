# TL SDK Python 绑定统一入口。
# 底层 _tl_host 为 nanobind 模块（linux-x86 / linux-arm64 / Windows 三平台统一，
# ARM64 交叉编译产物带 aarch64 平台标签），行为对齐原 linux-x86 SWIG 现状。
# __all__ 供静态分析器（Pylance/pyright）解析星号导入的符号名。
import os, sys
_d = os.path.dirname(os.path.abspath(__file__))
if _d not in sys.path: sys.path.insert(0, _d)
from _tl_host import *  # type: ignore  # noqa: F401,F403

__all__ = [
    "Alpha", "AdmittanceController", "AdmittanceParams", "CartesianParam", "CollisionPara", "CollisionSafeParam", "ConstForceStatus", "ConstantForceParams", "DISCONNECT", "EXCEPTION", "HanYu", "IOtype", "LogLevel",
    "ModbusMasterParameter", "ModbusRTUParameter", "ModbusTCPParameter", "MoveCmd", "OPERATION_NOT_ALLOWED", "PARAM_ERR", "RECEIVE_FAILED", "RemoteControl",
    "RemoteProgram", "Result", "RobotDHParam", "RobotJointParam", "RobotState", "SUCCESS", "Sensor6DData",
    "SensorBaseParam", "SixDimensionalForceCommunicationParams", "TIMEOUT", "ToolCoordinateRange", "ToolParam", "VectorDouble",
    "VectorInt", "VectorRemoteProgram", "VectorString", "VectorVectorString", "Coord", "PosType", "PosType_AP_TYPE", "PosType_E_TYPE", "PosType_GEType", "RobotType",
    "PosType_GPType", "PosType_PType", "PosType_RP_TYPE", "PosType_data", "calculate_user_coordinate", "calibration_oxy",
    "cancel_servo", "clear_error",
    "close_constforce", "close_servoJ", "close_servo_cst_mode", "close_servo_csv_mode", "close_servo_movej", "close_servo_movel", "connect_robot", "connect_robot_udp",
    "disconnect_robot", "drag_trajectory_delete", "drag_trajectory_export", "drag_trajectory_import", "drag_trajectory_playback",
    "get_analog_input", "get_analog_output", "get_cartesian_params",
    "get_collision", "get_collision_detection_param", "get_connection_status", "get_constforce_status", "get_current_coord",
    "get_current_mode", "get_current_position", "get_current_teach_sensitivity", "get_digital_input", "get_digital_output",
    "get_drag_thread_is_end", "get_four_point", "get_global_position", "get_global_sync_position", "get_global_variant",
    "get_io_type", "get_joint_position", "get_library_version", "get_local_pos_p_value", "get_origin_coord_to_target_coord", "get_quat2rpy",
    "get_r2tr", "get_remote_function", "get_robot_dh_param", "get_robot_joint_param", "get_robot_running_state",
    "get_robot_state", "get_robot_switch", "get_robot_type", "get_rpy2quat", "get_rpy2r", "get_sensor_6d_data",
    "get_sensor_base_param", "get_servo_state", "get_single_cycle", "get_six_dimensional_force_communication_params", "get_speed", "get_target_coord_pos_value",
    "get_teach_type", "get_tool_coordinate_range", "get_tool_hand_number", "get_tool_hand_param", "get_tr2r", "get_track_record_status",
    "get_user_coord_number", "get_user_coord_para", "job_create", "job_delete", "job_download_by_directory", "job_get_all_jobfile_name",
    "job_insert_moveC", "job_insert_moveJ", "job_insert_moveL", "job_open", "job_run", "job_upload_by_file",
    "log_download_by_quantity", "mark_base_sensor", "modbus_open_master", "modbus_read_coil_status", "modbus_read_holding_registers", "modbus_read_input_registers",
    "modbus_read_input_status", "modbus_set_master_parameter", "modbus_write_multiple_holding_registers", "modbus_write_signal_coil_status", "modbus_write_signal_holding_registers", "open_constforce",
    "open_servoJ", "open_servo_cst_mode", "open_servo_csv_mode", "open_servo_movej", "open_servo_movel", "query_joint_software_version", "queue_motion_clear_Data", "queue_motion_get_queuelen",
    "queue_motion_get_status", "queue_motion_push_back_moveC", "queue_motion_push_back_moveJ", "queue_motion_push_back_moveL", "queue_motion_push_back_moveS", "queue_motion_restart",
    "queue_motion_send_to_controller", "queue_motion_set_status", "queue_motion_size", "queue_motion_stop", "queue_motion_stop_not_power_off", "queue_motion_suspend",
    "recv_message", "restore_default_param_DH", "robot_go_home", "robot_go_to_reset_position", "robot_movej", "robot_movel", "robot_start_jogging", "robot_state_callback", "robot_stop_jogging",
    "servo_busy", "servo_movej", "servo_movel", "set_analog_output", "set_collision", "set_connect_timeout_seconds",
    "set_constforce_target_pose", "set_controller_ip", "set_current_coord", "set_current_mode", "set_current_teach_sensitivity", "set_darg_mode", "set_default_cartesian_params",
    "set_digital_output", "set_global_position", "set_receive_error_or_warnning_message_callback", "set_reconnect", "set_remote_function", "set_robot_dh_param", "set_robot_joint_param", "set_servoJ_pos", "set_servo_cst_torque", "set_servo_csv_velocity",
    "set_servo_poweroff", "set_servo_poweron", "set_servo_state", "set_six_dimensional_force_communication_params", "set_speed", "set_teach_type",
    "set_log_level", "set_tool_coordinate_range", "set_tool_hand_number", "set_tool_hand_param", "set_user_coord_number", "set_user_coordinate_data",
    "track_record_delete", "track_record_playback", "track_record_save", "track_record_start", "track_record_stop", "wait_servo",
    # ---- 新增 v24.xx 接口 ----
    "get_controller_id", "get_nexmotion_lib_version",
    "get_joint_temperature", "get_current_motor_current", "get_current_motor_torque",
    "get_pos_reachable", "get_current_line_speed_and_joint_speed",
    "job_insert_imove",
    # ---- 类与枚举值常量 ----
    "AlarmdIO", "BASE", "DragParam", "DragTorqueParam", "FIVE_AXIS_SERIAL",
    "FOUR_AXIS_PALLETIZING", "FOUR_AXIS_PALLETIZING_LEAD", "FOUR_AXIS_SCARA", "FOUR_AXIS_SERIAL", "JOINT",
    "PayloadParamBySensor", "RemoteProgramSetting", "SCARA_SPECIAL", "SEVEN_AXIS_SERIAL", "SINGLE_AXIS",
    "SIX_AXIS_COLLABORATIVE", "SIX_AXIS_SERIAL", "SafeIO", "THREE_AXIS_CARTESIAN", "THREE_AXIS_SCARA",
    "THREE_AXIS_SPECIAL", "TOOL", "TWO_AXIS_SCARA", "USER", "VectorAlarmdIO",
    "VectorRemoteProgramSetting",
    # ---- IO / 安全 ----
    "get_IO_reset_function", "set_IO_reset_function", "get_safe_IO_function", "set_safe_IO_function", "get_hard_enable_port",
    "set_hard_enable_port", "get_force_digital_input", "set_force_digital_input", "get_error_msg_of_digital_input", "set_error_msg_of_digital_input",
    "get_error_msg_of_digital_output", "set_error_msg_of_digital_output",
    # ---- 轴 / 伺服扩展 ----
    "get_axis_sdo", "get_axis_sdo_string", "set_axis_sdo", "set_axis_zero_position", "set_sync_axis_zero_position",
    "set_zero_pos_deviation", "get_joint_voltage", "get_current_encoded_pos", "get_current_extra_position", "get_current_positon_and_extra_position",
    "get_current_motor_speed", "get_current_motor_payload",
    # ---- 运动 / 坐标 ----
    "set_cartesian_params", "set_collision_para", "get_full_solution", "get_pos_trigger_interfer", "get_static_search_position",
    "four_point_calculation", "set_four_point_mark", "get_local_pos_e_value",
    # ---- 全局变量 ----
    "get_global_string", "set_global_string", "set_global_sync_position", "set_global_variant",
    # ---- Modbus / 末端 IO ----
    "modbus_get_master_connection_status", "modbus_write_multiple_coil_status", "set_terminal_io_485_param", "set_terminal_io_can_param", "terminal_io_read_modbus_data",
    "terminal_io_write_modbus_data",
    # ---- 作业控制 ----
    "job_break_point_run", "job_continue", "job_get_current_file", "job_get_current_file_csharp", "job_get_current_line",
    "job_insert_cil", "job_insert_end_if", "job_insert_end_until", "job_insert_end_while", "job_insert_if",
    "job_insert_io_out_command", "job_insert_jump", "job_insert_label", "job_insert_local_position", "job_insert_moveComm",
    "job_insert_samov_command", "job_insert_tasks", "job_insert_timer_command", "job_insert_until", "job_insert_wait_sync_task",
    "job_insert_while", "job_pause", "job_run_times", "job_step", "job_stop",
    "job_sync_job_file", "job_upload_by_directory",
    # ---- 队列运动 ----
    "queue_motion_push_back_cil", "queue_motion_push_back_end_if", "queue_motion_push_back_end_until", "queue_motion_push_back_end_while", "queue_motion_push_back_if",
    "queue_motion_push_back_imove", "queue_motion_push_back_jump", "queue_motion_push_back_label", "queue_motion_push_back_moveCA", "queue_motion_push_back_samov",
    "queue_motion_push_back_until", "queue_motion_push_back_while", "queue_motion_push_moveComm",
    # ---- 工具标定 ----
    "tool_hand_2_or_20_point_calibrate", "tool_hand_2_or_20_point_calibrate_caculate", "tool_hand_2_or_20_point_calibrate_clear", "tool_hand_7_point_calibrate", "tool_hand_7_point_calibrate_caculate",
    "tool_hand_7_point_calibrate_clear", "tool_hand_point_calibrate", "tool_hand_point_calibrate_caculate", "tool_hand_point_calibrate_clear",
    # ---- 手爪 / 灵巧手 ----
    "read_gripper_status", "write_gripper_continuous_force_control_grab", "write_gripper_force_control_grab", "write_gripper_move_to_position", "write_gripper_release",
    "write_gripper_stroke", "read_hand_action_sequence", "read_hand_gesture_sequence", "write_hand_angle_follow", "write_hand_force_threshold",
    "write_hand_freedom_angle", "write_hand_position_follow", "write_hand_speed",
    # ---- 远程 / 网络 ----
    "get_remote_param", "set_remote_param", "get_remote_program", "set_remote_program", "get_remote_status_tips",
    "set_remote_status_tips", "set_reconnect_callback", "get_controller_network_config", "set_controller_network_config", "get_controller_id_csharp",
    # ---- 系统 ----
    "backup_system", "restore_network_factory_settings", "send_message", "get_drag_param", "set_drag_param",
    "set_position_dragParams", "set_robots_parallel", "set_robot_switch", "get_robot_configuration", "get_payload_param_by_sensor",
    "get_sensor_data", "get_sensor_joint_thresholds", "get_teachbox_connection_status", "get_tool_trigger_interfer", "set_result_for_DH",
]
