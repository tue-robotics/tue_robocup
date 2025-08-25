from hsrb_interface import settings

_settings = settings._SETTINGS
_trajectory = _settings["trajectory"]
_trajectory["impedance_control"] = "/hero/impedance_control"
_trajectory["constraint_filter_service"] = "/hero/trajectory_filter/filter_trajectory_with_constraints"
_trajectory["timeopt_filter_service"] = "/hero/omni_base_timeopt_filter"
_trajectory["whole_timeopt_filter_service"] = "/hero/filter_hsrb_trajectory"
del _trajectory

_joint_group = _settings["joint_group"]
_whole_body = _joint_group["whole_body"]
_whole_body["joint_states_topic"] = "/hero/joint_states"
_whole_body["arm_controller_prefix"] = "/hero/arm_trajectory_controller"
_whole_body["head_controller_prefix"] = "/hero/head_trajectory_controller"
_whole_body["hand_controller_prefix"] = "/hero/gripper_controller"
_whole_body["omni_base_controller_prefix"] = "/hero/omni_base_controller"
_whole_body["plan_with_constraints_service"] = "/hero/plan_with_constraints"
_whole_body["plan_with_hand_goals_service"] = "/hero/plan_with_hand_goals"
_whole_body["plan_with_hand_line_service"] = "/hero/plan_with_hand_line"
_whole_body["plan_with_joint_goals_service"] = "/hero/plan_with_joint_goals"
del _whole_body
del _joint_group

_end_effector = _settings["end_effector"]
_gripper = _end_effector["gripper"]
_gripper["prefix"] = "/hero/gripper_controller"
del _gripper

_suction = _end_effector["suction"]
_suction["action"] = "/hero/suction_control"
_suction["suction_topic"] = "/hero/command_suction"
_suction["pressure_sensor_topic"] = "/hero/pressure_sensor"
del _suction
del _end_effector

_mobile_base = _settings["mobile_base"]
_omni_base = _mobile_base["omni_base"]
_omni_base["move_base_action"] = "/hero/move_base/move"  # Not available on HERO
_omni_base["follow_trajectory_action"] = "/hero/omni_base_controller"
_omni_base["pose_topic"] = "/hero/global_pose"  # Not available on HERO
_omni_base["goal_topic"] = "/hero/base_goal"  # Not available on HERO
del _omni_base
del _mobile_base

_camera = _settings["camera"]
_head_l_stereo_camera = _camera["head_l_stereo_camera"]
_head_l_stereo_camera["prefix"] = "/hero/head_l_stereo_camera"
del _head_l_stereo_camera

_head_r_stereo_camera = _camera["head_r_stereo_camera"]
_head_r_stereo_camera["prefix"] = "/hero/head_r_stereo_camera"
del _head_r_stereo_camera

_head_rgbd_sensor_rgb = _camera["head_rgbd_sensor_rgb"]
_head_rgbd_sensor_rgb["prefix"] = "/hero/head_rgbd_sensor/rgb"
del _head_rgbd_sensor_rgb

_head_rgbd_sensor_depth = _camera["head_rgbd_sensor_depth"]
_head_rgbd_sensor_depth["prefix"] = "/hero/head_rgbd_sensor/depth_registered"
del _head_rgbd_sensor_depth
del _camera

_imu = _settings["imu"]
_base_imu = _imu["base_imu"]
_base_imu["topic"] = "/hero/base_imu/data"
del _base_imu
del _imu

_force_torque = _settings["force_torque"]
_wrist_wrench = _force_torque["wrist_wrench"]
_wrist_wrench["raw_topic"] = "/hero/wrist_wrench/raw"
_wrist_wrench["compensated_topic"] = "/hero/wrist_wrench/compensated"
_wrist_wrench["reset_service"] = "/hero/wrist_wrench/readjust_offset"
del _wrist_wrench
del _force_torque

_lidar = _settings["lidar"]
_base_scan = _lidar["base_scan"]
_base_scan["topic"] = "/hero/base_laser/scan"  # Replaced by custom topic
del _base_scan
del _lidar

_object_detection = _settings["object_detection"]
_marker = _object_detection["marker"]
_marker["topic"] = "/hero/recognized_object"
del _marker
del _object_detection

_power_supply = _settings["power_supply"]
_battery = _power_supply["battery"]
_battery["topic"] = "/hero/battery_state"
del _battery
del _power_supply

# Still in global namespace
# _text_to_speech = _settings["text_to_speech"]
# _default_tts = _text_to_speech["default_tts"]
# _default_tts["topic"] = "talk_request"
# del _default_tts
# del _text_to_speech

# Not running
# _collision_world = _settings["collision_world"]
# _global_collision_world = _collision_world["global_collision_world"]
# _global_collision_world["service"] = "/hero/get_collision_enviroment"
# _global_collision_world["cotrol_topic"] = "/hero/known_object"
# _global_collision_world["listening_topic"] = "/hero/known_object_ids"
# del _global_collision_world
# del _collision_world

del _settings

from .hero import Hero
