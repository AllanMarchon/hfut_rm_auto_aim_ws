#!/usr/bin/env python3

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _truthy(value):
    return value.strip().lower() in ("1", "true", "yes", "on")


def _default_sim_dir():
    env_dir = os.environ.get("HFUT_AUTO_AIM_SIM_DIR", "")
    if env_dir and (Path(env_dir) / "run_stationary_spin_target_test.sh").is_file():
        return env_dir

    candidates = []
    for base in (Path.cwd(), Path(__file__).resolve()):
        for parent in (base, *base.parents):
            candidates.append(parent / "hfut_auto_aim_sim-main")
            candidates.append(parent / "hfut_auto_aim_sim")

    for candidate in candidates:
        if (candidate / "run_stationary_spin_target_test.sh").is_file():
            return str(candidate)

    return str(Path.cwd() / "hfut_auto_aim_sim-main")


def _start_webots(context):
    if not _truthy(LaunchConfiguration("launch_webots").perform(context)):
        return []

    sim_dir = Path(LaunchConfiguration("sim_dir").perform(context)).expanduser()
    scenario = LaunchConfiguration("scenario").perform(context).strip().lower()
    script_name = (
        "run_moving_spin_target_test.sh"
        if scenario in ("moving", "move", "mobile")
        else "run_stationary_spin_target_test.sh"
    )
    script_path = sim_dir / script_name
    if not script_path.is_file():
        raise RuntimeError(f"Webots sim script not found: {script_path}")

    return [
        ExecuteProcess(
            cmd=["bash", str(script_path)],
            cwd=str(sim_dir),
            output="screen",
            emulate_tty=True,
            additional_env={
                "WEBOTS_IMAGE_TOPIC": LaunchConfiguration("image_topic"),
                "WEBOTS_CAMERA_INFO_TOPIC": LaunchConfiguration("camera_info_topic"),
                "WEBOTS_JOINT_STATES_TOPIC": LaunchConfiguration("joint_states_topic"),
                "WEBOTS_GIMBAL_CMD_TOPIC": LaunchConfiguration("cmd_topic"),
                "WEBOTS_SERIAL_MODE": LaunchConfiguration("vision_mode"),
                "WEBOTS_BULLET_SPEED": LaunchConfiguration("bullet_speed"),
                "WEBOTS_SET_MODE_SERVICES": "gimbal_pipeline/set_mode",
                "WEBOTS_CAMERA_FOLLOW_CMD_GIMBAL": "true",
                "WEBOTS_GIMBAL_USE_DIFF_COMMANDS": "false",
                "WEBOTS_IMAGE_ENCODING": "bgr8",
            },
        )
    ]


def generate_launch_description():
    declare_sim_dir = DeclareLaunchArgument(
        "sim_dir",
        default_value=_default_sim_dir(),
        description="Path to hfut_auto_aim_sim directory.",
    )
    declare_launch_webots = DeclareLaunchArgument(
        "launch_webots",
        default_value="true",
        description="Launch Webots through the sim repository script.",
    )
    declare_scenario = DeclareLaunchArgument(
        "scenario",
        default_value="stationary",
        description="stationary or moving target scenario.",
    )
    declare_image_topic = DeclareLaunchArgument("image_topic", default_value="/image_raw")
    declare_camera_info_topic = DeclareLaunchArgument(
        "camera_info_topic", default_value="/camera_info"
    )
    declare_joint_states_topic = DeclareLaunchArgument(
        "joint_states_topic", default_value="/joint_states"
    )
    declare_serial_topic = DeclareLaunchArgument(
        "serial_topic", default_value="/serial/receive"
    )
    declare_cmd_topic = DeclareLaunchArgument(
        "cmd_topic", default_value="/armor_solver/cmd_gimbal"
    )
    declare_vision_mode = DeclareLaunchArgument(
        "vision_mode",
        default_value="0",
        description="0 red auto-aim, 1 blue auto-aim.",
    )
    declare_bullet_speed = DeclareLaunchArgument(
        "bullet_speed",
        default_value="22.5",
        description="Simulated bullet speed in m/s.",
    )
    declare_respect_mode = DeclareLaunchArgument("respect_mode", default_value="true")
    declare_require_serial = DeclareLaunchArgument("require_serial", default_value="true")
    declare_enable_fire = DeclareLaunchArgument("enable_fire", default_value="false")
    declare_control_backend = DeclareLaunchArgument(
        "control_backend", default_value="aimer"
    )
    declare_detector_type = DeclareLaunchArgument("detector_type", default_value="yolo")
    declare_async_inference = DeclareLaunchArgument(
        "async_inference", default_value="false"
    )
    declare_enemy_color = DeclareLaunchArgument("enemy_color", default_value="")
    declare_derive_enemy_color = DeclareLaunchArgument(
        "derive_enemy_color_from_mode", default_value="true"
    )
    declare_debug_visualization = DeclareLaunchArgument(
        "debug_visualization", default_value="true"
    )

    sim_serial_bridge = Node(
        package="rm_bringup",
        executable="sim_serial_bridge",
        name="sim_serial_bridge",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "joint_states_topic": LaunchConfiguration("joint_states_topic"),
                "serial_topic": LaunchConfiguration("serial_topic"),
                "vision_mode": ParameterValue(
                    LaunchConfiguration("vision_mode"), value_type=int
                ),
                "bullet_speed": ParameterValue(
                    LaunchConfiguration("bullet_speed"), value_type=float
                ),
                "frame_id": "odom",
            }
        ],
    )

    aim_v2_node = Node(
        package="aim_v2",
        executable="aim_v2_node",
        name="gimbal_pipeline",
        output="both",
        emulate_tty=True,
        parameters=[
            {
                "respect_mode": ParameterValue(
                    LaunchConfiguration("respect_mode"), value_type=bool
                ),
                "require_serial": ParameterValue(
                    LaunchConfiguration("require_serial"), value_type=bool
                ),
                "enable_fire": ParameterValue(
                    LaunchConfiguration("enable_fire"), value_type=bool
                ),
                "control_backend": LaunchConfiguration("control_backend"),
                "enemy_color": LaunchConfiguration("enemy_color"),
                "derive_enemy_color_from_mode": ParameterValue(
                    LaunchConfiguration("derive_enemy_color_from_mode"),
                    value_type=bool,
                ),
                "async_inference": ParameterValue(
                    LaunchConfiguration("async_inference"), value_type=bool
                ),
                "detector_type": LaunchConfiguration("detector_type"),
                "debug_visualization": ParameterValue(
                    LaunchConfiguration("debug_visualization"), value_type=bool
                ),
                "default_bullet_speed": ParameterValue(
                    LaunchConfiguration("bullet_speed"), value_type=float
                ),
            }
        ],
        remappings=[
            ("image_raw", LaunchConfiguration("image_topic")),
            ("serial/receive", LaunchConfiguration("serial_topic")),
            ("cmd_gimbal", LaunchConfiguration("cmd_topic")),
        ],
    )

    return LaunchDescription(
        [
            declare_sim_dir,
            declare_launch_webots,
            declare_scenario,
            declare_image_topic,
            declare_camera_info_topic,
            declare_joint_states_topic,
            declare_serial_topic,
            declare_cmd_topic,
            declare_vision_mode,
            declare_bullet_speed,
            declare_respect_mode,
            declare_require_serial,
            declare_enable_fire,
            declare_control_backend,
            declare_detector_type,
            declare_async_inference,
            declare_enemy_color,
            declare_derive_enemy_color,
            declare_debug_visualization,
            OpaqueFunction(function=_start_webots),
            TimerAction(period=0.5, actions=[sim_serial_bridge]),
            TimerAction(period=1.0, actions=[aim_v2_node]),
        ]
    )
