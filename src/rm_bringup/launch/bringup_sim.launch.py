#!/usr/bin/env python3

import os
from pathlib import Path

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _truthy(value):
    return value.strip().lower() in ("1", "true", "yes", "on")


def _launch_value(context, name):
    return LaunchConfiguration(name).perform(context)


def _launch_float(context, name):
    return float(_launch_value(context, name))


def _launch_bool(context, name):
    return _truthy(_launch_value(context, name))


def _float_list(value, expected_size, name):
    tokens = value.replace("[", " ").replace("]", " ").replace(",", " ").split()
    values = [float(token) for token in tokens]
    if len(values) != expected_size:
        raise RuntimeError(f"{name} expects {expected_size} values, got {len(values)}")
    return values


def _format_launch_default(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (list, tuple)):
        return " ".join(str(item) for item in value)
    return str(value)


def _default_sim_params_path():
    return Path(__file__).resolve().parent.parent / "config" / "sim_params.yaml"


def _load_sim_defaults():
    path = _default_sim_params_path()
    with path.open("r", encoding="utf-8") as config_file:
        return yaml.safe_load(config_file) or {}


def _sim_default(defaults, section, key, fallback):
    value = defaults.get(section, {}).get(key, fallback)
    return _format_launch_default(value)


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
                "WEBOTS_CAMERA_WIDTH": LaunchConfiguration("camera_width"),
                "WEBOTS_CAMERA_HEIGHT": LaunchConfiguration("camera_height"),
                "WEBOTS_CAMERA_FX": LaunchConfiguration("camera_fx"),
                "WEBOTS_CAMERA_FY": LaunchConfiguration("camera_fy"),
                "WEBOTS_CAMERA_CX": LaunchConfiguration("camera_cx"),
                "WEBOTS_CAMERA_CY": LaunchConfiguration("camera_cy"),
                "WEBOTS_CAMERA_DISTORTION_MODEL": "plumb_bob",
                "WEBOTS_CAMERA_D": LaunchConfiguration("distort_coeffs"),
                "WEBOTS_JOINT_STATES_TOPIC": LaunchConfiguration("joint_states_topic"),
                "WEBOTS_GIMBAL_CMD_TOPIC": LaunchConfiguration("cmd_topic"),
                "WEBOTS_SERIAL_MODE": LaunchConfiguration("vision_mode"),
                "WEBOTS_BULLET_SPEED": LaunchConfiguration("bullet_speed"),
                "WEBOTS_SET_MODE_SERVICES": "gimbal_pipeline/set_mode",
                "WEBOTS_CAMERA_FOLLOW_CMD_GIMBAL": "true",
                "WEBOTS_GIMBAL_USE_DIFF_COMMANDS": "false",
                "WEBOTS_GIMBAL_WEBOTS_YAW_SIGN": LaunchConfiguration("webots_yaw_sign"),
                "WEBOTS_GIMBAL_WEBOTS_PITCH_SIGN": LaunchConfiguration("webots_pitch_sign"),
                "WEBOTS_IMAGE_ENCODING": "bgr8",
            },
        )
    ]


def _start_aim_v2(context):
    fx = _launch_float(context, "camera_fx")
    fy = _launch_float(context, "camera_fy")
    cx = _launch_float(context, "camera_cx")
    cy = _launch_float(context, "camera_cy")
    camera_matrix = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    distort_coeffs = _float_list(_launch_value(context, "distort_coeffs"), 5, "distort_coeffs")
    r_camera2gimbal = _float_list(
        _launch_value(context, "R_camera2gimbal"), 9, "R_camera2gimbal"
    )
    t_camera2gimbal = _float_list(
        _launch_value(context, "t_camera2gimbal"), 3, "t_camera2gimbal"
    )

    return [
        Node(
            package="aim_v2",
            executable="aim_v2_node",
            name="gimbal_pipeline",
            output="both",
            emulate_tty=True,
            parameters=[
                {
                    "respect_mode": _launch_bool(context, "respect_mode"),
                    "require_serial": _launch_bool(context, "require_serial"),
                    "enable_fire": _launch_bool(context, "enable_fire"),
                    "control_backend": _launch_value(context, "control_backend"),
                    "enemy_color": _launch_value(context, "enemy_color"),
                    "derive_enemy_color_from_mode": _launch_bool(
                        context, "derive_enemy_color_from_mode"
                    ),
                    "async_inference": _launch_bool(context, "async_inference"),
                    "detector_type": _launch_value(context, "detector_type"),
                    "debug_visualization": _launch_bool(context, "debug_visualization"),
                    "default_bullet_speed": _launch_float(context, "bullet_speed"),
                    "camera_matrix": camera_matrix,
                    "distort_coeffs": distort_coeffs,
                    "R_camera2gimbal": r_camera2gimbal,
                    "t_camera2gimbal": t_camera2gimbal,
                    "yaw_offset": _launch_float(context, "yaw_offset"),
                    "pitch_offset": _launch_float(context, "pitch_offset"),
                }
            ],
            remappings=[
                ("image_raw", _launch_value(context, "image_topic")),
                ("serial/receive", _launch_value(context, "serial_topic")),
                ("cmd_gimbal", _launch_value(context, "cmd_topic")),
            ],
        )
    ]


def generate_launch_description():
    sim_defaults = _load_sim_defaults()

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
    declare_camera_width = DeclareLaunchArgument(
        "camera_width", default_value=_sim_default(sim_defaults, "camera", "width", 1440)
    )
    declare_camera_height = DeclareLaunchArgument(
        "camera_height", default_value=_sim_default(sim_defaults, "camera", "height", 1080)
    )
    declare_camera_fx = DeclareLaunchArgument(
        "camera_fx", default_value=_sim_default(sim_defaults, "camera", "fx", 1739.130435)
    )
    declare_camera_fy = DeclareLaunchArgument(
        "camera_fy", default_value=_sim_default(sim_defaults, "camera", "fy", 1739.130435)
    )
    declare_camera_cx = DeclareLaunchArgument(
        "camera_cx", default_value=_sim_default(sim_defaults, "camera", "cx", 719.5)
    )
    declare_camera_cy = DeclareLaunchArgument(
        "camera_cy", default_value=_sim_default(sim_defaults, "camera", "cy", 539.5)
    )
    declare_distort_coeffs = DeclareLaunchArgument(
        "distort_coeffs",
        default_value=_sim_default(
            sim_defaults, "camera", "distort_coeffs", [0, 0, 0, 0, 0]
        ),
    )
    declare_r_camera2gimbal = DeclareLaunchArgument(
        "R_camera2gimbal",
        default_value=_sim_default(
            sim_defaults, "extrinsics", "R_camera2gimbal", [0, 0, 1, -1, 0, 0, 0, -1, 0]
        ),
    )
    declare_t_camera2gimbal = DeclareLaunchArgument(
        "t_camera2gimbal",
        default_value=_sim_default(sim_defaults, "extrinsics", "t_camera2gimbal", [0, 0, 0]),
    )
    declare_yaw_offset = DeclareLaunchArgument(
        "yaw_offset",
        default_value=_sim_default(sim_defaults, "extrinsics", "yaw_offset", 0.0),
    )
    declare_pitch_offset = DeclareLaunchArgument(
        "pitch_offset",
        default_value=_sim_default(sim_defaults, "extrinsics", "pitch_offset", 0.0),
    )
    declare_webots_yaw_sign = DeclareLaunchArgument(
        "webots_yaw_sign",
        default_value=_sim_default(sim_defaults, "webots", "yaw_sign", 1.0),
        description="Webots yaw motion sign; flip to -1.0 if yaw moves away from target.",
    )
    declare_webots_pitch_sign = DeclareLaunchArgument(
        "webots_pitch_sign",
        default_value=_sim_default(sim_defaults, "webots", "pitch_sign", -1.0),
        description="Webots pitch motion sign; flip to 1.0 if pitch moves away from target.",
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
        default_value=_sim_default(sim_defaults, "serial", "vision_mode", 0),
        description="0 red auto-aim, 1 blue auto-aim.",
    )
    declare_bullet_speed = DeclareLaunchArgument(
        "bullet_speed",
        default_value=_sim_default(sim_defaults, "serial", "bullet_speed", 22.5),
        description="Simulated bullet speed in m/s.",
    )
    declare_respect_mode = DeclareLaunchArgument("respect_mode", default_value="true")
    declare_require_serial = DeclareLaunchArgument("require_serial", default_value="true")
    declare_enable_fire = DeclareLaunchArgument("enable_fire", default_value="false")
    declare_control_backend = DeclareLaunchArgument(
        "control_backend", default_value="aimer"
    )
    declare_detector_type = DeclareLaunchArgument(
        "detector_type", default_value="traditional"
    )
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
    declare_roll = DeclareLaunchArgument("roll", default_value="0.0")
    declare_yaw = DeclareLaunchArgument("yaw", default_value="0.0")
    declare_pitch = DeclareLaunchArgument("pitch", default_value="0.0")

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
                "roll": ParameterValue(LaunchConfiguration("roll"), value_type=float),
                "yaw": ParameterValue(LaunchConfiguration("yaw"), value_type=float),
                "pitch": ParameterValue(LaunchConfiguration("pitch"), value_type=float),
                "frame_id": "odom",
            }
        ],
    )

    return LaunchDescription(
        [
            declare_sim_dir,
            declare_launch_webots,
            declare_scenario,
            declare_image_topic,
            declare_camera_info_topic,
            declare_camera_width,
            declare_camera_height,
            declare_camera_fx,
            declare_camera_fy,
            declare_camera_cx,
            declare_camera_cy,
            declare_distort_coeffs,
            declare_r_camera2gimbal,
            declare_t_camera2gimbal,
            declare_yaw_offset,
            declare_pitch_offset,
            declare_webots_yaw_sign,
            declare_webots_pitch_sign,
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
            declare_roll,
            declare_yaw,
            declare_pitch,
            OpaqueFunction(function=_start_webots),
            TimerAction(period=0.5, actions=[sim_serial_bridge]),
            TimerAction(period=1.0, actions=[OpaqueFunction(function=_start_aim_v2)]),
        ]
    )
