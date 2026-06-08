#!/usr/bin/env python3

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    bringup_share = get_package_share_directory("rm_bringup")
    config_root = os.path.join(bringup_share, "config")
    launch_params_path = os.path.join(config_root, "launch_params_decoupled.yaml")

    try:
        with open(launch_params_path, "r", encoding="utf-8") as f:
            launch_params = yaml.safe_load(f) or {}
    except Exception:
        launch_params = {
            "robot": "uav",
            "image_source": "video",
            "virtual_serial": True,
            "namespace": "",
            "aim_v2": {},
        }

    robot_name = str(launch_params.get("robot", "uav")).strip() or "uav"
    aim_v2_params = launch_params.get("aim_v2", {}) or {}

    def launch_default(name, default):
        value = aim_v2_params.get(name, default)
        if isinstance(value, bool):
            return str(value).lower()
        return str(value)

    def get_robot_params(name):
        tuning_path = os.path.join(config_root, f"{name}_params.yaml")
        if os.path.isfile(tuning_path):
            return tuning_path

        path = os.path.join(config_root, robot_name, f"{name}_params.yaml")
        if os.path.isfile(path):
            return path
        return os.path.join(config_root, "uav", f"{name}_params.yaml")

    declare_image_source = DeclareLaunchArgument(
        "image_source",
        default_value=str(launch_params.get("image_source", "video")),
        description="图像输入源：video | mindvision | hik",
    )
    declare_virtual_serial = DeclareLaunchArgument(
        "virtual_serial",
        default_value=str(launch_params.get("virtual_serial", True)).lower(),
        description="是否使用虚拟下位机替代真实串口",
    )
    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value=str(launch_params.get("namespace", "")),
        description="所有节点和话题使用的 ROS 命名空间",
    )
    declare_respect_mode = DeclareLaunchArgument(
        "respect_mode",
        default_value=launch_default("respect_mode", True),
        description="是否只在自瞄模式下输出云台指令",
    )
    declare_require_serial = DeclareLaunchArgument(
        "require_serial",
        default_value=launch_default("require_serial", True),
        description="是否要求收到串口反馈后才开始解算",
    )
    declare_enable_fire = DeclareLaunchArgument(
        "enable_fire",
        default_value=launch_default("enable_fire", False),
        description="是否允许 aim_v2 输出 fire_advice",
    )
    declare_control_backend = DeclareLaunchArgument(
        "control_backend",
        default_value=launch_default("control_backend", "planner"),
        description="aim_v2 控制后端：planner | aimer",
    )
    declare_enemy_color = DeclareLaunchArgument(
        "enemy_color",
        default_value=launch_default("enemy_color", ""),
        description="强制指定敌方颜色：red | blue；空字符串表示使用 aim_v2 默认逻辑",
    )
    declare_derive_enemy_color_from_mode = DeclareLaunchArgument(
        "derive_enemy_color_from_mode",
        default_value=launch_default("derive_enemy_color_from_mode", True),
        description="是否根据串口 vision_mode 0/1 自动切换敌方颜色",
    )
    declare_async_inference = DeclareLaunchArgument(
        "async_inference",
        default_value=launch_default("async_inference", True),
        description="是否启用 SP OpenVINO 异步推理链路",
    )
    declare_detector_type = DeclareLaunchArgument(
        "detector_type",
        default_value=launch_default("detector_type", "yolo"),
        description="aim_v2 检测器类型：yolo | traditional",
    )

    declare_debug_visualization = DeclareLaunchArgument(
        "debug_visualization",
        default_value=launch_default("debug_visualization", True),
        description="Publish aim_v2 debug image and MarkerArray topics for Foxglove/RViz",
    )

    def create_camera_node(context):
        image_source = LaunchConfiguration("image_source").perform(context).lower()
        if image_source == "video":
            return [
                Node(
                    package="video_player",
                    executable="video_player_node",
                    name="video_player",
                    output="both",
                    emulate_tty=True,
                    parameters=[get_robot_params("video_player")],
                )
            ]
        if image_source == "mindvision":
            return [
                Node(
                    package="mindvision_camera",
                    executable="mindvision_camera_node",
                    name="mv_camera",
                    output="both",
                    emulate_tty=True,
                    parameters=[get_robot_params("mindvision_camera_driver")],
                )
            ]
        if image_source == "hik":
            return [
                Node(
                    package="ros2_hik_camera",
                    executable="ros2_hik_camera_node",
                    name="hik_camera",
                    output="both",
                    emulate_tty=True,
                    parameters=[get_robot_params("hik_camera_driver")],
                    additional_env={
                        "MVCAM_SDK_PATH": "/opt/MVS",
                        "MVCAM_COMMON_RUNENV": "/opt/MVS/lib",
                    },
                )
            ]
        return [
            Node(
                package="mindvision_camera",
                executable="mindvision_camera_node",
                name="camera_driver",
                output="both",
                emulate_tty=True,
                parameters=[get_robot_params("mindvision_camera_driver")],
            )
        ]

    def create_serial_node(context):
        virtual_serial = LaunchConfiguration("virtual_serial").perform(context).lower() == "true"
        if virtual_serial:
            return [
                Node(
                    package="rm_serial_driver",
                    executable="virtual_serial_node",
                    name="virtual_serial",
                    output="both",
                    emulate_tty=True,
                    parameters=[get_robot_params("virtual_serial")],
                )
            ]
        return [
            Node(
                package="rm_serial_driver",
                executable="rm_serial_driver_node",
                name="serial_driver",
                output="both",
                emulate_tty=True,
                parameters=[get_robot_params("serial_driver")],
            )
        ]

    aim_v2_node = Node(
        package="aim_v2",
        executable="aim_v2_node",
        name="gimbal_pipeline",
        output="both",
        emulate_tty=True,
        parameters=[
            {
                "respect_mode": LaunchConfiguration("respect_mode"),
                "require_serial": LaunchConfiguration("require_serial"),
                "enable_fire": LaunchConfiguration("enable_fire"),
                "control_backend": LaunchConfiguration("control_backend"),
                "enemy_color": LaunchConfiguration("enemy_color"),
                "derive_enemy_color_from_mode": LaunchConfiguration(
                    "derive_enemy_color_from_mode"
                ),
                "async_inference": LaunchConfiguration("async_inference"),
                "detector_type": LaunchConfiguration("detector_type"),
                "debug_visualization": LaunchConfiguration("debug_visualization"),
            }
        ],
        remappings=[
            ("image_raw", "/image_raw"),
            ("serial/receive", "/serial/receive"),
            ("cmd_gimbal", "/armor_solver/cmd_gimbal"),
        ],
    )

    return LaunchDescription(
        [
            declare_image_source,
            declare_virtual_serial,
            declare_namespace,
            declare_respect_mode,
            declare_require_serial,
            declare_enable_fire,
            declare_control_backend,
            declare_enemy_color,
            declare_derive_enemy_color_from_mode,
            declare_async_inference,
            declare_detector_type,
            declare_debug_visualization,
            PushRosNamespace(LaunchConfiguration("namespace")),
            TimerAction(period=1.0, actions=[OpaqueFunction(function=create_serial_node)]),
            TimerAction(period=1.5, actions=[OpaqueFunction(function=create_camera_node)]),
            TimerAction(period=2.0, actions=[aim_v2_node]),
        ]
    )
