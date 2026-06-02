#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    image_topic_arg = DeclareLaunchArgument(
        "image_topic", default_value="/image_raw", description="Input image topic"
    )
    serial_topic_arg = DeclareLaunchArgument(
        "serial_topic", default_value="/serial/receive", description="Serial receive topic"
    )
    cmd_topic_arg = DeclareLaunchArgument(
        "cmd_topic",
        default_value="/armor_solver/cmd_gimbal",
        description="HFUT serial driver gimbal command topic",
    )
    respect_mode_arg = DeclareLaunchArgument(
        "respect_mode", default_value="true", description="Only aim in auto aim modes"
    )
    enable_fire_arg = DeclareLaunchArgument(
        "enable_fire", default_value="false", description="Allow fire_advice output"
    )
    control_backend_arg = DeclareLaunchArgument(
        "control_backend",
        default_value="planner",
        description="SP control backend: planner | aimer",
    )
    enemy_color_arg = DeclareLaunchArgument(
        "enemy_color",
        default_value="",
        description="Initial enemy color override: red | blue. Empty uses config.",
    )
    derive_enemy_color_from_mode_arg = DeclareLaunchArgument(
        "derive_enemy_color_from_mode",
        default_value="true",
        description="Switch enemy color from serial vision mode 0/1",
    )
    async_inference_arg = DeclareLaunchArgument(
        "async_inference",
        default_value="true",
        description="Use SP OpenVINO start_async detector pipeline when available",
    )
    detector_type_arg = DeclareLaunchArgument(
        "detector_type", default_value="yolo", description="yolo or traditional"
    )

    aim_node = Node(
        package="aim_v2",
        executable="aim_v2_node",
        name="gimbal_pipeline",
        output="both",
        emulate_tty=True,
        parameters=[
            {
                "respect_mode": LaunchConfiguration("respect_mode"),
                "enable_fire": LaunchConfiguration("enable_fire"),
                "control_backend": LaunchConfiguration("control_backend"),
                "enemy_color": LaunchConfiguration("enemy_color"),
                "derive_enemy_color_from_mode": LaunchConfiguration(
                    "derive_enemy_color_from_mode"
                ),
                "async_inference": LaunchConfiguration("async_inference"),
                "detector_type": LaunchConfiguration("detector_type"),
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
            image_topic_arg,
            serial_topic_arg,
            cmd_topic_arg,
            respect_mode_arg,
            enable_fire_arg,
            control_backend_arg,
            enemy_color_arg,
            derive_enemy_color_from_mode_arg,
            async_inference_arg,
            detector_type_arg,
            aim_node,
        ]
    )
