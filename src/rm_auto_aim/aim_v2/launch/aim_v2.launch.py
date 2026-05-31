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
            detector_type_arg,
            aim_node,
        ]
    )
