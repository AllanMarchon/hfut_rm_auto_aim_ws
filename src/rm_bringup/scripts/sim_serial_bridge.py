#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from rm_interfaces.msg import SerialReceiveData


class SimSerialBridge(Node):
    def __init__(self):
        super().__init__("sim_serial_bridge")

        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("serial_topic", "/serial/receive")
        self.declare_parameter("yaw_joint_name", "yaw_joint")
        self.declare_parameter("pitch_joint_name", "pitch_joint")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("vision_mode", 0)
        self.declare_parameter("bullet_speed", 22.5)
        self.declare_parameter("publish_rate", 100.0)
        self.declare_parameter("roll", 0.0)

        self._latest_joint_state = None
        joint_states_topic = (
            self.get_parameter("joint_states_topic").get_parameter_value().string_value
        )
        serial_topic = self.get_parameter("serial_topic").get_parameter_value().string_value
        publish_rate = max(
            1.0, self.get_parameter("publish_rate").get_parameter_value().double_value
        )

        self._pub = self.create_publisher(
            SerialReceiveData, serial_topic, qos_profile_sensor_data
        )
        self._sub = self.create_subscription(
            JointState,
            joint_states_topic,
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self._timer = self.create_timer(1.0 / publish_rate, self._publish_serial)

        self.get_logger().info(
            f"sim_serial_bridge: {joint_states_topic} -> {serial_topic}, "
            f"rate={publish_rate:.1f}Hz"
        )

    def _on_joint_state(self, msg: JointState):
        self._latest_joint_state = msg

    def _publish_serial(self):
        if self._latest_joint_state is None:
            return

        joint_state = self._latest_joint_state
        yaw_rad = self._joint_position(
            joint_state,
            self.get_parameter("yaw_joint_name").get_parameter_value().string_value,
            fallback_index=0,
        )
        pitch_rad = self._joint_position(
            joint_state,
            self.get_parameter("pitch_joint_name").get_parameter_value().string_value,
            fallback_index=1,
        )

        msg = SerialReceiveData()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        msg.mode = (
            self.get_parameter("vision_mode").get_parameter_value().integer_value & 0xFF
        )
        msg.bullet_speed = float(
            self.get_parameter("bullet_speed").get_parameter_value().double_value
        )
        msg.roll = float(self.get_parameter("roll").get_parameter_value().double_value)
        msg.yaw = math.degrees(yaw_rad)
        msg.pitch = math.degrees(pitch_rad)
        self._pub.publish(msg)

    @staticmethod
    def _joint_position(msg: JointState, joint_name: str, fallback_index: int) -> float:
        if joint_name in msg.name:
            index = msg.name.index(joint_name)
            if index < len(msg.position):
                return msg.position[index]
        if fallback_index < len(msg.position):
            return msg.position[fallback_index]
        return 0.0


def main():
    rclpy.init()
    node = SimSerialBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
