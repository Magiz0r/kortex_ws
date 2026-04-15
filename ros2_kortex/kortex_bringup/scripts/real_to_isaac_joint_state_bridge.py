#!/usr/bin/env python3

from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class RealToIsaacJointStateBridge(Node):
    def __init__(self) -> None:
        super().__init__("real_to_isaac_joint_state_bridge")

        default_joints = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
            "joint_7",
            "finger_joint",
        ]

        self.declare_parameter("source_joint_states_topic", "/joint_states")
        self.declare_parameter("isaac_joint_commands_topic", "/isaac_joint_commands")
        self.declare_parameter("publish_rate_hz", 60.0)
        self.declare_parameter("required_joints", default_joints)
        self.declare_parameter("use_source_timestamp", False)

        self._source_topic = (
            self.get_parameter("source_joint_states_topic").get_parameter_value().string_value
        )
        self._isaac_topic = (
            self.get_parameter("isaac_joint_commands_topic").get_parameter_value().string_value
        )
        self._publish_rate_hz = (
            self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        )
        self._required_joints = list(
            self.get_parameter("required_joints").get_parameter_value().string_array_value
        )
        self._use_source_timestamp = (
            self.get_parameter("use_source_timestamp").get_parameter_value().bool_value
        )

        if self._publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be greater than 0")

        self._latest_joint_state: Optional[JointState] = None
        self._joint_positions: Dict[str, float] = {}
        self._joint_velocities: Dict[str, float] = {}
        self._joint_efforts: Dict[str, float] = {}
        self._ready_logged = False
        self._last_missing: Tuple[str, ...] = tuple()

        self._publisher = self.create_publisher(JointState, self._isaac_topic, 10)
        self._subscription = self.create_subscription(
            JointState, self._source_topic, self._joint_state_callback, 50
        )
        self._timer = self.create_timer(1.0 / self._publish_rate_hz, self._publish_joint_commands)

        self.get_logger().info(
            "Mirroring %s to %s for joints: %s"
            % (self._source_topic, self._isaac_topic, ", ".join(self._required_joints))
        )

    def _joint_state_callback(self, msg: JointState) -> None:
        self._latest_joint_state = msg

        for index, joint_name in enumerate(msg.name):
            if index < len(msg.position):
                self._joint_positions[joint_name] = msg.position[index]
            if index < len(msg.velocity):
                self._joint_velocities[joint_name] = msg.velocity[index]
            if index < len(msg.effort):
                self._joint_efforts[joint_name] = msg.effort[index]

    def _publish_joint_commands(self) -> None:
        if self._latest_joint_state is None:
            return

        missing_joints = tuple(
            joint_name
            for joint_name in self._required_joints
            if joint_name not in self._joint_positions
        )
        if missing_joints:
            if missing_joints != self._last_missing:
                self.get_logger().warn(
                    "Waiting for required joints from %s: %s"
                    % (self._source_topic, ", ".join(missing_joints))
                )
                self._last_missing = missing_joints
            return

        if not self._ready_logged:
            self.get_logger().info("Received all required joints, publishing Isaac mirror commands")
            self._ready_logged = True
            self._last_missing = tuple()

        command_msg = JointState()
        command_msg.name = list(self._required_joints)
        command_msg.position = [self._joint_positions[joint_name] for joint_name in self._required_joints]

        if all(joint_name in self._joint_velocities for joint_name in self._required_joints):
            command_msg.velocity = [
                self._joint_velocities[joint_name] for joint_name in self._required_joints
            ]

        if all(joint_name in self._joint_efforts for joint_name in self._required_joints):
            command_msg.effort = [
                self._joint_efforts[joint_name] for joint_name in self._required_joints
            ]

        if self._use_source_timestamp:
            command_msg.header = self._latest_joint_state.header
        else:
            command_msg.header.stamp = self.get_clock().now().to_msg()

        self._publisher.publish(command_msg)


def main() -> None:
    rclpy.init()
    node = RealToIsaacJointStateBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
