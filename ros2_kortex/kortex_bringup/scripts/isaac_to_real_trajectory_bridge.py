#!/usr/bin/env python3

import math
from typing import Dict, List, Optional, Sequence, Set

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


class IsaacToRealTrajectoryBridge(Node):
    def __init__(self) -> None:
        super().__init__("isaac_to_real_trajectory_bridge")

        default_arm_joints = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
            "joint_7",
        ]
        default_continuous_joints = [
            "joint_1",
            "joint_3",
            "joint_5",
            "joint_7",
        ]

        self.declare_parameter("source_joint_states_topic", "/isaac_joint_states")
        self.declare_parameter("source_joint_command_topic", "/isaac_joint_commands")
        self.declare_parameter("real_joint_states_topic", "/joint_states")
        self.declare_parameter(
            "arm_action_name", "/joint_trajectory_controller/follow_joint_trajectory"
        )
        self.declare_parameter("gripper_action_name", "/robotiq_gripper_controller/gripper_cmd")
        self.declare_parameter("arm_joints", default_arm_joints)
        self.declare_parameter("gripper_joint", "finger_joint")
        self.declare_parameter("arm_goal_duration_sec", 1.0)
        self.declare_parameter("gripper_max_effort", 100.0)
        self.declare_parameter("arm_position_epsilon", 0.01)
        self.declare_parameter("gripper_position_epsilon", 0.01)
        self.declare_parameter("arm_command_trigger_epsilon", 0.08)
        self.declare_parameter("arm_target_stable_time_sec", 0.25)
        self.declare_parameter("arm_goal_cooldown_sec", 0.75)
        self.declare_parameter("max_arm_joint_step", 0.35)
        self.declare_parameter("max_gripper_step", 0.15)
        self.declare_parameter("real_state_timeout_sec", 1.0)
        self.declare_parameter("execute_gripper", True)
        self.declare_parameter("timer_period_sec", 0.1)
        self.declare_parameter("continuous_joints", default_continuous_joints)

        self._source_joint_states_topic = self.get_parameter(
            "source_joint_states_topic"
        ).get_parameter_value().string_value
        self._source_joint_command_topic = self.get_parameter(
            "source_joint_command_topic"
        ).get_parameter_value().string_value
        self._real_joint_states_topic = self.get_parameter(
            "real_joint_states_topic"
        ).get_parameter_value().string_value
        self._arm_action_name = self.get_parameter("arm_action_name").get_parameter_value().string_value
        self._gripper_action_name = (
            self.get_parameter("gripper_action_name").get_parameter_value().string_value
        )
        self._arm_joints = list(self.get_parameter("arm_joints").get_parameter_value().string_array_value)
        self._gripper_joint = self.get_parameter("gripper_joint").get_parameter_value().string_value
        self._arm_goal_duration_sec = (
            self.get_parameter("arm_goal_duration_sec").get_parameter_value().double_value
        )
        self._gripper_max_effort = (
            self.get_parameter("gripper_max_effort").get_parameter_value().double_value
        )
        self._arm_position_epsilon = (
            self.get_parameter("arm_position_epsilon").get_parameter_value().double_value
        )
        self._gripper_position_epsilon = (
            self.get_parameter("gripper_position_epsilon").get_parameter_value().double_value
        )
        self._arm_command_trigger_epsilon = (
            self.get_parameter("arm_command_trigger_epsilon").get_parameter_value().double_value
        )
        self._arm_target_stable_time_sec = (
            self.get_parameter("arm_target_stable_time_sec").get_parameter_value().double_value
        )
        self._arm_goal_cooldown_sec = (
            self.get_parameter("arm_goal_cooldown_sec").get_parameter_value().double_value
        )
        self._max_arm_joint_step = (
            self.get_parameter("max_arm_joint_step").get_parameter_value().double_value
        )
        self._max_gripper_step = (
            self.get_parameter("max_gripper_step").get_parameter_value().double_value
        )
        self._real_state_timeout_sec = (
            self.get_parameter("real_state_timeout_sec").get_parameter_value().double_value
        )
        self._execute_gripper = (
            self.get_parameter("execute_gripper").get_parameter_value().bool_value
        )
        self._timer_period_sec = (
            self.get_parameter("timer_period_sec").get_parameter_value().double_value
        )
        self._continuous_joints: Set[str] = set(
            self.get_parameter("continuous_joints").get_parameter_value().string_array_value
        )

        self._real_joint_positions: Dict[str, float] = {}
        self._last_real_state_time = None
        self._pending_arm_target: Optional[List[float]] = None
        self._pending_arm_target_time = None
        self._pending_gripper_target: Optional[float] = None
        self._last_sent_arm_target: Optional[List[float]] = None
        self._last_sent_gripper_target: Optional[float] = None
        self._arm_goal_in_flight = False
        self._gripper_goal_in_flight = False
        self._arm_goal_cooldown_until = None
        self._arm_server_warned = False
        self._gripper_server_warned = False
        self._real_state_warned = False
        self._missing_source_warned = False

        self._source_topics = [
            topic_name
            for topic_name in (
                self._source_joint_states_topic,
                self._source_joint_command_topic,
            )
            if topic_name
        ]
        self._source_topics = list(dict.fromkeys(self._source_topics))
        if not self._source_topics:
            raise ValueError("At least one Isaac source topic must be configured")

        self._arm_client = ActionClient(
            self, FollowJointTrajectory, self._arm_action_name
        )
        self._gripper_client = ActionClient(
            self, GripperCommand, self._gripper_action_name
        )

        self._source_subscriptions = [
            self.create_subscription(
                JointState, topic_name, self._source_joint_command_callback, 20
            )
            for topic_name in self._source_topics
        ]
        self.create_subscription(
            JointState, self._real_joint_states_topic, self._real_joint_state_callback, 50
        )
        self.create_timer(self._timer_period_sec, self._dispatch_pending_commands)

        self.get_logger().info(
            "Relaying Isaac targets from %s to %s and %s"
            % (", ".join(self._source_topics), self._arm_action_name, self._gripper_action_name)
        )

    def _real_joint_state_callback(self, msg: JointState) -> None:
        self._last_real_state_time = self.get_clock().now()
        for index, joint_name in enumerate(msg.name):
            if index < len(msg.position):
                self._real_joint_positions[joint_name] = msg.position[index]

    def _source_joint_command_callback(self, msg: JointState) -> None:
        position_map = {
            joint_name: msg.position[index]
            for index, joint_name in enumerate(msg.name)
            if index < len(msg.position)
        }

        missing_arm_joints = [
            joint_name for joint_name in self._arm_joints if joint_name not in position_map
        ]
        if missing_arm_joints:
            if not self._missing_source_warned:
                self.get_logger().warn(
                    "Ignoring source command because Isaac target is missing joints: %s"
                    % ", ".join(missing_arm_joints)
                )
                self._missing_source_warned = True
            return

        self._missing_source_warned = False
        arm_target = [position_map[joint_name] for joint_name in self._arm_joints]
        if self._arm_targets_differ(arm_target, self._pending_arm_target, self._arm_position_epsilon):
            self._pending_arm_target = arm_target
            self._pending_arm_target_time = self.get_clock().now()

        if self._execute_gripper and self._gripper_joint in position_map:
            self._pending_gripper_target = position_map[self._gripper_joint]

    def _dispatch_pending_commands(self) -> None:
        if not self._real_state_is_fresh():
            return

        if self._pending_arm_target is not None and not self._arm_goal_in_flight:
            if self._arm_goal_is_ready(self._pending_arm_target):
                if self._arm_step_is_safe(self._pending_arm_target):
                    self._send_arm_goal(self._pending_arm_target)

        if (
            self._execute_gripper
            and self._pending_gripper_target is not None
            and not self._gripper_goal_in_flight
        ):
            if self._gripper_target_changed(self._pending_gripper_target):
                if self._gripper_step_is_safe(self._pending_gripper_target):
                    self._send_gripper_goal(self._pending_gripper_target)

    def _real_state_is_fresh(self) -> bool:
        if self._last_real_state_time is None:
            if not self._real_state_warned:
                self.get_logger().warn(
                    "Waiting for fresh real robot state on %s before executing Isaac targets"
                    % self._real_joint_states_topic
                )
                self._real_state_warned = True
            return False

        age = (self.get_clock().now() - self._last_real_state_time).nanoseconds / 1e9
        if age > self._real_state_timeout_sec:
            if not self._real_state_warned:
                self.get_logger().warn(
                    "Real robot state is stale (%.3fs old), holding Isaac commands" % age
                )
                self._real_state_warned = True
            return False

        self._real_state_warned = False
        return True

    def _arm_target_changed(self, target: Sequence[float]) -> bool:
        if self._last_sent_arm_target is None:
            return True
        return self._arm_targets_differ(target, self._last_sent_arm_target, self._arm_position_epsilon)

    def _arm_targets_differ(
        self,
        left: Optional[Sequence[float]],
        right: Optional[Sequence[float]],
        epsilon: float,
    ) -> bool:
        if left is None or right is None:
            return True
        return any(
            self._joint_distance(self._arm_joints[index], left[index], right[index]) > epsilon
            for index in range(len(self._arm_joints))
        )

    def _arm_target_is_significant(self, target: Sequence[float]) -> bool:
        if any(
            self._joint_distance(joint_name, target[index], self._real_joint_positions[joint_name])
            > self._arm_command_trigger_epsilon
            for index, joint_name in enumerate(self._arm_joints)
            if joint_name in self._real_joint_positions
        ):
            return True

        if self._last_sent_arm_target is None:
            return False

        return any(
            self._joint_distance(joint_name, target[index], self._last_sent_arm_target[index])
            > self._arm_command_trigger_epsilon
            for index, joint_name in enumerate(self._arm_joints)
        )

    def _arm_goal_is_ready(self, target: Sequence[float]) -> bool:
        if self._pending_arm_target_time is None:
            return False

        now = self.get_clock().now()
        if self._arm_goal_cooldown_until is not None and now < self._arm_goal_cooldown_until:
            return False

        target_age_sec = (now - self._pending_arm_target_time).nanoseconds / 1e9
        if target_age_sec < self._arm_target_stable_time_sec:
            return False

        if not self._arm_target_changed(target):
            return False

        return self._arm_target_is_significant(target)

    def _gripper_target_changed(self, target: float) -> bool:
        if self._last_sent_gripper_target is None:
            return True
        return abs(target - self._last_sent_gripper_target) > self._gripper_position_epsilon

    def _arm_step_is_safe(self, target: Sequence[float]) -> bool:
        missing_real_joints = [
            joint_name for joint_name in self._arm_joints if joint_name not in self._real_joint_positions
        ]
        if missing_real_joints:
            self.get_logger().warn(
                "Cannot validate arm step, real state is missing joints: %s"
                % ", ".join(missing_real_joints)
            )
            return False

        for index, joint_name in enumerate(self._arm_joints):
            current = self._real_joint_positions[joint_name]
            step_size = self._joint_distance(joint_name, target[index], current)
            if step_size > self._max_arm_joint_step:
                self.get_logger().warn(
                    "Rejected Isaac arm target because %s step is too large (current %.3f, target %.3f, wrapped step %.3f, limit %.3f)"
                    % (joint_name, current, target[index], step_size, self._max_arm_joint_step)
                )
                return False

        return True

    def _joint_distance(self, joint_name: str, target: float, current: float) -> float:
        delta = target - current
        if joint_name in self._continuous_joints:
            return abs(math.atan2(math.sin(delta), math.cos(delta)))
        return abs(delta)

    def _gripper_step_is_safe(self, target: float) -> bool:
        if self._gripper_joint not in self._real_joint_positions:
            self.get_logger().warn(
                "Cannot validate gripper step, real state is missing %s" % self._gripper_joint
            )
            return False

        current = self._real_joint_positions[self._gripper_joint]
        if abs(target - current) > self._max_gripper_step:
            self.get_logger().warn(
                "Rejected Isaac gripper target because step is too large (current %.3f, target %.3f, limit %.3f)"
                % (current, target, self._max_gripper_step)
            )
            return False

        return True

    def _send_arm_goal(self, target: Sequence[float]) -> None:
        if not self._arm_client.server_is_ready():
            if not self._arm_server_warned:
                self.get_logger().warn(
                    "Arm action server %s is not ready yet" % self._arm_action_name
                )
                self._arm_server_warned = True
            return

        self._arm_server_warned = False
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = list(self._arm_joints)

        point = JointTrajectoryPoint()
        point.positions = list(target)
        point.time_from_start = self._seconds_to_duration(self._arm_goal_duration_sec)
        goal_msg.trajectory.points = [point]

        self._arm_goal_in_flight = True
        future = self._arm_client.send_goal_async(goal_msg)
        future.add_done_callback(self._arm_goal_response_callback)
        self._last_sent_arm_target = list(target)
        self.get_logger().info("Sent Isaac arm target to real trajectory controller")

    def _send_gripper_goal(self, target: float) -> None:
        if not self._gripper_client.server_is_ready():
            if not self._gripper_server_warned:
                self.get_logger().warn(
                    "Gripper action server %s is not ready yet" % self._gripper_action_name
                )
                self._gripper_server_warned = True
            return

        self._gripper_server_warned = False
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = float(target)
        goal_msg.command.max_effort = self._gripper_max_effort

        self._gripper_goal_in_flight = True
        future = self._gripper_client.send_goal_async(goal_msg)
        future.add_done_callback(self._gripper_goal_response_callback)
        self._last_sent_gripper_target = float(target)
        self.get_logger().info("Sent Isaac gripper target to real gripper controller")

    def _arm_goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._arm_goal_in_flight = False
            self.get_logger().warn("Real arm controller rejected Isaac goal")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._arm_result_callback)

    def _gripper_goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._gripper_goal_in_flight = False
            self.get_logger().warn("Real gripper controller rejected Isaac goal")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._gripper_result_callback)

    def _arm_result_callback(self, future) -> None:
        self._arm_goal_in_flight = False
        self._arm_goal_cooldown_until = self.get_clock().now() + rclpy.duration.Duration(
            seconds=self._arm_goal_cooldown_sec
        )
        result = future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().warn(
                "Real arm controller finished Isaac goal with error %d: %s"
                % (result.error_code, result.error_string)
            )

    def _gripper_result_callback(self, future) -> None:
        self._gripper_goal_in_flight = False
        result = future.result().result
        if not result.reached_goal:
            self.get_logger().warn("Real gripper did not fully reach the Isaac target")

    @staticmethod
    def _seconds_to_duration(seconds: float) -> Duration:
        sec = int(seconds)
        nanosec = int((seconds - sec) * 1e9)
        return Duration(sec=sec, nanosec=nanosec)


def main() -> None:
    rclpy.init()
    node = IsaacToRealTrajectoryBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
