#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    from kortex_api.UDPTransport import UDPTransport
    from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
    from kortex_api.SessionManager import SessionManager
    from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
    from kortex_api.autogen.messages import Session_pb2

    KORTEX_API_IMPORT_ERROR = None
except ImportError as exc:  # pragma: no cover - depends on local Kinova install
    UDPTransport = None
    RouterClient = None
    RouterClientSendOptions = None
    SessionManager = None
    BaseCyclicClient = None
    Session_pb2 = None
    KORTEX_API_IMPORT_ERROR = exc


class KinovaReadOnlyJointStatePublisher(Node):
    def __init__(self) -> None:
        super().__init__("kinova_read_only_joint_state_publisher")

        self.declare_parameter("robot_ip", "192.168.1.10")
        self.declare_parameter("username", "admin")
        self.declare_parameter("password", "admin")
        self.declare_parameter("udp_port", 10001)
        self.declare_parameter("session_inactivity_timeout_ms", 60000)
        self.declare_parameter("connection_inactivity_timeout_ms", 2000)
        self.declare_parameter("publish_rate_hz", 60.0)
        self.declare_parameter("joint_state_topic", "/kinova_read_only/joint_states")
        self.declare_parameter(
            "arm_joint_names",
            ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"],
        )
        self.declare_parameter("publish_gripper", True)
        self.declare_parameter("finger_joint_name", "finger_joint")
        self.declare_parameter("gripper_max_joint_position", 0.81)

        self._robot_ip = self.get_parameter("robot_ip").get_parameter_value().string_value
        self._username = self.get_parameter("username").get_parameter_value().string_value
        self._password = self.get_parameter("password").get_parameter_value().string_value
        self._udp_port = self.get_parameter("udp_port").get_parameter_value().integer_value
        self._session_inactivity_timeout_ms = (
            self.get_parameter("session_inactivity_timeout_ms").get_parameter_value().integer_value
        )
        self._connection_inactivity_timeout_ms = (
            self.get_parameter("connection_inactivity_timeout_ms")
            .get_parameter_value()
            .integer_value
        )
        self._publish_rate_hz = (
            self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        )
        self._joint_state_topic = (
            self.get_parameter("joint_state_topic").get_parameter_value().string_value
        )
        self._arm_joint_names = list(
            self.get_parameter("arm_joint_names").get_parameter_value().string_array_value
        )
        self._publish_gripper = (
            self.get_parameter("publish_gripper").get_parameter_value().bool_value
        )
        self._finger_joint_name = (
            self.get_parameter("finger_joint_name").get_parameter_value().string_value
        )
        self._gripper_max_joint_position = (
            self.get_parameter("gripper_max_joint_position").get_parameter_value().double_value
        )

        if not self._robot_ip:
            raise ValueError("robot_ip must not be empty")
        if self._publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be greater than 0")
        if len(self._arm_joint_names) != 7:
            raise ValueError("arm_joint_names must contain exactly 7 Kinova arm joints")

        self._publisher = self.create_publisher(JointState, self._joint_state_topic, 10)
        self._timer = self.create_timer(1.0 / self._publish_rate_hz, self._publish_joint_state)

        self._transport = None
        self._router = None
        self._session_manager = None
        self._base_cyclic = None
        self._connected = False
        self._reported_import_error = False
        self._last_error: Optional[str] = None
        self._reported_missing_gripper = False

        joint_names = list(self._arm_joint_names)
        if self._publish_gripper:
            joint_names.append(self._finger_joint_name)

        self.get_logger().info(
            "Publishing read-only Kinova feedback from %s to %s for joints: %s"
            % (self._robot_ip, self._joint_state_topic, ", ".join(joint_names))
        )

    def destroy_node(self) -> bool:
        self._disconnect()
        return super().destroy_node()

    def _publish_joint_state(self) -> None:
        if KORTEX_API_IMPORT_ERROR is not None:
            if not self._reported_import_error:
                self.get_logger().error(
                    "kortex_api Python package is not available: %s" % KORTEX_API_IMPORT_ERROR
                )
                self._reported_import_error = True
            return

        if not self._ensure_connection():
            return

        try:
            feedback = self._base_cyclic.RefreshFeedback()
        except Exception as exc:  # pragma: no cover - depends on hardware
            self._handle_runtime_error("Failed to read Kinova feedback: %s" % exc)
            self._disconnect()
            return

        actuator_count = len(feedback.actuators)
        if actuator_count < len(self._arm_joint_names):
            self._handle_runtime_error(
                "Received only %d actuators, expected %d"
                % (actuator_count, len(self._arm_joint_names))
            )
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._arm_joint_names)
        msg.position = []
        msg.velocity = []
        msg.effort = []

        for index, joint_name in enumerate(self._arm_joint_names):
            actuator = feedback.actuators[index]
            msg.position.append(self._wrap_to_pi(math.radians(actuator.position)))
            msg.velocity.append(math.radians(actuator.velocity))
            msg.effort.append(float(actuator.torque))

        if self._publish_gripper:
            msg.name.append(self._finger_joint_name)
            gripper_position = self._read_gripper_position(feedback)
            msg.position.append(gripper_position if gripper_position is not None else 0.0)
            msg.velocity.append(0.0)
            msg.effort.append(0.0)

        self._publisher.publish(msg)

    def _ensure_connection(self) -> bool:
        if self._connected:
            return True

        try:
            self._connect()
        except Exception as exc:  # pragma: no cover - depends on hardware
            self._handle_runtime_error("Failed to connect to Kinova feedback stream: %s" % exc)
            self._disconnect()
            return False

        self._last_error = None
        self.get_logger().info(
            "Connected to Kinova feedback stream at %s:%d" % (self._robot_ip, self._udp_port)
        )
        return True

    def _connect(self) -> None:
        self._transport = UDPTransport()
        self._router = RouterClient(self._transport, RouterClient.basicErrorCallback)
        self._transport.connect(self._robot_ip, self._udp_port)

        session_info = Session_pb2.CreateSessionInfo()
        session_info.username = self._username
        session_info.password = self._password
        session_info.session_inactivity_timeout = int(self._session_inactivity_timeout_ms)
        session_info.connection_inactivity_timeout = int(self._connection_inactivity_timeout_ms)

        self._session_manager = SessionManager(self._router)
        self._session_manager.CreateSession(session_info)
        self._base_cyclic = BaseCyclicClient(self._router)
        self._connected = True

    def _disconnect(self) -> None:
        if self._session_manager is not None:
            try:
                router_options = RouterClientSendOptions()
                router_options.timeout_ms = 1000
                self._session_manager.CloseSession(router_options)
            except Exception:
                pass

        if self._transport is not None:
            try:
                self._transport.disconnect()
            except Exception:
                pass

        self._transport = None
        self._router = None
        self._session_manager = None
        self._base_cyclic = None
        self._connected = False

    def _read_gripper_position(self, feedback) -> Optional[float]:
        try:
            motors = feedback.interconnect.gripper_feedback.motor
            if not motors:
                raise IndexError("no gripper motors reported")
            return float(motors[0].position) / 100.0 * self._gripper_max_joint_position
        except Exception:
            if not self._reported_missing_gripper:
                self.get_logger().warn(
                    "Gripper feedback was not available; publishing %s at 0.0"
                    % self._finger_joint_name
                )
                self._reported_missing_gripper = True
            return None

    def _handle_runtime_error(self, message: str) -> None:
        if message != self._last_error:
            self.get_logger().warn(message)
            self._last_error = message

    @staticmethod
    def _wrap_to_pi(angle_radians: float) -> float:
        wrapped = (angle_radians + math.pi) % (2.0 * math.pi) - math.pi
        if wrapped == -math.pi and angle_radians > 0.0:
            return math.pi
        return wrapped


def main() -> None:
    rclpy.init()
    node = KinovaReadOnlyJointStatePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
