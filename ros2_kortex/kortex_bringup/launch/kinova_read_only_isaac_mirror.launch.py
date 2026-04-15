# Copyright (c) 2026

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _to_bool(value: str) -> bool:
    return value.strip().lower() in ("1", "true", "yes", "on")


def launch_setup(context, *args, **kwargs):
    publish_gripper = _to_bool(LaunchConfiguration("publish_gripper").perform(context))
    required_joints = [
        "joint_1",
        "joint_2",
        "joint_3",
        "joint_4",
        "joint_5",
        "joint_6",
        "joint_7",
    ]
    if publish_gripper:
        required_joints.append("finger_joint")

    feedback_node = Node(
        package="kortex_driver",
        executable="kinova_read_only_joint_state_publisher",
        name="kinova_read_only_joint_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_ip": LaunchConfiguration("robot_ip").perform(context),
                "username": LaunchConfiguration("username").perform(context),
                "password": LaunchConfiguration("password").perform(context),
                "joint_state_topic": LaunchConfiguration("joint_state_topic").perform(context),
                "publish_rate_hz": float(
                    LaunchConfiguration("feedback_publish_rate_hz").perform(context)
                ),
                "publish_gripper": publish_gripper,
            }
        ],
    )

    mirror_node = Node(
        package="kortex_bringup",
        executable="real_to_isaac_joint_state_bridge.py",
        name="real_to_isaac_joint_state_bridge",
        output="screen",
        parameters=[
            {
                "source_joint_states_topic": LaunchConfiguration("joint_state_topic").perform(
                    context
                ),
                "isaac_joint_commands_topic": LaunchConfiguration(
                    "isaac_joint_commands_topic"
                ).perform(context),
                "publish_rate_hz": float(
                    LaunchConfiguration("mirror_publish_rate_hz").perform(context)
                ),
                "required_joints": required_joints,
                "use_source_timestamp": _to_bool(
                    LaunchConfiguration("use_source_timestamp").perform(context)
                ),
            }
        ],
    )

    return [feedback_node, mirror_node]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            description="IP address of the Kinova arm to mirror into Isaac without taking motion control.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("username", default_value="admin", description="Kinova username.")
    )
    declared_arguments.append(
        DeclareLaunchArgument("password", default_value="admin", description="Kinova password.")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "joint_state_topic",
            default_value="/kinova_read_only/joint_states",
            description="JointState topic published from the read-only Kinova feedback connection.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "isaac_joint_commands_topic",
            default_value="/isaac_joint_commands",
            description="JointState command topic consumed by Isaac's topic_based_ros2_control plugin.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "feedback_publish_rate_hz",
            default_value="60.0",
            description="How often to publish read-only Kinova joint feedback.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mirror_publish_rate_hz",
            default_value="60.0",
            description="How often to republish mirrored commands to Isaac.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "publish_gripper",
            default_value="true",
            description="Include finger_joint feedback when available.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_source_timestamp",
            default_value="false",
            description="Reuse incoming JointState timestamps when mirroring to Isaac.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
