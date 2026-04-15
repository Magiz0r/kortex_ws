# Copyright (c) 2026

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "source_joint_states_topic",
            default_value="/joint_states",
            description="JointState topic from the real robot stack.",
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
            "publish_rate_hz",
            default_value="60.0",
            description="How often to republish mirrored commands to Isaac.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_source_timestamp",
            default_value="false",
            description="Reuse the incoming JointState header stamp instead of the current clock.",
        )
    )

    mirror_node = Node(
        package="kortex_bringup",
        executable="real_to_isaac_joint_state_bridge.py",
        name="real_to_isaac_joint_state_bridge",
        output="screen",
        parameters=[
            {
                "source_joint_states_topic": LaunchConfiguration("source_joint_states_topic"),
                "isaac_joint_commands_topic": LaunchConfiguration("isaac_joint_commands_topic"),
                "publish_rate_hz": ParameterValue(
                    LaunchConfiguration("publish_rate_hz"), value_type=float
                ),
                "use_source_timestamp": ParameterValue(
                    LaunchConfiguration("use_source_timestamp"), value_type=bool
                ),
            }
        ],
    )

    return LaunchDescription(declared_arguments + [mirror_node])
