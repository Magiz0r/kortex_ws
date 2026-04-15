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
            default_value="/isaac_joint_states",
            description="Primary JointState topic coming from the Isaac articulation state.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "source_joint_command_topic",
            default_value="/isaac_joint_commands",
            description="Optional secondary Isaac JointState topic for custom command publishers.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "real_joint_states_topic",
            default_value="/joint_states",
            description="Real robot JointState topic used for safety checks.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "arm_action_name",
            default_value="/joint_trajectory_controller/follow_joint_trajectory",
            description="Real arm FollowJointTrajectory action server.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_action_name",
            default_value="/robotiq_gripper_controller/gripper_cmd",
            description="Real gripper action server.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "arm_goal_duration_sec",
            default_value="1.0",
            description="Trajectory duration for each Isaac arm target.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "max_arm_joint_step",
            default_value="0.35",
            description="Maximum allowed single-joint step from the real arm state.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "max_gripper_step",
            default_value="0.15",
            description="Maximum allowed gripper step from the real gripper state.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "execute_gripper",
            default_value="true",
            description="Whether to forward Isaac gripper targets to the real gripper.",
        )
    )

    bridge_node = Node(
        package="kortex_bringup",
        executable="isaac_to_real_trajectory_bridge.py",
        name="isaac_to_real_trajectory_bridge",
        output="screen",
        parameters=[
            {
                "source_joint_states_topic": LaunchConfiguration("source_joint_states_topic"),
                "source_joint_command_topic": LaunchConfiguration("source_joint_command_topic"),
                "real_joint_states_topic": LaunchConfiguration("real_joint_states_topic"),
                "arm_action_name": LaunchConfiguration("arm_action_name"),
                "gripper_action_name": LaunchConfiguration("gripper_action_name"),
                "arm_goal_duration_sec": ParameterValue(
                    LaunchConfiguration("arm_goal_duration_sec"), value_type=float
                ),
                "max_arm_joint_step": ParameterValue(
                    LaunchConfiguration("max_arm_joint_step"), value_type=float
                ),
                "max_gripper_step": ParameterValue(
                    LaunchConfiguration("max_gripper_step"), value_type=float
                ),
                "execute_gripper": ParameterValue(
                    LaunchConfiguration("execute_gripper"), value_type=bool
                ),
            }
        ],
    )

    return LaunchDescription(declared_arguments + [bridge_node])
