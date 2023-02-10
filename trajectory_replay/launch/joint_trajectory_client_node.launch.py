from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    launch_args = []
    launch_args.append(
        DeclareLaunchArgument(
            name="seconds_from_start",
            default_value="1",
            description="Time to move from one target point to the next.",
        )
    )

    joint_trajectory_client_node = Node(
        package="trajectory_replay",
        executable="joint_trajectory_client_node.py",
        output="screen",
        parameters=[{"seconds_from_start": LaunchConfiguration("seconds_from_start")}],
    )

    return LaunchDescription(launch_args+ [joint_trajectory_client_node])
