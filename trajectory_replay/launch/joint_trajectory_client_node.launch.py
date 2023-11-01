from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            name="seconds_from_start",
            default_value="1",
            description="Time to move from one target point to the next.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="namespace",
            default_value="lbr",
            description="Namespace for trajectory client node.",
        )
    )

    ld.add_action(
        Node(
            package="trajectory_replay",
            executable="joint_trajectory_client_node.py",
            output="screen",
            parameters=[
                {"seconds_from_start": LaunchConfiguration("seconds_from_start")}
            ],
            namespace=LaunchConfiguration("namespace"),
        )
    )

    return ld
