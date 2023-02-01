import os

import xacro
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # spinner
    lbr_spinner_node = Node(
        package="lbr_fri_ros2",
        executable="lbr_spinner",
        emulate_tty=True,
        output="screen",
    )

    # robot description and controller configurations
    robot_description = {
        "robot_description": xacro.process(
            os.path.join(
                get_package_share_directory("lbr_description"),
                "urdf",
                "med7",
                "med7.urdf.xacro",
            ),
            mappings={
                "sim": "false",
            },
        )
    }

    controller_configurations = os.path.join(
        get_package_share_directory("lbr_bringup"), "config", "lbr_controllers.yml"
    )

    # controller manager
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_configurations],
        output="screen",
    )

    # joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    # rviz2
    rviz2_config = os.path.join(
        get_package_share_directory("trajectory_recording"), "config", "config.rviz"
    )
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz2_config],
    )

    # admittance control noe
    admittance_node = Node(
        package="lbr_examples",
        executable="admittance_control_node.py",
    )

    return LaunchDescription(
        [
            lbr_spinner_node,
            controller_manager_node,
            joint_state_broadcaster_spawner,
            robot_state_publisher_node,
            rviz2_node,
            admittance_node,
        ]
    )
