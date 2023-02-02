import os

import xacro
import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def load_yaml(package_name: str, file_path: str):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as f:
            return yaml.safe_load(f)
    except EnvironmentError:
        return None


def load_file(package_name: str, file_path: str) -> str:
    package_path = get_package_share_directory(package_name)
    absolut_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolut_file_path, "r") as f:
            return f.read()
    except EnvironmentError:
        return None


def generate_launch_description() -> LaunchDescription:

    model = "med7"
    sim = "true"

    robot_description = {
        "robot_description": xacro.process(
            os.path.join(
                get_package_share_directory("lbr_description"),
                "urdf",
                model,
                f"{model}.urdf.xacro",
            ),
            mappings={"sim": sim},
        )
    }

    robot_description_semantic = {
        "robot_description_semantic": load_file("lbr_moveit", f"srdf/{model}.srdf")
    }

    kinematics_yaml = load_yaml("lbr_moveit", "config/kinematics.yml")

    # Update group name
    kinematics_yaml["{}_arm".format(model)] = kinematics_yaml["group_name"]
    del kinematics_yaml["group_name"]

    moveit_interface_node = Node(
        package="trajectory_replay",
        executable="moveit_interface_node",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml],
        output="screen",
    )

    return LaunchDescription([moveit_interface_node])
