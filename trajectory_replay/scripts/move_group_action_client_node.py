#!/usr/bin/python3
from typing import List

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    OrientationConstraint,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Point

import csv


class MoveGroupActionClientNode(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self.declare_parameters(
            namespace="",
            parameters=[
                ("move_group_action_server_name", "/move_action"),
                ("move_group_name", "med7_arm"),
                ("base", "world"),
                ("end_effector", "lbr_link_ee"),
            ],
        )
        self.move_group_action_server_name = (
            self.get_parameter("move_group_action_server_name")
            .get_parameter_value()
            .string_value
        )
        self.move_group_name = (
            self.get_parameter("move_group_name").get_parameter_value().string_value
        )
        self.base = self.get_parameter("base").get_parameter_value().string_value
        self.end_effector = (
            self.get_parameter("end_effector").get_parameter_value().string_value
        )

        self.move_group_action_client = ActionClient(
            self, MoveGroup, self.move_group_action_server_name
        )

        self.get_logger().info(
            f"Waiting for action server {self.move_group_action_server_name}..."
        )
        if not self.move_group_action_client.wait_for_server(timeout_sec=1):
            raise RuntimeError(
                f"Couldn't connect to action server {self.move_group_action_server_name}."
            )
        self.get_logger().info(f"Done.")

    def send_goal_async(self, target: Pose):
        goal = MoveGroup.Goal()
        goal.request.allowed_planning_time = 1.0
        goal.request.goal_constraints.append(
            Constraints(
                position_constraints=[
                    PositionConstraint(
                        header=Header(frame_id=self.base),
                        link_name=self.end_effector,
                        constraint_region=BoundingVolume(
                            primitives=[SolidPrimitive(type=2, dimensions=[0.0001])],
                            primitive_poses=[Pose(position=target.position)],
                        ),
                        weight=1.0,
                    )
                ],
                orientation_constraints=[
                    OrientationConstraint(
                        header=Header(frame_id=self.base),
                        link_name=self.end_effector,
                        orientation=target.orientation,
                        absolute_x_axis_tolerance=0.001,
                        absolute_y_axis_tolerance=0.001,
                        absolute_z_axis_tolerance=0.001,
                        weight=1.0,
                    )
                ],
            )
        )
        goal.request.group_name = self.move_group_name
        goal.request.max_acceleration_scaling_factor = 0.1
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.num_planning_attempts = 1

        return self.move_group_action_client.send_goal_async(goal)


def load_poses(file: str) -> List:
    data = []
    with open(file, "r") as f:
        reader = csv.reader(f)
        for row in reader:
            row = [float(xi) for xi in row]
            data.append(row)
    return data


def main(args: List = None) -> None:
    rclpy.init(args=args)
    move_group_action_client_node = MoveGroupActionClientNode(
        "move_group_action_client_node"
    )

    poses = load_poses("/tmp/traj/src/trajectory_replay/data.csv")

    # load list
    for pose in poses:
        pose = Pose(
            position=Point(x=pose[0], y=pose[1], z=pose[2]),
            orientation=Quaternion(x=pose[3], y=pose[4], z=pose[5], w=pose[6]),
        )
        future = move_group_action_client_node.send_goal_async(pose)
        rclpy.spin_until_future_complete(
            move_group_action_client_node, future
        )  # gets stuck for invalid goals

    rclpy.shutdown()


if __name__ == "__main__":
    main()
