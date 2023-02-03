#!/usr/bin/python3

import time
from typing import List

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


class JointTrajectoryClientNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.declare_parameters(
            namespace="",
            parameters=[
                (
                    "joint_trajectory_server_name",
                    "/position_trajectory_controller/follow_joint_trajectory",
                ),
            ],
        )
        self.joint_trajectory_server_name_ = (
            self.get_parameter("joint_trajectory_server_name")
            .get_parameter_value()
            .string_value
        )
        self.joint_trajectory_client_ = ActionClient(
            self, FollowJointTrajectory, self.joint_trajectory_server_name_
        )
        self.joint_trajectory_goal_future_ = None
        self.joint_trajectory_result_future_ = None

        self.get_logger().info(
            f"Waiting for action server {self.joint_trajectory_server_name_}..."
        )
        if not self.joint_trajectory_client_.wait_for_server(timeout_sec=2):
            self.get_logger().error(
                f"Action server {self.joint_trajectory_server_name_} not available."
            )
            self.destroy_node()
            return
        self.get_logger().info("Done")
        self.get_logger().info("JointTrajectoryClientNode has been started.")

    @property
    def joint_trajectory_goal_future(self) -> Future:
        return self.joint_trajectory_goal_future_

    @property
    def joint_trajectory_result_future(self) -> Future:
        return self.joint_trajectory_result_future_

    def send_joint_state_goal_async(self, joint_state: JointState) -> bool:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        goal.trajectory.joint_names = joint_state.name
        goal.trajectory.points.append(
            JointTrajectoryPoint(
                positions=joint_state.position,
                time_from_start=Duration(sec=1, nanosec=0),
            )
        )
        self.joint_trajectory_goal_future_ = (
            self.joint_trajectory_client_.send_goal_async(goal)
        )
        rclpy.spin_until_future_complete(self, self.joint_trajectory_goal_future_)
        goal_handle = self.joint_trajectory_goal_future_.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected :(")
            return False
        self.get_logger().info("Goal accepted :)")
        self.joint_trajectory_result_future_ = goal_handle.get_result_async()
        return True

    def is_done(self) -> bool:
        if not self.joint_trajectory_result_future_:
            return True
        rclpy.spin_until_future_complete(self, self.joint_trajectory_result_future_)
        if (
            self.joint_trajectory_result_future_.result().status
            == GoalStatus.STATUS_SUCCEEDED
        ):
            return True
        else:
            return False


def blocking_camera_scan():
    print("Scanning camera...")
    time.sleep(1)
    print("Done")


def main(args: List = None) -> None:
    rclpy.init(args=args)
    joint_trajectory_client_node = JointTrajectoryClientNode(
        "joint_trajectory_client_node"
    )

    for _ in range(3):
        joint_state = JointState(
            name=[
                "lbr_joint_0",
                "lbr_joint_1",
                "lbr_joint_2",
                "lbr_joint_3",
                "lbr_joint_4",
                "lbr_joint_5",
                "lbr_joint_6",
            ],
            position=[0.0] * 4 + (np.random.randn(3) * 0.1 - 0.05).tolist(),
        )
        joint_trajectory_client_node.send_joint_state_goal_async(joint_state)
        while not joint_trajectory_client_node.is_done():
            time.sleep(0.1)

        # scan camera
        blocking_camera_scan()


if __name__ == "__main__":
    main()