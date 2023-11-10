#!/usr/bin/python3

import csv
import time
from typing import List

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

import zmq

class JointTrajectoryClientNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.declare_parameters(
            namespace="",
            parameters=[
                (
                    "joint_trajectory_server_name",
                    "position_trajectory_controller/follow_joint_trajectory",
                ),
                (
                    "seconds_from_start",
                    1,
                ),
            ],
        )
        self.init_ = False
        self.joint_trajectory_server_name_ = (
            self.get_parameter("joint_trajectory_server_name")
            .get_parameter_value()
            .string_value
        )
        self.seconds_from_start = (
            self.get_parameter("seconds_from_start").get_parameter_value().integer_value
        )
        self.joint_trajectory_client_ = ActionClient(
            self, FollowJointTrajectory, self.joint_trajectory_server_name_
        )

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

    def send_joint_state_goal(self, joint_state: JointState) -> bool:
        seconds_from_start = self.seconds_from_start
        goal_sec_tolerance = 1
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_state.name
        goal.goal_time_tolerance.sec = goal_sec_tolerance
        if not self.init_:
            seconds_from_start = 10
            self.init_ = True
        goal.trajectory.points.append(
            JointTrajectoryPoint(
                positions=joint_state.position,
                velocities=[0.0] * len(joint_state.position),
                accelerations=[0.0] * len(joint_state.position),
                time_from_start=Duration(sec=seconds_from_start, nanosec=0),
            )
        )
        goal_future = self.joint_trajectory_client_.send_goal_async(goal)

        rclpy.spin_until_future_complete(self, goal_future)
        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by server.")
            return
        self.get_logger().info("Goal was accepted by server.")

        # wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=seconds_from_start + goal_sec_tolerance
        )

        if (
            result_future.result().result.error_code
            != FollowJointTrajectory.Result.SUCCESSFUL
        ):
            self.get_logger().error("Failed to execute joint trajectory.")
            return False
        return True

def load_joint_states(file: str) -> List[JointState]:
    joint_states = []
    with open(file, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            joint_state = JointState(
                name=list(row.keys()), position=[float(xi) for xi in list(row.values())]
            )
            joint_states.append(joint_state)
    return joint_states







class ScanServer():
    
    def __init__(self, ip):
        self.ip = ip
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect(self.ip)
        self.context
        
    def do_scan(self):
        self.socket.send(b"UPDATE")
        message = self.socket.recv()
        if message != b"OK":
            raise RuntimeError(message)
        
    def finish_scan(self):
        self.socket.send(b"FINISH")

def main(args: List = None) -> None:
    
    rclpy.init(args=args)
    joint_trajectory_client_node = JointTrajectoryClientNode(
        "joint_trajectory_client_node"
    )

    joint_states = load_joint_states(
        "/home/charlie/projects/trajectory_replay_ws/src/trajectory_replay/lbr_states.csv"  # TODO: remove path
    )
    
    scan_server = ScanServer("tcp://localhost:5555")

    for joint_state in joint_states[::100]:
        joint_trajectory_client_node.get_logger().info(f"Goal: {joint_state}.")
        if not joint_trajectory_client_node.send_joint_state_goal(joint_state):
            raise RuntimeError("Failed to execute trajectory.")
        scan_server.do_scan()

    scan_server.finish_scan()
    
if __name__ == "__main__":
    main()
