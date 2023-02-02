#!/usr/bin/python3
from typing import List

import rclpy
from geometry_msgs.msg import Quaternion, Vector3
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
from rclpy.action import ActionClient
from rclpy.node import Node


class MoveGroupForwardNode(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        action_name = "/move_action"
        self.move_group_action_client_ = ActionClient(self, MoveGroup, action_name)

        self.get_logger().info(f"Waiting for {action_name} action server...")
        self.move_group_action_client_.wait_for_server()
        self.get_logger().info(f"{action_name} action server is ready.")

        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.max_velocity_scaling_factor = 0.01
        goal.request.allowed_planning_time = 5.0
        goal.request.goal_constraints.append(
            Constraints(
                position_constraints=[
                    PositionConstraint(
                        link_name="lbr_link_ee",
                        target_point_offset=Vector3(
                            x=0.0,
                            y=0.0,
                            z=0.0,
                        ),
                    )
                ],
                orientation_constraints=[
                    OrientationConstraint(
                        link_name="lbr_link_ee",
                        orientation=Quaternion(
                            x=0.0,
                            y=0.0,
                            z=0.0,
                            w=1.0,
                        ),
                    )
                ],
            )
        )
        self.move_group_action_client_.send_goal_async(goal)


def main(args: List = None) -> None:
    rclpy.init(args=args)
    move_group_forward_node = MoveGroupForwardNode("move_group_forward_node")
    rclpy.spin(move_group_forward_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
