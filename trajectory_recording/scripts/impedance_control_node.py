#!/usr/bin/python3
import numpy as np
import rclpy
from lbr_fri_msgs.msg import LBRCommand, LBRState
from rclpy.node import Node


class ImpedanceControl(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self.lbr_command_publisher = self.create_publisher(
            LBRCommand, "/lbr_command", 1
        )
        self.lbr_state_subscriber = self.create_subscription(
            LBRState, "/lbr_state", self.lbr_state_callback, 1
        )

    def lbr_state_callback(self, lbr_state: LBRState) -> None:
        lbr_command = LBRCommand()

        lbr_command.joint_position = (
            np.array(lbr_state.measured_joint_position)
            + 0.01
            * np.where(
                np.array(lbr_state.external_torque) > 4.0,
                np.array(lbr_state.external_torque),
                0.0,
            )
        ).tolist()

        self.get_logger().info(f"comman:\n{lbr_command.joint_position}")

        self.lbr_command_publisher.publish(lbr_command)


def main(args=None):
    rclpy.init(args=args)
    impedance_control = ImpedanceControl("impedance_control_node")
    rclpy.spin(impedance_control)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
