import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import JointState
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class LinkTransformPublisherNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.tf_buffer = Buffer()
        self.transform_listener = TransformListener(self.tf_buffer, self)

        self.declare_parameters(
            namespace="",
            parameters=[
                ("source_frame", "world"),
                ("target_frame", "lbr_link_ee"),
            ],
        )

        self.source_frame = (
            self.get_parameter("source_frame").get_parameter_value().string_value
        )
        self.target_frame = (
            self.get_parameter("target_frame").get_parameter_value().string_value
        )

        self.joint_states_subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_states_callback,
            qos_profile_system_default,
        )

        self.link_transform_publisher = self.create_publisher(
            TransformStamped, "~/link_transform", qos_profile_system_default
        )

    def joint_states_callback(self, joint_states: JointState) -> None:
        tf_stamped = self.tf_buffer.lookup_transform(
            self.target_frame, self.source_frame, 0
        )
        self.link_transform_publisher.publish(tf_stamped)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LinkTransformPublisherNode("end_effector_pose_publisher_node")
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
