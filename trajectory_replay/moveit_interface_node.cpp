#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

class MoveItInterfaceNode : public rclcpp::Node {

public:
  MoveItInterfaceNode(const std::string &node_name, const rclcpp::NodeOptions &node_options)
      : Node(node_name, node_options) {

    this->declare_parameter<std::string>("move_group_name", "med7_arm");
    this->declare_parameter<double>("max_velocity_scaling_factor", 0.01);
    this->declare_parameter<double>("max_acceleration_scaling_factor", 0.1);
    move_group_name_ = this->get_parameter("move_group_name").as_string();
    max_velocity_scaling_factor_ = this->get_parameter("max_velocity_scaling_factor").as_double();
    max_acceleration_scaling_factor_ =
        this->get_parameter("max_acceleration_scaling_factor").as_double();

    motion_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/motion_service",
        std::bind(&MoveItInterfaceNode::motion_service_callback_, this, std::placeholders::_1,
                  std::placeholders::_2),
        rmw_qos_profile_system_default);

    RCLCPP_INFO(this->get_logger(), "MoveItInterfaceNode started");
  }

  bool init() {
    try {
      move_group_interface_ptr_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
          this->shared_from_this(), move_group_name_);

      this->move_group_interface_ptr_->setMaxVelocityScalingFactor(
          this->max_velocity_scaling_factor_);
      this->move_group_interface_ptr_->setMaxAccelerationScalingFactor(
          this->max_acceleration_scaling_factor_);

    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize MoveItInterfaceNode: %s", e.what());
      return false;
    }
    return true;
  }

protected:
  void motion_service_callback_(const std_srvs::srv::Trigger::Request::SharedPtr,
                                std_srvs::srv::Trigger::Response::SharedPtr response) {
    RCLCPP_INFO(this->get_logger(), "Triggering motion!");
    geometry_msgs::msg::Pose target;
    target.position.z = 1.;
    this->move_group_interface_ptr_->setPoseTarget(target);
    this->move_group_interface_ptr_->move();
  }

  std::string move_group_name_;
  double max_velocity_scaling_factor_, max_acceleration_scaling_factor_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_ptr_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr motion_service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto moveit_interface_node = std::make_shared<MoveItInterfaceNode>(
      "moveit_interface_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  if (!moveit_interface_node->init()) {
    throw std::runtime_error("Failed to initialize MoveItInterfaceNode");
  }
  rclcpp::spin(moveit_interface_node);
  rclcpp::shutdown();
  return 0;
}
