#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"

#include "trajectory_replay_msgs/srv/pose.hpp"

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

    pose_service_ptr_ = this->create_service<trajectory_replay_msgs::srv::Pose>(
        "~/pose_service",
        std::bind(&MoveItInterfaceNode::pose_service_callback_, this, std::placeholders::_1,
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
  void pose_service_callback_(const trajectory_replay_msgs::srv::Pose::Request::SharedPtr request,
                              trajectory_replay_msgs::srv::Pose::Response::SharedPtr response) {
    this->move_group_interface_ptr_->setPoseTarget(request->target);
    RCLCPP_INFO(this->get_logger(), "Attempting asynchronous move...");
    auto ret = this->move_group_interface_ptr_->asyncMove();
    if (ret != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      std::stringstream ss;
      ss << "Failed to move move group " << move_group_name_ << " with error code "
         << std::to_string(ret.val);
      response->message = ss.str();
      response->success = false;
      RCLCPP_ERROR(this->get_logger(), response->message);
      return;
    }
    response->message = "Success";
    response->success = true;
    RCLCPP_INFO(this->get_logger(), response->message);
  }

  std::string move_group_name_;
  double max_velocity_scaling_factor_, max_acceleration_scaling_factor_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_ptr_;
  rclcpp::Service<trajectory_replay_msgs::srv::Pose>::SharedPtr pose_service_ptr_;
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
