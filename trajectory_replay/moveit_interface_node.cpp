#include <memory>
#include <stdexcept>
#include <string>

#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"

class MoveItInterfaceNode : public rclcpp::Node {

public:
  MoveItInterfaceNode(const std::string &node_name = "moveit_interface_node") : Node(node_name) {

    this->declare_parameter<std::string>("move_group_name", "arm");
    move_group_name_ = this->get_parameter("move_group_name").as_string();

    RCLCPP_INFO(this->get_logger(), "MoveItInterfaceNode started");
  }

  bool init() {
    try {
      move_group_interface_ptr_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
          this->shared_from_this(), "arm");
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize MoveItInterfaceNode: %s", e.what());
      return false;
    }
    return true;
  }

protected:
  std::string move_group_name_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_ptr_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto moveit_interface_node = std::make_shared<MoveItInterfaceNode>();
  if (!moveit_interface_node->init()) {
    throw std::runtime_error("Failed to initialize MoveItInterfaceNode");
  }
  rclcpp::spin(moveit_interface_node);
  rclcpp::shutdown();
  return 0;
}
