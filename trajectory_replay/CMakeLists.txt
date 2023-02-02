cmake_minimum_required(VERSION 3.8)
project(trajectory_replay)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

set(dependencies
    rclcpp
    moveit_ros_planning_interface
)

add_executable(moveit_interface_node moveit_interface_node.cpp)

ament_target_dependencies(moveit_interface_node ${dependencies})

install(
    TARGETS moveit_interface_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
