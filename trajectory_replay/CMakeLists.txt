cmake_minimum_required(VERSION 3.8)
project(trajectory_replay)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(trajectory_replay_msgs REQUIRED)

set(dependencies
    geometry_msgs    
    moveit_ros_planning_interface
    rclcpp
    std_srvs
    trajectory_replay_msgs
)

add_executable(moveit_interface_node moveit_interface_node.cpp)

ament_target_dependencies(moveit_interface_node ${dependencies})

install(
    TARGETS moveit_interface_node
    DESTINATION lib/${PROJECT_NAME}
)

install(
    PROGRAMS
        scripts/move_group_action_client_node.py
        scripts/joint_trajectory_client_node.py
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
