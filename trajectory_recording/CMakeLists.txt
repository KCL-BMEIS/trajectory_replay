cmake_minimum_required(VERSION 3.8)
project(trajectory_recording)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/link_transform_publisher_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
