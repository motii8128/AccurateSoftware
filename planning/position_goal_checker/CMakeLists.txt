cmake_minimum_required(VERSION 3.5)
project(position_goal_checker)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_position_goal_checker SHARED
  DIRECTORY src
)
target_include_directories(target_position_goal_checker
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_position_goal_checker
  PLUGIN "position_goal_checker::PositionGoalChecker"
  EXECUTABLE position_goal_checker_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)