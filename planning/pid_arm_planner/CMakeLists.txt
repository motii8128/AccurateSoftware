cmake_minimum_required(VERSION 3.5)
project(pid_arm_planner)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_pid_arm_planner SHARED
  DIRECTORY src
)
target_include_directories(target_pid_arm_planner
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_pid_arm_planner
  PLUGIN "pid_arm_planner::PidArmPlanner"
  EXECUTABLE pid_arm_planner_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)