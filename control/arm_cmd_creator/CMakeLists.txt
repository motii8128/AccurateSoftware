cmake_minimum_required(VERSION 3.5)
project(arm_cmd_creator)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_arm_cmd_creator SHARED
  DIRECTORY src
)

target_include_directories(target_arm_cmd_creator
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_arm_cmd_creator
  PLUGIN "arm_cmd_creator::ArmCmdCreator"
  EXECUTABLE arm_cmd_creator_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)