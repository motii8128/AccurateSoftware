cmake_minimum_required(VERSION 3.5)
project(near_point_remover)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_near_point_remover SHARED
  DIRECTORY src
)
target_include_directories(target_near_point_remover
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_near_point_remover
  PLUGIN "near_point_remover::NearPointRemover"
  EXECUTABLE near_point_remover_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)