cmake_minimum_required(VERSION 3.5)
project(accurate_localization)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_accurate_localization SHARED
  DIRECTORY src
)
target_include_directories(target_accurate_localization
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_accurate_localization
  PLUGIN "accurate_localization::AccurateLocalization"
  EXECUTABLE accurate_localization_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)