cmake_minimum_required(VERSION 3.5)
project(power_smoother)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_power_smoother SHARED
  DIRECTORY src
)
target_include_directories(target_power_smoother
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_power_smoother
  PLUGIN "power_smoother::PowerSmoother"
  EXECUTABLE power_smoother_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)