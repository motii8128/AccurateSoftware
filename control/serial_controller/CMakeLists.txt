cmake_minimum_required(VERSION 3.5)
project(serial_controller)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_serial_controller SHARED
  DIRECTORY src
)

target_include_directories(target_serial_controller
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_serial_controller
  PLUGIN "serial_controller::SerialController"
  EXECUTABLE serial_controller_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)