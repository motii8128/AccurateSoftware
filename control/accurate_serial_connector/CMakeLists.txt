cmake_minimum_required(VERSION 3.5)
project(accurate_serial_connector)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_accurate_serial_connector SHARED
  DIRECTORY src
)
target_include_directories(target_accurate_serial_connector
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_accurate_serial_connector
  PLUGIN "serial_controller::SerialController"
  EXECUTABLE accurate_serial_connector_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)