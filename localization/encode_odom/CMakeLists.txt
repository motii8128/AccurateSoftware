cmake_minimum_required(VERSION 3.5)
project(encode_odom)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_encode_odom SHARED
  DIRECTORY src
)
target_include_directories(target_encode_odom
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_encode_odom
  PLUGIN "encode_odom::EncodeOdometer"
  EXECUTABLE encode_odom_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)