cmake_minimum_required(VERSION 3.5)
project(color_board_connector)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_color_board_connector SHARED
  DIRECTORY src
)
target_include_directories(target_color_board_connector
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_color_board_connector
  PLUGIN "color_board_connector::ColorBoardConnector"
  EXECUTABLE color_board_connector_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)