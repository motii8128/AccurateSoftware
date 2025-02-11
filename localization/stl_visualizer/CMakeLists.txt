cmake_minimum_required(VERSION 3.5)
project(stl_visualizer)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_stl_visualizer SHARED
  DIRECTORY src
)
target_include_directories(target_stl_visualizer
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_stl_visualizer
  PLUGIN "stl_visualizer::StlVisualizer"
  EXECUTABLE stl_visualizer_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)