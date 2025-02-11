cmake_minimum_required(VERSION 3.5)
project(localization_sim)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_localization_sim SHARED
  DIRECTORY src
)
target_include_directories(target_localization_sim
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_localization_sim
  PLUGIN "localization_sim::LocalizationSim"
  EXECUTABLE localization_sim_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)