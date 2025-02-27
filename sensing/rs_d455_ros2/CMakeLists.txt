cmake_minimum_required(VERSION 3.5)
project(rs_d455_ros2)

find_package(ament_cmake_auto REQUIRED)
find_package(realsense2 REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_rs_d455_ros2 SHARED
  DIRECTORY src
)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)
target_include_directories(target_rs_d455_ros2
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_rs_d455_ros2
  PLUGIN "rs_d455_ros2::RealSenseD455_ROS2"
  EXECUTABLE rs_d455_ros2_node
)
target_link_libraries(target_rs_d455_ros2 
  ${OpenCV_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${realsense2_LIBRARY}
)

ament_auto_package(
  INSTALL_TO_SHARE
)
