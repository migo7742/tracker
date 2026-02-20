cmake_minimum_required(VERSION 3.8)
project(decomp_ros_utils)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(decomp_ros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# decomp_ros_utils 主要是头文件工具库（data_ros_utils.h 等），
# rviz 插件部分因 ROS2 Humble 中 rviz_rendering::MeshShape 不可用而暂时跳过。

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  decomp_ros_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
)

ament_package()
