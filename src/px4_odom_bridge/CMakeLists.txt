cmake_minimum_required(VERSION 3.8)
project(px4_odom_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(px4_odom_bridge src/px4_odom_bridge.cpp)
ament_target_dependencies(px4_odom_bridge
  rclcpp
  nav_msgs
  px4_msgs
)

install(TARGETS px4_odom_bridge
  DESTINATION lib/${PROJECT_NAME})

ament_package()
