cmake_minimum_required(VERSION 3.8)
project(decomp_ros_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 显式列出文件名，不要使用 GLOB，不要加绝对路径
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Ellipsoid.msg"
  "msg/EllipsoidArray.msg"
  "msg/Polyhedron.msg"
  "msg/PolyhedronArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
