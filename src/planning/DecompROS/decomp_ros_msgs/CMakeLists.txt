cmake_minimum_required(VERSION 3.8)
project(decomp_ros_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 查找依赖
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 2. 查找消息文件
# 原版 catkin_simple 会自动查找 msg 目录下的文件。
# 为了保持方便，这里使用 GLOB 自动查找所有 .msg 文件。
# (注意：在严格的工程实践中，建议手动列出文件名以避免重新编译时的潜在问题，但在迁移时这样最快)
file(GLOB MSG_FILES "msg/*.msg")

if(NOT MSG_FILES)
  message(FATAL_ERROR "No .msg files found in msg/ directory!")
endif()

# 3. 生成消息代码
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES geometry_msgs
)

# 4. 导出依赖
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(geometry_msgs)

ament_package()
