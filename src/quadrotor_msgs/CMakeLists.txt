cmake_minimum_required(VERSION 3.8)
project(quadrotor_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 寻找依赖包
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 定义消息文件列表
set(msg_files
  "msg/OccMap3d.msg"
  "msg/PolyTraj.msg"
  "msg/ReplanState.msg"
  "msg/PositionCommand.msg"
  "msg/SO3Command.msg"
  "msg/AuxCommand.msg"
  "msg/Px4ctrlDebug.msg"
  "msg/TakeoffLand.msg"
)

# 生成 ROS 2 接口
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs builtin_interfaces geometry_msgs nav_msgs
)

# 导出依赖 (供下游包使用)
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(std_msgs)
ament_export_dependencies(builtin_interfaces)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)

ament_package()
