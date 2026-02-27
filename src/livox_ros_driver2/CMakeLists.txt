cmake_minimum_required(VERSION 3.14)
project(livox_ros_driver2)

# 1. 寻找生成消息所需的最小依赖
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# 2. 定义要生成的 .msg 文件
# 请确保你的 msg/ 目录下有这两个文件
set(msg_files
  "msg/CustomPoint.msg"
  "msg/CustomMsg.msg"
)

# 3. 生成消息接口
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs builtin_interfaces
  ADD_LINTER_TESTS
)

# 4. 导出包信息
ament_package()
