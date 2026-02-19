cmake_minimum_required(VERSION 3.8)
project(object_detection_msgs)

# 1. 查找依赖包
# ament_cmake 替代 catkin
find_package(ament_cmake REQUIRED)
# rosidl_default_generators 替代 message_generation
find_package(rosidl_default_generators REQUIRED)
# 消息依赖
find_package(std_msgs REQUIRED)

# 2. 生成消息接口
# 在 ROS 2 中，不再分 add_message_files 和 generate_messages，而是合并为一个宏
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoundingBox.msg"
  "msg/BoundingBoxes.msg"
  "msg/ObjectCount.msg"
  "msg/CarPosition.msg"
  "msg/motorAngle.msg"
  DEPENDENCIES std_msgs
)

# 3. 导出包信息
ament_package()
