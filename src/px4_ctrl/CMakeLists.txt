cmake_minimum_required(VERSION 3.8)
project(px4_ctrl)

# 建议明确指定 C++14 或 C++17，ROS 2 常用 C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# 如果你的 include 目录下没有头文件，这行可以注释掉，但保留也没事
include_directories(include)

# 编译节点
add_executable(offboard_control_node src/offboard_control.cpp)

# 设置 ROS 依赖
ament_target_dependencies(offboard_control_node 
  rclcpp 
  px4_msgs 
  quadrotor_msgs
  std_msgs
  nav_msgs
)

# 链接 Eigen (使用现代 CMake 写法)
target_link_libraries(offboard_control_node Eigen3::Eigen)

# ---------------------------------------------------------
# [关键修改] 安装规则
# ---------------------------------------------------------

# 1. 安装编译好的可执行文件
install(TARGETS
  offboard_control_node
  DESTINATION lib/${PROJECT_NAME})

# 2. [新增] 安装 launch 和 config 目录到 share 路径
# 只有加了这几行，ros2 launch 才能找到文件
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
