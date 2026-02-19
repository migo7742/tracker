cmake_minimum_required(VERSION 3.8)
project(target_ekf)

# 设置 C++ 标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(object_detection_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED) # 推荐引入，用于处理四元数转换

# 包含目录
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# 1. target_ekf_node
add_executable(target_ekf_node src/target_ekf_node.cpp)
ament_target_dependencies(target_ekf_node
  rclcpp
  nav_msgs
  object_detection_msgs
  message_filters
  tf2_eigen
)
# Eigen 需要单独链接包含路径（有些系统上 ament 不自动处理 Eigen include）
target_include_directories(target_ekf_node PUBLIC ${EIGEN3_INCLUDE_DIR})

# 2. target_ekf_sim_node
add_executable(target_ekf_sim_node src/target_ekf_sim_node.cpp)
ament_target_dependencies(target_ekf_sim_node
  rclcpp
  nav_msgs
  message_filters
  tf2_eigen
)
target_include_directories(target_ekf_sim_node PUBLIC ${EIGEN3_INCLUDE_DIR})

# 安装规则
install(TARGETS
  target_ekf_node
  target_ekf_sim_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装 config 文件
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
