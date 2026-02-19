cmake_minimum_required(VERSION 3.8)
project(traj_opt)

# ROS 2 默认推荐 C++17，但也兼容 C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

# 包含目录
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
)

# 定义库
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}.cc
  src/${PROJECT_NAME}_fake.cc
)

# 链接 ROS 2 依赖
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
)

# 确保库能找到自己的头文件 (对于 install 后的引用很重要)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Eigen3_INCLUDE_DIRS}
)

# --- 安装与导出 ---

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 安装库文件
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 导出依赖 (供下游包 find_package 使用)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp)

ament_package()
