cmake_minimum_required(VERSION 3.8)
project(decomp_ros_utils)

# ROS 2 推荐使用 C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED) # Rviz 渲染相关
find_package(decomp_ros_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

# Qt MOC (Meta-Object Compiler) 自动处理
set(CMAKE_AUTOMOC ON)

include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
)

# 定义库文件 (Rviz 插件)
add_library(decomp_rviz_plugins SHARED
  src/bound_visual.cpp
  src/mesh_visual.cpp
  src/vector_visual.cpp
  src/ellipsoid_array_visual.cpp
  src/ellipsoid_array_display.cpp
  src/polyhedron_array_display.cpp
)

# 链接库
target_link_libraries(decomp_rviz_plugins
  decomp_ros_msgs::decomp_ros_msgs
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  Qt5::Widgets
  rclcpp::rclcpp
)

# 确保头文件包含正确
target_include_directories(decomp_rviz_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# --- 导出 Rviz 插件 ---
# 这替代了 ROS 1 package.xml 中的 <export> 标签逻辑的一部分
# 注意：第一个参数必须是 rviz_common (因为它是加载器)，第二个是 xml 文件
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# --- 安装规则 ---

install(TARGETS decomp_rviz_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

# --- 导出依赖 ---
ament_export_include_directories(include)
ament_export_libraries(decomp_rviz_plugins)
ament_export_dependencies(rclcpp rviz_common decomp_ros_msgs)

ament_package()
