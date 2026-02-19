cmake_minimum_required(VERSION 3.8)
project(mapping)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# 假设 pose_utils 和 object_detection_msgs 已经迁移并安装
find_package(pose_utils REQUIRED) 
find_package(object_detection_msgs REQUIRED)

# System dependencies
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# ---------------------------------------------------------
# 1. Core Library (mapping.cc)
# ---------------------------------------------------------
add_library(${PROJECT_NAME}_lib SHARED
  src/${PROJECT_NAME}.cc
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  sensor_msgs
  pcl_conversions
  Eigen3
)

# ---------------------------------------------------------
# 2. Component (formerly Nodelet)
# ---------------------------------------------------------
# 编译为一个共享库，供容器加载
add_library(${PROJECT_NAME}_component SHARED
  src/${PROJECT_NAME}_nodelet.cpp
)
target_link_libraries(${PROJECT_NAME}_component
  ${PROJECT_NAME}_lib
  ${OpenCV_LIBS}
)
ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  rclcpp_components
  cv_bridge
  quadrotor_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  pose_utils
  object_detection_msgs
)

# 注册组件 (替代了 nodelet_plugin.xml)
rclcpp_components_register_nodes(${PROJECT_NAME}_component "mapping::MappingNode")

# ---------------------------------------------------------
# 3. Visualization Node
# ---------------------------------------------------------
add_executable(${PROJECT_NAME}_vis_node
  src/${PROJECT_NAME}_vis_node.cpp
)
target_link_libraries(${PROJECT_NAME}_vis_node
  ${PROJECT_NAME}_lib
)
ament_target_dependencies(${PROJECT_NAME}_vis_node
  rclcpp
  sensor_msgs
  quadrotor_msgs
)

# ---------------------------------------------------------
# 4. History Path Visualizer
# ---------------------------------------------------------
add_executable(visualize_history_path
  src/visualize_history_path.cpp
)
ament_target_dependencies(visualize_history_path
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  quadrotor_msgs
  Eigen3
)

# ---------------------------------------------------------
# Install
# ---------------------------------------------------------
install(TARGETS 
  ${PROJECT_NAME}_lib
  ${PROJECT_NAME}_component
  ${PROJECT_NAME}_vis_node
  visualize_history_path
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
