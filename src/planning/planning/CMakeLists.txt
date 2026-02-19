cmake_minimum_required(VERSION 3.8)
project(planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 default is C++14, but C++17 is often preferred/required for modern libraries
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Custom packages (Assuming these are ported to ROS 2)
find_package(quadrotor_msgs REQUIRED)
find_package(traj_opt REQUIRED)
find_package(mapping REQUIRED)
find_package(decomp_ros_utils REQUIRED)

find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# ---------------------------------------------------------
# Planning Component (formerly Nodelet)
# ---------------------------------------------------------
add_library(${PROJECT_NAME}_component SHARED
  src/planning_nodelet.cpp
)

ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  pcl_conversions
  ament_index_cpp
  quadrotor_msgs
  traj_opt
  mapping
  decomp_ros_utils
  Eigen3
)

# Register the component so it can be loaded into a container
rclcpp_components_register_nodes(${PROJECT_NAME}_component "planning::PlanningNode")

# ---------------------------------------------------------
# Trajectory Server
# ---------------------------------------------------------
add_executable(traj_server src/traj_server.cpp)
ament_target_dependencies(traj_server
  rclcpp
  std_msgs
  nav_msgs
  quadrotor_msgs
  visualization_msgs
  traj_opt
  Eigen3
)

# ---------------------------------------------------------
# Play Bag Node
# ---------------------------------------------------------
add_executable(play_bag_node src/play_bag_node.cpp)
ament_target_dependencies(play_bag_node
  rclcpp
  quadrotor_msgs
  sensor_msgs
  mapping
  ament_index_cpp
)

# ---------------------------------------------------------
# Test Node
# ---------------------------------------------------------
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/test_node.cpp")
  add_executable(test_node src/test_node.cpp)
  ament_target_dependencies(test_node
    rclcpp
    quadrotor_msgs
    # Add other dependencies if test_node uses them
  )
endif()

# ---------------------------------------------------------
# Install / Export
# ---------------------------------------------------------

# Export include directories for other packages
ament_export_include_directories(include)

# Install libraries (component) and executables
install(TARGETS
  ${PROJECT_NAME}_component
  traj_server
  play_bag_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(TARGET test_node)
  install(TARGETS test_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files (if any)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.py" PATTERN "*.xml"
)

ament_package()
