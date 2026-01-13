cmake_minimum_required(VERSION 3.8)
project(niryo_robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(warehouse_ros_sqlite REQUIRED)
find_package(moveit_ros_warehouse REQUIRED)

# Include directories
include_directories(include)

# C++ executable
add_executable(moveit_control_node_cpp src/moveit_control_node.cpp)
ament_target_dependencies(moveit_control_node_cpp
  rclcpp
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
  trajectory_msgs
  tf2_ros
  tf2_geometry_msgs
  std_srvs
  warehouse_ros_sqlite
  moveit_ros_warehouse
)

# add_executable(moveit_warehouse_test_node_cpp src/moveit_warehouse_test_node.cpp)
# ament_target_dependencies(moveit_warehouse_test_node_cpp  
#   rclcpp
#   moveit_ros_planning_interface
#   moveit_ros_planning
#   moveit_msgs
#   geometry_msgs
#   std_msgs
#   sensor_msgs
#   trajectory_msgs
#   tf2_ros
#   tf2_geometry_msgs
#   std_srvs
#   warehouse_ros_sqlite
#   moveit_ros_warehouse
# )
install(TARGETS moveit_control_node_cpp  DESTINATION lib/${PROJECT_NAME})

# Python package
ament_python_install_package(${PROJECT_NAME})

# Launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
# Config files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()