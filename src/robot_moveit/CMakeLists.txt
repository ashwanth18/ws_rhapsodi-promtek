cmake_minimum_required(VERSION 3.8)
project(robot_moveit)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(robot_common_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
## No interface generation here; services live in robot_common_msgs

add_library(robot_moveit_lib SHARED
  src/move_to_server.cpp
)
target_include_directories(robot_moveit_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(robot_moveit_lib
  rclcpp rclcpp_action geometry_msgs tf2_geometry_msgs moveit_ros_planning_interface moveit_msgs moveit_core moveit_ros_planning robot_common_msgs
)

# yaml-cpp is a non-ament package; link its CMake target explicitly
target_link_libraries(robot_moveit_lib yaml-cpp)

add_executable(move_to_server_node src/move_to_server_node.cpp)
target_link_libraries(move_to_server_node robot_moveit_lib)

add_executable(target_recorder_node src/target_recorder_node.cpp src/target_recorder.cpp)
target_include_directories(target_recorder_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(target_recorder_node rclcpp geometry_msgs moveit_ros_planning_interface robot_common_msgs)
ament_target_dependencies(target_recorder_node tf2_ros tf2_geometry_msgs)
target_link_libraries(target_recorder_node yaml-cpp)


install(TARGETS robot_moveit_lib move_to_server_node target_recorder_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ config/
  DESTINATION share/${PROJECT_NAME}
)

# No interfaces generated here; RecordTarget moved to robot_common_msgs

ament_export_dependencies(rosidl_default_runtime)

ament_package()


