cmake_minimum_required(VERSION 3.8)
project(scooping_controller)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_common_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(scooping_marker_server src/interactive_marker_server.cpp)
target_compile_features(scooping_marker_server PUBLIC cxx_std_17)
target_include_directories(scooping_marker_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  scooping_marker_server
  geometric_shapes
  geometry_msgs
  interactive_markers
  moveit_msgs
  rclcpp
  shape_msgs
  std_msgs
  std_srvs
  tf2
  visualization_msgs
)
target_link_libraries(scooping_marker_server yaml-cpp)

add_library(scooping_rviz_panel SHARED
  src/scooping_panel.cpp
  include/scooping_controller/scooping_panel.hpp
)
target_compile_features(scooping_rviz_panel PUBLIC cxx_std_17)
target_include_directories(scooping_rviz_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  scooping_rviz_panel
  geometry_msgs
  pluginlib
  rclcpp
  rclcpp_action
  robot_common_msgs
  rviz_common
  std_msgs
  std_srvs
  tf2
  visualization_msgs
)
target_link_libraries(scooping_rviz_panel Qt5::Widgets yaml-cpp)

add_executable(scooping_mtc_node src/scooping_mtc_node.cpp)
target_compile_features(scooping_mtc_node PUBLIC cxx_std_17)
target_include_directories(scooping_mtc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  scooping_mtc_node
  ament_index_cpp
  control_msgs
  geometric_shapes
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  rclcpp
  rclcpp_action
  robot_common_msgs
  shape_msgs
  std_msgs
  std_srvs
)

add_executable(container_marker_publisher src/container_marker_publisher.cpp)
target_compile_features(container_marker_publisher PUBLIC cxx_std_17)
target_include_directories(container_marker_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(
  container_marker_publisher
  geometric_shapes
  moveit_msgs
  rclcpp
  shape_msgs
  visualization_msgs
)

add_executable(planning_scene_collision_publisher src/planning_scene_collision_publisher.cpp)
target_compile_features(planning_scene_collision_publisher PUBLIC cxx_std_17)
target_include_directories(planning_scene_collision_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  planning_scene_collision_publisher
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  shape_msgs
  geometric_shapes
)

add_executable(scooping_task_frame_publisher src/scooping_task_frame_publisher.cpp)
target_compile_features(scooping_task_frame_publisher PUBLIC cxx_std_17)
target_include_directories(scooping_task_frame_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  scooping_task_frame_publisher
  geometric_shapes
  geometry_msgs
  moveit_msgs
  rclcpp
  shape_msgs
  tf2_ros
)

install(
  TARGETS
    scooping_marker_server
    scooping_mtc_node
    container_marker_publisher
    planning_scene_collision_publisher
    scooping_task_frame_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
    scooping_rviz_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(
  DIRECTORY
    launch
    config
    models
    worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES scooping_rviz_plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(rviz_common scooping_rviz_plugin_description.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
