cmake_minimum_required(VERSION 3.8)
project(niryo_mtc_demos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(alternative_path_costs src/alternative_path_costs.cpp)
target_include_directories(alternative_path_costs PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(alternative_path_costs PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  alternative_path_costs
  generate_parameter_library
  moveit_core
  moveit_ros_planning_interface
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  rclcpp
)

install(TARGETS alternative_path_costs
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/rviz)

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
