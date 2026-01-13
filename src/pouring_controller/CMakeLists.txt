cmake_minimum_required(VERSION 3.8)
project(pouring_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(robot_common_msgs REQUIRED)
find_package(pluginlib REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/pour_server.cpp
  src/pid_vibration.cpp
  src/bangbang_trickle.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp rclcpp_action std_msgs control_msgs trajectory_msgs robot_common_msgs pluginlib
)

add_executable(pour_server_node src/pour_server_node.cpp)
target_link_libraries(pour_server_node ${PROJECT_NAME})
ament_target_dependencies(pour_server_node rclcpp)

add_executable(pour_client_demo src/pour_client_demo.cpp)
ament_target_dependencies(pour_client_demo rclcpp rclcpp_action robot_common_msgs)

install(TARGETS ${PROJECT_NAME} pour_server_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS pour_client_demo
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()


