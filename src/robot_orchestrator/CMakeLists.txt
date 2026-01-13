cmake_minimum_required(VERSION 3.8)
project(robot_orchestrator)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_common_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_srvs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/pour_to_target_node.cpp
  src/move_to_node.cpp
  src/scan_qr_node.cpp
  src/register_nodes.cpp
  src/has_active_container_node.cpp
  src/load_next_container_node.cpp
  src/compute_remaining_node.cpp
  src/fetch_batch_node.cpp
  src/project_container_node.cpp
  # header-only vibrate_node.hpp is included via register; no cpp needed
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp rclcpp_action robot_common_msgs ament_index_cpp behaviortree_cpp behaviortree_ros2 std_srvs
)

target_link_libraries(${PROJECT_NAME}
  behaviortree_cpp::behaviortree_cpp
  behaviortree_ros2::behaviortree_ros2
)

add_executable(orchestrator_node src/orchestrator_main.cpp)
target_link_libraries(orchestrator_node ${PROJECT_NAME} behaviortree_cpp::behaviortree_cpp behaviortree_ros2::behaviortree_ros2)
ament_target_dependencies(orchestrator_node rclcpp ament_index_cpp std_srvs)

install(TARGETS ${PROJECT_NAME} orchestrator_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION share/${PROJECT_NAME}
)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/bt_trees)
  install(DIRECTORY bt_trees/
    DESTINATION share/${PROJECT_NAME}/bt_trees
  )
endif()

ament_package()


