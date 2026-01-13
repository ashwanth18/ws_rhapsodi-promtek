cmake_minimum_required(VERSION 3.8)
project(qr_scanner)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_common_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(ZBAR REQUIRED zbar)
add_library(${PROJECT_NAME} SHARED
  src/scan_qr_server.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} rclcpp rclcpp_action robot_common_msgs sensor_msgs cv_bridge image_transport)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${ZBAR_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS} ${ZBAR_INCLUDE_DIRS})

add_executable(scan_qr_server_node src/scan_qr_server_node.cpp)
target_link_libraries(scan_qr_server_node ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} scan_qr_server_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


