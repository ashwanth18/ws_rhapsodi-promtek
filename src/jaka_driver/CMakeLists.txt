cmake_minimum_required(VERSION 3.8)
project(jaka_driver)

set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(jaka_msgs REQUIRED)
# find_package(moveit_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(control_msgs REQUIRED)

# Optionally find additional dependencies
# find_package(Boost REQUIRED)

# Declare ROS 2 message and service dependencies
find_package(rosidl_default_generators REQUIRED)

# Declare executable targets
add_executable(jaka_driver src/jaka_driver.cpp)
add_executable(sdk_test src/sdk_test.cpp)
add_executable(client src/client.cpp)
add_executable(servoj_demo src/servoj_demo.cpp)

# Link libraries
ament_target_dependencies(jaka_driver
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  jaka_msgs
  # moveit_msgs
  action_msgs
  control_msgs
)
ament_target_dependencies(sdk_test
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  jaka_msgs
  # moveit_msgs
  action_msgs
)
ament_target_dependencies(client
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  jaka_msgs
  # moveit_msgs
  action_msgs
  control_msgs
)
ament_target_dependencies(servoj_demo
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  jaka_msgs
  # moveit_msgs
  action_msgs
)

# For including directories in ROS 2
target_include_directories(jaka_driver
  PUBLIC
    include
  PRIVATE
    ${ament_INCLUDE_DIRS}
)

target_include_directories(client
  PUBLIC
    include
  PRIVATE
    ${ament_INCLUDE_DIRS}
)

target_include_directories(sdk_test
  PUBLIC
    include
  PRIVATE
    ${ament_INCLUDE_DIRS}
)

target_include_directories(servoj_demo
  PUBLIC
    include
  PRIVATE
    ${ament_INCLUDE_DIRS}
)



# Specify where to find external libraries
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(JAKA_API_LIB ${LIBRARY_DIR}/libjakaAPI.so)

# Add the directory to the search path
link_directories(${LIBRARY_DIR})

# Find the library and link it properly
target_link_libraries(jaka_driver ${JAKA_API_LIB})
target_link_libraries(sdk_test ${JAKA_API_LIB})
target_link_libraries(client ${JAKA_API_LIB})
target_link_libraries(servoj_demo ${JAKA_API_LIB})

# So install/ libjaka_driver finds libjakaAPI.so without LD_LIBRARY_PATH hacks.
foreach(_jaka_tgt IN ITEMS jaka_driver sdk_test client servoj_demo)
  set_target_properties(${_jaka_tgt} PROPERTIES
    BUILD_RPATH "${LIBRARY_DIR}"
    INSTALL_RPATH "$ORIGIN/.."
  )
endforeach()

# Install executables
install(TARGETS
  jaka_driver
  sdk_test
  client
  servoj_demo
  DESTINATION lib/${PROJECT_NAME}
)

# Install the vendor SDK shared library next to lib/<pkg>/
install(FILES ${JAKA_API_LIB}
  DESTINATION lib
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
