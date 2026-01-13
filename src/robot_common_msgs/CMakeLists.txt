cmake_minimum_required(VERSION 3.8)
project(robot_common_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ContainerSpec.msg"
  "msg/SystemStatus.msg"
  "msg/PourStatus.msg"
  "action/MoveTo.action"
  "action/ScanQr.action"
  "action/Scoop.action"
  "action/PourToTarget.action"
  "srv/StartBatch.srv"
  "srv/RecordTarget.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
