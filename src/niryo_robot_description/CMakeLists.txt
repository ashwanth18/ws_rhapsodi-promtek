cmake_minimum_required(VERSION 3.8)
project(niryo_robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install only the meshes needed by the Ned3pro (and shared gripper/led)
# production URDF. The unused ned/ned2/one/d435 trees add ~100 MB and are
# excluded from the install image. COLLADA and hires STL archives stay in
# source for regeneration (scripts/decimate_ned3pro_meshes.py).
install(
  DIRECTORY
    meshes/ned3pro/
    meshes/gripper_1/
    meshes/led_ring/
  DESTINATION share/${PROJECT_NAME}/meshes
  PATTERN "*.dae" EXCLUDE
  PATTERN "hires" EXCLUDE
)

install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

ament_package()
