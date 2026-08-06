cmake_minimum_required(VERSION 3.8)
project(scoop_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Decimated visuals/collision STLs. Hires archives stay in source for
# regeneration (scripts/decimate_stl.py) and are excluded from the install.
install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
  PATTERN "hires" EXCLUDE
)

ament_package()
