cmake_minimum_required(VERSION 3.8)
project(full_assembly)

find_package(ament_cmake REQUIRED)

# The URDF refers to meshes as package://full_assembly/meshes/*.stl, so the
# directory layout below has to be preserved verbatim in the install space.
install(
  DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
