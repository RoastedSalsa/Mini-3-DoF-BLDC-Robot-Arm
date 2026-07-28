cmake_minimum_required(VERSION 3.8)
project(mini_ranka_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Guards the CAD-export -> firmware-convention rework in urdf/mini_ranka.urdf
  # against a silently flipped axis or a drifted link length.
  ament_add_pytest_test(test_kinematics test/test_kinematics.py)
endif()

ament_package()
