cmake_minimum_required(VERSION 3.22)
project(pan_tilt)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PanSweep.action"
  "action/TiltSweep.action"
  "action/ScanSequence.action"
  "action/MoveToAngles.action"
)

# Install action server executable
install(
  PROGRAMS
    pan_tilt/PanTiltActionServer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pan_tilt_action_server
)

# Install URDF
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install rviz files
install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

