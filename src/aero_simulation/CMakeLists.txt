cmake_minimum_required(VERSION 3.8)
project(aero_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)


# Install Python nodes
install(
  PROGRAMS
    scripts/joystick_control_node.py
    scripts/rover_control_node.py
    scripts/camera_node.py
    scripts/robot_state_publisher_node.py
    scripts/joint_state_publisher_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install resources
install(
  DIRECTORY urdf launch meshes config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()