cmake_minimum_required(VERSION 3.8)
project(aero_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(std_msgs REQUIRED)

# Add C++ nodes
add_executable(joystick_control_node src/joystick_control_node.cpp)
ament_target_dependencies(joystick_control_node rclcpp std_msgs)

add_executable(rover_control_node src/rover_control_node.cpp)
ament_target_dependencies(rover_control_node rclcpp std_msgs)

add_executable(camera_node src/camera_node.cpp)
ament_target_dependencies(camera_node rclcpp std_msgs)

add_executable(robot_state_publisher_node src/robot_state_publisher_node.cpp)
ament_target_dependencies(robot_state_publisher_node rclcpp urdf)

add_executable(joint_state_publisher_node src/joint_state_publisher_node.cpp)
ament_target_dependencies(joint_state_publisher_node rclcpp std_msgs)

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