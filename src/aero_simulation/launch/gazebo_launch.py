from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # YAML dosyalarının yollarını belirle
    config_dir = os.path.join(get_package_share_directory('aero_simulation'), 'config')

    joystick_params = os.path.join(config_dir, 'joystick_params.yaml')
    rover_control_params = os.path.join(config_dir, 'rover_control_params.yaml')
    camera_params = os.path.join(config_dir, 'camera_params.yaml')
    robot_state_params = os.path.join(config_dir, 'robot_state_params.yaml')
    joint_state_params = os.path.join(config_dir, 'joint_state_params.yaml')

    # URDF dosyasının yolunu belirle
    urdf_path = os.path.join(
        get_package_share_directory('aero_simulation'),
        'urdf',
        'rover.urdf'
    )

    return LaunchDescription([
        # Joystick Kontrol Node'u
        Node(
            package='aero_simulation',
            executable='joystick_control_node',
            name='joystick_control_node',
            output='screen',
            parameters=[joystick_params]
        ),

        # Rover Kontrol Node'u
        Node(
            package='aero_simulation',
            executable='rover_control_node',
            name='rover_control_node',
            output='screen',
            parameters=[rover_control_params]
        ),

        # Kamera Node'u
        Node(
            package='aero_simulation',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[camera_params]
        ),

        # Robot State Publisher Node'u
        Node(
            package='aero_simulation',
            executable='robot_state_publisher_node',
            name='robot_state_publisher_node',
            output='screen',
            parameters=[{'robot_description': open(urdf_path).read()}]
        ),

        # Joint State Publisher Node'u
        Node(
            package='aero_simulation',
            executable='joint_state_publisher_node',
            name='joint_state_publisher_node',
            output='screen',
            parameters=[joint_state_params]
        ),
    ])
