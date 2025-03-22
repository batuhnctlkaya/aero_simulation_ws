import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from robot_state_publisher import RobotStatePublisher
import os

class RobotStatePublisherNode(Node):
    def __init__(self):
        super().__init__('robot_state_publisher_node')
        
        # URDF dosyasının yolunu belirle
        urdf_path = os.path.join(
            get_package_share_directory('aero_simulation'),
            'urdf',
            'rover.urdf'
        )

        # URDF dosyasını yükle
        with open(urdf_path, 'r') as file:
            robot_description = file.read()

        # Robot State Publisher'ı başlat
        self.robot_state_publisher = RobotStatePublisher(self, robot_description)
        self.timer = self.create_timer(0.02, self.publish_states)  # 50 Hz

    def publish_states(self):
        """Robot durumunu yayınlar."""
        self.robot_state_publisher.publish_transforms()

def main(args=None):
    rclpy.init(args=args)
    node = RobotStatePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
