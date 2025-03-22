import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStatePublisherNode(Node):
    def __init__(self):
        super().__init__('joint_state_publisher_node')
        
        # Parametreleri YAML dosyasından yükle
        self.declare_parameter('source_list', ['/joint_states'])
        self.declare_parameter('publish_frequency', 50.0)

        self.source_list = self.get_parameter('source_list').value
        self.publish_frequency = self.get_parameter('publish_frequency').value

        # Publisher ve Timer
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(1.0 / self.publish_frequency, self.publish_joint_states)

    def publish_joint_states(self):
        """Eklem durumlarını yayınlar."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2']  # Eklem isimleri
        msg.position = [0.0, 0.0]  # Eklem pozisyonları
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
