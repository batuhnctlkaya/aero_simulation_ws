import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoystickControlNode(Node):
    def __init__(self):
        super().__init__('joystick_control_node')
        
        # Parametreleri YAML dosyasından yükle
        self.declare_parameter('axis_linear', 1)
        self.declare_parameter('axis_angular', 0)

        self.axis_linear = self.get_parameter('axis_linear').value
        self.axis_angular = self.get_parameter('axis_angular').value

        # Publisher ve Subscriber
        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def joy_callback(self, msg):
        """Joystick verilerini alır ve hız komutlarını yayınlar."""
        if len(msg.axes) < 2:
            self.get_logger().warn("Joystick verisi eksik! axes boyutu: " + str(len(msg.axes)))
            return

        cmd = Twist()
        cmd.linear.x = msg.axes[self.axis_linear]
        cmd.angular.z = msg.axes[self.axis_angular]

        self.cmd_vel_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
