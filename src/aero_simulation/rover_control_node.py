import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class PIDController:
    """PID kontrol algoritması"""
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, setpoint, measured_value):
        """PID hesaplamasını yapar"""
        error = setpoint - measured_value
        self.integral += error
        derivative = error - self.prev_error
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.prev_error = error
        return output

class RoverControlNode(Node):
    def __init__(self):
        super().__init__('rover_control_node')
        
        # Parametreleri YAML dosyasından yükle
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.1)
        self.declare_parameter('kd', 0.05)

        self.pid_left = PIDController(
            kp=self.get_parameter('kp').value,
            ki=self.get_parameter('ki').value,
            kd=self.get_parameter('kd').value
        )
        self.pid_right = PIDController(
            kp=self.get_parameter('kp').value,
            ki=self.get_parameter('ki').value,
            kd=self.get_parameter('kd').value
        )

        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        except serial.SerialException as e:
            self.get_logger().error(f"Seri port açma hatası: {e}")
            return

        self.encoder_value_left = 0
        self.encoder_value_right = 0
        self.target_speed_left = 0
        self.target_speed_right = 0
        self.timer = self.create_timer(0.1, self.update_motor_speed)

    def cmd_vel_callback(self, msg):
        """Joystick'ten gelen hızı günceller"""
        linear_speed = msg.linear.x  # İleri/geri hız
        angular_speed = msg.angular.z  # Dönüş hızı

        # Tank tipi sürüş için sağ ve sol motor hızlarını hesapla
        self.target_speed_left = linear_speed - angular_speed
        self.target_speed_right = linear_speed + angular_speed

    def read_encoder(self):
        """Arduino'dan encoder verisini alır"""
        try:
            self.serial_port.write(b'R\n')
            line = self.serial_port.readline().decode().strip()
            
            if line:
                values = line.split()
                if len(values) == 2:
                    self.encoder_value_left = float(values[0])
                    self.encoder_value_right = float(values[1])
                else:
                    self.get_logger().warn("Hatalı encoder verisi alındı!")
            else:
                self.get_logger().warn("Boş encoder verisi alındı!")
        except Exception as e:
            self.get_logger().error(f"Seri port okuma hatası: {e}")

    def update_motor_speed(self):
        """PID kontrolü ile motor hızlarını günceller"""
        self.read_encoder()

        # PID kontrolü ile motor hızlarını düzelt
        control_signal_left = self.pid_left.compute(self.target_speed_left, self.encoder_value_left)
        control_signal_right = self.pid_right.compute(self.target_speed_right, self.encoder_value_right)

        # Motor hızlarını Arduino'ya gönder
        command = f'L {control_signal_left} R {control_signal_right}\n'

        try:
            self.serial_port.write(command.encode())
        except Exception as e:
            self.get_logger().error(f"Seri port yazma hatası: {e}")

    def destroy(self):
        """Düğüm kapatıldığında seri portu kapatır"""
        if self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    rover_control_node = RoverControlNode()
    rclpy.spin(rover_control_node)
    rover_control_node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
