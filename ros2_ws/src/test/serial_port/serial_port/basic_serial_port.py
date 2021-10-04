import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class SerialPortSubscriber(Node):
    def __init__(self):
        super().__init__("serial_port_subscriber")

        self.declare_parameter("port", "dev/ttyACM0")
        self.declare_parameter("baudrate", "9600")

        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").value
        self.topicName = "serialPort"

        self.get_logger().info(f'Port: {self.port}, Baudrate: {self.baudrate}, Topic: {self.topicName}')

        self.subscriber = self.create_subscription(
            Float64MultiArray,
            self.topicName,
            self.subsrciber_callback,
            10
        )

    def subsrciber_callback(self, msg):
        ser = serial.Serial()
        ser.baudrate = self.baudrate
        ser.port = self.port
        ser.open()
        ser.write(b'hello')
        ser.close()
    
def main(args=None):
    rclpy.init(args=args)

    subscriber = SerialPortSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
