import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Byte

class SendingTestDataPackage(Node):
    def __init__(self):
        super().__init__("sending_test_data_package")

        self.declare_parameter("id", 0)
        self.declare_parameter("topic", "serialPort/write/dev/ttyACM0")

        self.id = self.get_parameter("id").value
        self.topic = self.get_parameter("topic").value

        self.get_logger().debug(f'Topic: {self.topic}')
        self.publisher_ = self.create_publisher(Byte, self.topic, 1)

        msg = Byte();
        msg.data = bytes([0x87])
        self.publisher_.publish(msg)

        msg.data = bytes([self.id])
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = SendingTestDataPackage()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
