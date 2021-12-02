import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray

class SendingTestDataPackage(Node):
    def __init__(self):
        super().__init__("sending_test_data_package")

        self.declare_parameter("data", [0x8f])
        self.declare_parameter("topic", "serialPort/write/dev/ttyACM0")

        self.data = self.get_parameter("data").value
        self.topic = self.get_parameter("topic").value

        self.get_logger().debug(f'Topic: {self.topic}')
        self.publisher_ = self.create_publisher(ByteMultiArray, self.topic, 1)

        msg = ByteMultiArray();
        for d in self.data:
            msg.data.append(bytes([d]))

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = SendingTestDataPackage()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
