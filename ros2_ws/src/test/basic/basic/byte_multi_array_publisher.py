import rclpy
import time
from rclpy import publisher
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray

class BasicByteMultiArrayPublisher(Node):
    def __init__(self):
        super().__init__("byte_multi_array_publisher")

        self.topicName = "basic_msg/byte_multi_array" 
        self.get_logger().debug(f'Topic: {self.topicName}')
        self.publisher_ = self.create_publisher(ByteMultiArray,
                                                self.topicName,
                                                1)

        pubMsg = ByteMultiArray()
        pubMsg.data = [bytes([0]), bytes([1]), bytes([2])] 

        while 1:
            self.publisher_.publish(pubMsg)
            time.sleep(1)

def main(args=None):
    rclpy.init(args=args)

    pub  = BasicByteMultiArrayPublisher()
    rclpy.spin(pub)
    pub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
