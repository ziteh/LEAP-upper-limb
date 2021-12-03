import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
from std_msgs.msg import UInt8

class MotorPositionControlSender(Node):
    def __init__(self):
        super().__init__("motor_position_control_sender")

        self.declare_parameter("serialPort", "/dev/ttyACM0")
        self.serialPortName = self.get_parameter("serialPort").value

        self.pubTopic = f"serialPort/write{self.serialPortName}"
        self.subTopic = "motor/positionControl"

        self.get_logger().info(f'Data package topic: {self.subTopic}')
        self.get_logger().info(f'Serial port topic: {self.pubTopic}')

        self.subscriber = self.create_subscription(
            UInt8,
            self.subTopic,
            self.subsrciber_callback,
            10
        )

        self.publisher_ = self.create_publisher(ByteMultiArray, 
                                                self.pubTopic,
                                                10)


    def subsrciber_callback(self, msg):
        self.get_logger().info(f'Get: {msg}')
        if(msg.data > 100):
            msg.data = 100
        position = msg.data * (4095 / 100)
        p1 = int(position) & 0x3f
        p2 = (int(position) >> 6) & 0x3f
        self.publisher_.publish(bytes([0, p1, p2]))

def main(args=None):
    rclpy.init(args=args)

    node = MotorPositionControlSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
