import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
from std_msgs.msg import Float64MultiArray

class ForceSensorValueDecoder(Node):
    def __init__(self):
        super().__init__("force_sensor_value_decoder")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.port = self.get_parameter("port").value

        self.pubTopic = "s3ra/relative" 
        self.subTopic = f"serialPort/read{self.port}" 

        self.publisher_ = self.create_publisher(Float64MultiArray,
                                                self.pubTopic,
                                                1)

        self.subscriber = self.create_subscription(
            ByteMultiArray,
            self.subTopic,
            self.subsrciber_callback,
            10
        )

    def subsrciber_callback(self, subMsd):
        self.get_logger().debug(f'Get: {subMsd}')
        if subMsd.data[0] == b'\x8d':
            id = subMsd.data[1][0] & 0x07
            xValue = (subMsd.data[2][0] & 0x3f) + ((subMsd.data[3][0] & 0x3f) << 6)
            yValue = (subMsd.data[4][0] & 0x3f) + ((subMsd.data[5][0] & 0x3f) << 6)
            zValue = (subMsd.data[6][0] & 0x3f) + ((subMsd.data[7][0] & 0x3f) << 6)

            xRelative = 0.0
            if xValue > 1900:
                xRelative = 0.05
            elif xValue < 1200:
                xRelative = -0.05

            yRelative = 0.0
            if yValue > 1900:
                yRelative = 0.05
            elif yValue < 1200:
                yRelative = -0.05

            zRelative = 0.0
            if zValue > 1900:
                zRelative = 0.05
            elif zValue < 1200:
                zRelative = -0.05
            
            pubMsg = Float64MultiArray()
            pubMsg.data = [float(xRelative), 0.0, 0.0]
            self.publisher_.publish(pubMsg)

def main(args=None):
    rclpy.init(args=args)

    node  = ForceSensorValueDecoder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
