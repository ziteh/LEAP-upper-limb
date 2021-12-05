import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class Simple3RArmRelativeController(Node):
    def __init__(self):
        super().__init__("simple_3r_arm_relative_controller")

        self.publisher_ = self.create_publisher(Float64MultiArray, "/ik3r", 10)

        self.fk3r_subscriber = self.create_subscription(
            Float64MultiArray,
            "/fk3r",
            self.fk3r_subscriber_callback,
            10
        )

        self.relative_subscriber = self.create_subscription(
            Float64MultiArray,
            "/s3ra/relative",
            self.relative_subscriber_callback,
            10
        )

    def fk3r_subscriber_callback(self, msg):
        self.position = msg.data

    def relative_subscriber_callback(self, msg):
        pubMsg = Float64MultiArray()
        pubMsg.data = [
            self.position[0] + msg.data[0],
            self.position[1] + msg.data[1],
            self.position[2] + msg.data[2]
        ]
        self.publisher_.publish(pubMsg)

def main(args=None):
    rclpy.init(args=args)
    node = Simple3RArmRelativeController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
