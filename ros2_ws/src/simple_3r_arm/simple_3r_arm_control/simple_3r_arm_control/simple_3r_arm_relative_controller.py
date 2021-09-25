import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from simple_3r_arm_interface.srv import FK

class Simple3RArmRelativeController(Node):
    def __init__(self):
        super().__init__("simple_3r_arm_relative_controller")

        self.declare_parameter("controller_name","forward_position_controller")
        self.controller_name = self.get_parameter("controller_name").value

        topic = "/s3ra/relative"
        self.subscriber = self.create_subscription(
            Float64MultiArray,
            topic,
            self.subscriber_callback,
            10
        )

    def subscriber_callback(self, msg):
        self.get_logger().info(f'{msg.data}')
        self.goal = []
        cli = FKClient()
        cli.send_request()

        while rclpy.ok():
            rclpy.spin_once(cli)
            if cli.future.done():
                try:
                    response = cli.future.result()
                except Exception as e:
                    self.get_logger().info(f'{e}')
                else:
                    self.get_logger().info(f'Get response: {response}')
                    self.goal = [
                        response.rho + msg.data[0],
                        response.phi + msg.data[1],
                        response.z + msg.data[2]
                    ]
                    self.publish()
                break
        
    
    def publish(self):
        pubTopic = "ik3r"
        self.publisher_ = self.create_publisher(Float64MultiArray, pubTopic, 1)
        pubMsg = Float64MultiArray()
        pubMsg.data = self.goal
        self.get_logger().info(f'Publishing: {pubMsg.data}')
        self.publisher_.publish(pubMsg)
        self.get_logger().info("Done")


class FKClient(Node):
    def __init__(self):
        super().__init__("FKClient")

        serviceFK = "/fk3r"
        self.get_logger().info(serviceFK)
        self.cli = self.create_client(FK, serviceFK)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service "{serviceFK}" not available, waiting again...')

    def send_request(self):
        req = FK.Request()
        self.future = self.cli.call_async(req)
        self.get_logger().debug("Request sended")


def main(args=None):
    rclpy.init(args=args)
    server = Simple3RArmRelativeController()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
