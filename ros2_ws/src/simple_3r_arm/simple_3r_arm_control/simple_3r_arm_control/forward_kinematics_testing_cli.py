import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from simple_3r_arm_interface.srv import FK

class TestingCli(Node):
    def __init__(self):
        super().__init__("testing_fk_cli")
        serviceFK = "fk3r"
        self.cli = self.create_client(FK, serviceFK)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service "{serviceFK}" not available, waiting again...')

    def send_request(self):
        req = FK.Request()
        self.future = self.cli.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    cli = TestingCli()
    cli.send_request()

    while rclpy.ok():
        rclpy.spin_once(cli)
        if cli.future.done():
            try:
                response = cli.future.result()
            except Exception as e:
                cli.get_logger().info(f'{e}')
            else:
                cli.get_logger().info(f'Get: {response}')
            break

    cli.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
