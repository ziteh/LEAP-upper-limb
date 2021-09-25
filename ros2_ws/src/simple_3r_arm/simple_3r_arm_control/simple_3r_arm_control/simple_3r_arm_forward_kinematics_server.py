import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from simple_3r_arm_interface.srv import FK

class Simple3RArmForwardKinematicsServer(Node):
    def __init__(self):
        super().__init__("simple_3r_arm_forward_kinematice_server")

        self.declare_parameter("controller_name","forward_position_controller")
        self.declare_parameter("l2",0.5)
        self.declare_parameter("l3",0.5)

        self.controller_name = self.get_parameter("controller_name").value
        self.l2 = self.get_parameter("l2").value
        self.l3 = self.get_parameter("l3").value

        self.theta = [0.0, 0.0, 0.0]
        topicJointState = "joint_states"
        self.subscriber = self.create_subscription(
            JointState,
            topicJointState,
            self.joint_state_callback,
            10
        )

        serviceFK = "fk3r"
        self.get_logger().info(f'Service: {serviceFK}')
        self.srv = self.create_service(FK,serviceFK,self.fk_callback)

    def fk_callback(self, request, response):
        response.rho = self.l2 * math.cos(self.theta[1])+self.l3*math.cos(self.theta[1]+self.theta[2])
        response.z = self.l2 * math.sin(self.theta[1])+self.l3*math.sin(self.theta[1]+self.theta[2])
        response.phi = self.theta[0]
        response.z = -response.z

        return response

    def joint_state_callback(self, msg):
        #self.get_logger().info(f'Get: {msg.position}')
        self.theta = [
            msg.position[0],
            msg.position[1],
            msg.position[2]
        ]
        # rate=self.create_rate(2)
        # rate.sleep()
    
def main(args=None):
    rclpy.init(args=args)
    server = Simple3RArmForwardKinematicsServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
