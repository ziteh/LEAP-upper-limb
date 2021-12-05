import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class Simple3RArmForwardKinematicsNode(Node):
    def __init__(self):
        super().__init__("simple_3r_arm_forward_kinematics_node")

        self.declare_parameter("controller_name","forward_position_controller")
        self.declare_parameter("l2",0.5)
        self.declare_parameter("l3",0.5)

        self.controller_name = self.get_parameter("controller_name").value
        self.l2 = self.get_parameter("l2").value
        self.l3 = self.get_parameter("l3").value

        subTopic = "joint_states"
        self.subscriber = self.create_subscription(
            JointState,
            subTopic,
            self.subscriber_callback,
            10
        )

    def subscriber_callback(self, msg):
        self.joint_state = [msg.position[0], msg.position[1], msg.position[2]]
        self.publish()
    
    def publish(self):
        pubTopic = "/ik3r"
        self.publisher_ = self.create_publisher(Float64MultiArray, pubTopic, 1)
        pubMsg = Float64MultiArray()
        pubMsg.data = self.forward_kinematics()
        self.get_logger().info(f'Publishing: {pubMsg.data}')
        self.publisher_.publish(pubMsg)

    def forward_kinematics(self):
        x = float()
        x = self.l2 * math.cos(self.joint_state[1])+self.l3*math.cos(self.joint_state[1]+self.joint_state[2])

        y = float()
        y = self.l2 * math.sin(self.joint_state[1])+self.l3*math.sin(self.joint_state[1]+self.joint_state[2])
        y = -y

        z = float()
        z = self.joint_state[0]

        return [x, y, z]

def main(args=None):
    rclpy.init(args=args)
    node = Simple3RArmForwardKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
