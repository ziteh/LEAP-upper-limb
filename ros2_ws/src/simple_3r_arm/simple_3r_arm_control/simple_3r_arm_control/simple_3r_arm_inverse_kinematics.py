import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String

class Simple3RArmInverseKinematicsSubscriber(Node):
    def __init__(self):
        super().__init__("simple_3r_arm_inverse_kinematice")

        self.declare_parameter("controller_name","forward_position_controller")
        self.declare_parameter("l2",0.5)
        self.declare_parameter("l3",0.5)

        self.controller_name = self.get_parameter("controller_name").value
        self.l2 = self.get_parameter("l2").value
        self.l3 = self.get_parameter("l3").value
        
        subscriberTopic = "ik3r"
        self.get_logger().info(f'Topic: {subscriberTopic}')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            subscriberTopic,
            self.listener_callback,
            10
        )
        self.subscription # Prevent unused variable warning.

    def listener_callback(self, msg):
        fMsg = Float64MultiArray()
        fMsg.data = msg.data
        self.get_logger().info(f'Get: {msg.data}')
        rho = msg.data[0]
        phi = msg.data[1]
        z = -msg.data[2]

        theta2 = 2*math.atan2(
            math.sqrt(math.pow(self.l2+self.l3,2)-(math.pow(rho,2)+math.pow(z,2))),
            math.sqrt((math.pow(rho,2)+math.pow(z,2))-math.pow(self.l2-self.l3,2))
            )
        theta1 = math.atan2(z,rho)+math.atan2(
            self.l3*math.sin(theta2),
            self.l2 + self.l3*math.cos(theta2)
        )
        goal = [
            phi,
            theta1,
            -theta2
        ]

        pubTopic = "/" + self.controller_name + "/" + "commands"
        self.publisher_ = self.create_publisher(Float64MultiArray, pubTopic, 1)
        pubMsg = Float64MultiArray()
        pubMsg.data = goal
        self.get_logger().info(f'Publishing: {pubMsg.data}')
        self.publisher_.publish(pubMsg)
        self.get_logger().info("Done")
    
def main(args=None):
    rclpy.init(args=args)
    subscriber = Simple3RArmInverseKinematicsSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
