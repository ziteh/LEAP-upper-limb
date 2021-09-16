from _typeshed import Self
import rclpy
from rclpy import publisher
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class Simple3RArmJointControlPublisher(Node):
    def __init__(self):
        super().__init__("simple_3r_arm_joint_controller")

        self.declare_parameter("controller_name","forward_position_controller")
        self.declare_parameter("goal", [0,0,0])

        controller_name = self.get_parameter("controller_name").value
        goal = self.get_parameter("goal").value

        f_goal = []
        for value in goal:
            self.f_goal.append(float(value))

        topic = "/" + controller_name + "/" + "commands"
        
        self.publisher = self.create_publisher(Float64MultiArray, topic,1)
        msg = Float64MultiArray()
        msg.data = f_goal
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.publisher.publish(msg)

def main(args=Node):
    rclpy.init(args=args)
    control_publisher = Simple3RArmJointControlPublisher()
    rclpy.spin(control_publisher)
    control_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
