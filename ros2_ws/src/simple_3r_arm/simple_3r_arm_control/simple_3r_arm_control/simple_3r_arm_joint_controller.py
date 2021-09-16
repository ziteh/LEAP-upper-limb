import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class Simple3RArmJointControlPublisher(Node):
    def __init__(self):
        super().__init__("simple_3r_arm_joint_controller")

        self.declare_parameter("controller_name","forward_position_controller")
        self.declare_parameter("goal", [0,0,0])
        self.declare_parameter("frequency", 2)

        controller_name = self.get_parameter("controller_name").value
        goal = self.get_parameter("goal").value
        frequency = self.get_parameter("frequency").value

        self.f_goal = []
        for value in goal:
            self.f_goal.append(float(value))
        
        topic = "/" + controller_name + "/" + "commands"
        self.get_logger().info(f'Topic: {topic}')
        self.publisher_ = self.create_publisher(Float64MultiArray, topic, 1)
        self.timer = self.create_timer(frequency, self.timer_callback)

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data = self.f_goal
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.publisher_.publish(msg)
        self.get_logger().info("Done")

def main(args=None):
    rclpy.init(args=args)
    control_publisher = Simple3RArmJointControlPublisher()
    rclpy.spin(control_publisher)
    control_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
