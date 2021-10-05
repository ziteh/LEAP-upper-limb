from typing import Tuple
import rclpy
import time
import threading
import serial
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class SerialPortSubscriber(Node):
    def __init__(self):
        super().__init__("serial_port_subscriber")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", "9600")
        self.declare_parameter("receive_delay", 1)

        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").value
        self.receiveDelay = self.get_parameter("receive_delay").value
        self.topicName = "serialPort"

        self.get_logger().info(f'Port: {self.port}, Baudrate: {self.baudrate}, Topic: {self.topicName}')

        self.subscriber = self.create_subscription(
            Float64MultiArray,
            self.topicName,
            self.subsrciber_callback,
            10
        )
        self.init_serial_port()

    def subsrciber_callback(self, msg):
        self.get_logger().info(f'{msg}')

    def init_serial_port(self):
        self.sp = SerialPort(self.receiveDelay, self.port, self.baudrate)
        thread = threading.Thread(target = self.sp.read)
        thread.start()


class SerialPort:
    latestMessage = '<NEVER RECEIVE>'

    def __init__(self, timesleep ,port, baudrate):
        self.timeSleep = timesleep
        self.sp = serial.Serial(port,baudrate)
        self.open()
    
    def open(self):
        if not self.sp.is_open:
            self.sp.open()
    
    def close(self):
        self.sp.close()
    
    def send(self, data):
        number = self.sp.write(data)
        return number

    def read(self):
        try:
            while True:
                self.latestMessage = self.sp.readline().decode('utf-8').splitlines()[0]
                print(self.latestMessage)
                time.sleep(self.timeSleep)

        except KeyboardInterrupt:
            if self.sp != None:
                self.sp.close()


def main(args=None):
    rclpy.init(args=args)

    subscriber = SerialPortSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
