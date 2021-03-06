from typing import Tuple
import rclpy
import time
import threading
import serial
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray

class SerialPortSubscriber(Node):
    def __init__(self):
        super().__init__("serial_port_subscriber")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", "9600")
        self.declare_parameter("receive_delay", 0.01)

        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").value
        self.receiveDelay = self.get_parameter("receive_delay").value

        self.writeTopicName = f"serialPort/write{self.port}" 
        self.readTopicName = f"serialPort/read{self.port}" 

        self.get_logger().debug(f'Port: {self.port}, Baudrate: {self.baudrate}, Write Topic: {self.writeTopicName}, Read Topic: {self.readTopicName}')

        self.subscriber = self.create_subscription(
            ByteMultiArray,
            self.writeTopicName,
            self.subsrciber_callback,
            10
        )

        self.init_serial_port()

    def subsrciber_callback(self, msg):
        self.get_logger().debug(f'Get: {msg}')
        self.sp.send(msg.data)

    def init_serial_port(self):
        self.publisher_ = self.create_publisher(ByteMultiArray, self.readTopicName, 1)
        self.sp = SerialPort(self.receiveDelay, self.port, self.baudrate,self.publisher_)
        thread = threading.Thread(target = self.sp.read)
        thread.start()


class SerialPort:
    def __init__(self, timesleep ,port, baudrate, rosPub):
        self.timeSleep = timesleep
        self.publisher = rosPub
        self.sp = serial.Serial(port,baudrate)
        self.open()
    
    def open(self):
        if not self.sp.is_open:
            self.sp.open()
    
    def close(self):
        self.sp.close()
    
    def send(self, data):
        dataInt =  []
        for d in data:
            dataInt.append(d[0])

        number = self.sp.write(bytes(dataInt))
        return number

    def read(self):
        try:
            while True:
                if self.sp.in_waiting > 0:
                    spMsg = self.sp.read_until(b'\xff')

                    pubMsg = ByteMultiArray()
                    for s in spMsg:
                        pubMsg.data.append(bytes([s]))

                    if(pubMsg.data[0][0] > 0x7f):
                        pubMsg.data.pop() # Remove last element (0xff).
                        self.publisher.publish(pubMsg)
                        time.sleep(self.timeSleep)

        except KeyboardInterrupt:
            if self.sp != None:
                self.sp.close()
        
        except IndexError:
            1


def main(args=None):
    rclpy.init(args=args)

    subscriber = SerialPortSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
