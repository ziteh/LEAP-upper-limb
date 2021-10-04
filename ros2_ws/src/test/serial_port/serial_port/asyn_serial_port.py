import rclpy
import serial
import asyncio
import serial_asyncio
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class SerialPortSubscriber(Node):
    def __init__(self):
        super().__init__("serial_port_subscriber")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", "9600")

        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").value
        self.topicName = "serialPort"

        self.get_logger().info(f'Port: {self.port}, Baudrate: {self.baudrate}, Topic: {self.topicName}')

        self.subscriber = self.create_subscription(
            Float64MultiArray,
            self.topicName,
            self.subsrciber_callback,
            10
        )

        # self.ser = serial.Serial(self.port, self.baudrate)
        self.serial_port_send()

    def subsrciber_callback(self, msg):
        self.get_logger().info(f'{msg}')
        # self.ser.write(msg)

    def serial_port_send(self):
        loop = asyncio.get_event_loop()
        coro = serial_asyncio.create_serial_connection(loop, OutputProtocol, self.port, baudrate=self.baudrate)
        transport, protocol = loop.run_until_complete(coro)
        loop.run_forever()
        loop.close()
    
# Source: https://pyserial-asyncio.readthedocs.io/en/latest/shortintro.html#protocol-example
class OutputProtocol(asyncio.Protocol):
    def connection_made(self, transport):
        self.transport = transport
        print('port opened', transport)
        transport.serial.rts = False  # You can manipulate Serial object via transport
        transport.write(b'Hello, World!\n')  # Write serial data via transport

    def data_received(self, data):
        print('data received', repr(data))
        # if b'\n' in data:
            # self.transport.close()

    def connection_lost(self, exc):
        print(f'{exc}')
        print('port closed')
        self.transport.loop.stop()

    def pause_writing(self):
        print('pause writing')
        print(self.transport.get_write_buffer_size())

    def resume_writing(self):
        print(self.transport.get_write_buffer_size())
        print('resume writing')

def main(args=None):
    rclpy.init(args=args)

    subscriber = SerialPortSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
