# serial_port

Here are some ROS2 executables:
- basic_serial_port
- asyn_serial_port (Useless)
- thread_serial_port
- read_write_from_topic (with thread)
- read_write_from_topic_byte (copied from 'read_write_from_topic')
- read_write_from_topic_bytemultiarray (copied from 'read_write_from_topic_byte')

# Usage

Change access permissions first any time you plug in an USB serial port device.

Example of `ttyACM0`:
```cmd
sudo chmod 666 /dev/ttyACM0
```

## read_write_from_topic

Run executable:
```cmd
ros2 run serial_port read_write_from_topic 
```

Print received message from serial port:
```cmd
ros2 topic echo /serialPort/read/dev/ttyACM0 
```

Send message (send "Hello World" for example):
```cmd
ros2 topic pub /serialPort/write/dev/ttyACM0 std_msgs/msg/String "data: Hello Woeld"
```

## read_write_from_topic_byte

Run executable:
```cmd
ros2 run serial_port read_write_from_topic_byte
```

Print received message from serial port:
```cmd
ros2 topic echo /serialPort/read/dev/ttyACM0 
```

Send message (send `0x21` for example):
```cmd
ros2 topic pub /serialPort/write/dev/ttyACM0 std_msgs/msg/Byte "data: [0x21]"
```
