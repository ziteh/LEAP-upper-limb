# data_package

Here are some ROS2 executables:
- sending_test
- sending_test_bytemultiarray
- motor_position_control_sender

# Usage

## sending_test_bytemultiarray

Change access permissions first any time you plug in an USB serial port device.

Example of `ttyACM0`:
```cmd
sudo chmod 666 /dev/ttyACM0
```

Init and connect to serial port device:
```cmd
ros2 run serial_port read_write_from_topic_bytemultiarray
```

Send `0x87` `0x00` for example:
```cmd
ros2 run data_package sending_test_bytemultiarray --ros-args -p data:=[0x87,0x00]
```
## motor_position_control

Launch with:

```cmd
ros2 launch data_package motor_position_control.launch.py 
```

Send position(%) by publish topic:
```cmd
ros2 topic pub /motor/positionControl std_msgs/msg/UInt8 "data: 50"
```
