# simple_3r_arm
Here are some ROS2 packages:
- simple_3r_arm_control
- simple_3r_arm_description
- simple_3r_arm_hardware
- simple_3r_arm_interface
- simple_3r_arm_launch
- ros2_control_test_nodes (From [ros2_control_demos/ros2_control_test_nodes](https://github.com/ros-controls/ros2_control_demos/tree/master/ros2_control_test_nodes))

# Usage
- Install dependencies:
```cmd
rosdep install --from-paths <packages_path>
```

- Build:
```cmd
colcon build
```

Do not forget to source `setup.bash` from the `install` folder.

## Show in RViz
Only one command:  
```
ros2 launch simple_3r_arm_description view_robot.launch.py
```

## Run with ros2_control
```cmd
ros2 launch simple_3r_arm_launch simple_3r_arm.launch.py
```  

Control by the following commands:
### A. ROS CLI
```cmd
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data:
- 1
- 1.5
- 2"
```

### B. Using `ros2_control_test_nodes`:
```cmd
ros2 launch simple_3r_arm_launch test_forward_position_controller.launch.py
```

### C. Using `simple_3r_arm_joint_controller` node:
```cmd
ros2 run simple_3r_arm_control simple_3r_arm_joint_controller --ros-args -p goal:=[1.0,1.5,2.0] -p frequency:=2
```

### D. Using with forward/inverse kinematics:
Start:
```cmd
ros2 launch simple_3r_arm_launch simple_3r_arm.launch_with_kinematics.py
```

Control position of simple_3r_arm by inverse kinematics:
```cmd
ros2 topic pub /ik3r std_msgs/msg/Float64MultiArray "data:
- 0.4
- 0.1
- 0.2"
```

Get position of simple_3r_arm by forward kinematics:
```cmd
ros2 service call /fk3r simple_3r_arm_interface/srv/FK
```
