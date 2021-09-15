# simple_3r_arm
Here are three ROS2 packages:
- simple_3r_arm_description
- simple_3r_arm_hardware
- simple_3r_arm_launch

# Usage
- Install dependencies:
```cmd
rosdep install --from-paths <packages_path>
```

- Build:
```cam
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

```cmd
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data:
- 1
- 1.5
- 2"
```