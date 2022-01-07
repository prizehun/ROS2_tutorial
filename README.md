# ROS_tutorial
ROS2 cicular moving turtle

## Build
in your work space
```
cd ~/turtle_ws
colcon build --symlink-install --packages-select rvd_action_interfaces
colcon build --symlink-install --packages-select moving_turtle
. ~/turtle_ws/install/local_setup.bash
```
## Launch
```
ros2 launch moving_turtle circularturtle.launch.py
```
## Topic publish
open other terminal
```
ros2 topic pub --once /rad_vel_dir rvd_action_interfaces/msg/RVD "{radius: 4.0, velocity: 10.0, direction: True}"
```
if direction is true(1) then turtle moves in counter clockwise direction.
else moves in clockwise direction
