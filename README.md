# ROS_tutorial
ROS2 cicular moving turtle

## Package Create
```
cd ~/turtle_ws/src
ros2 pkg create rvd_action_interfaces --build-type ament_cmake
ros2 pkg create moving_turtle --build-type ament_python
```

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

# ROS_example_1
ROS2 dwa ai moving turtle that pass waypoints and avoid obstacle

![rosgraph](https://user-images.githubusercontent.com/67641480/148827705-185c165c-8ff6-4571-b1fb-a79457712dd5.png)


## Package Create
```
cd ~/aiturtle_ws/src
ros2 pkg create dwa_aiturtle --build-type ament_python
```
## Run turtlesim & spawn new turtle
```
ros2 run turtlesim turtlesim_node
```
open new terminal
```
ros2 service call /spawn turtlesim/srv/Spawn "{x: 1.0, y: 1.0, theta: 0.0, name: 'turtle2'}"
ros2 run dwa_aiturtle dwaplanner
```
If you want, you can use launch file to run & spawn turtle
## Result
![dwa_turtle](https://user-images.githubusercontent.com/67641480/148827836-069a1304-5c24-4f37-b05a-34cc43d75743.png)

## Reference
[Dynamic window approach](https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/DynamicWindowApproach/dynamic_window_approach.py)
