# ROS_tutorial
ROS2 cicular moving turtle

## Object
Turtlesim에서 아래 조건을 만족하면서 입력 명령을 받아 거북이를 등속 원운동하게하는 ROS2 패키지를 설계하시오. 
1. 원운동의 회전반경(radius), linear velocity(velocity), 회전방향(direction) 값을 포함하는 ROS 메세지 타입을 정의하고 이 토픽 메세지를 받아 Turtlesim에 속도명령(cmd_vel)을 주는 프로그램을 설계한다. 
2. ros2 launch로 turtlesim과 설계한 프로그램을 동시에 실행할 수 있는 launch 파일을 제작한다. 
3. turtlesim 실행 후, ros2 topic pub으로 새롭게 정의한 메시지 타입을 publish하여 등속 원운동을 명령한다. 
4. C++ / Python등의 언어는 자유롭게 선택하여 진행한다.

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
ROS2 dwa ai moving turtle that passes waypoints and avoids obstacle    
## Object
로봇(turtle2)의 위치를 topic을 통해 파악하고, static 장애물(turtle1)을 피해 waypoint를 순환하도록 /turtle2/cmd_vel 을 Output으로 보내는 dwa local planner 패키지 작성. 필요하다면 파라미터를 조정할 것
## rqt_graph
![rosgraph](https://user-images.githubusercontent.com/67641480/148827705-185c165c-8ff6-4571-b1fb-a79457712dd5.png)


## Package Create
```
cd ~/aiturtle_ws/src
ros2 pkg create dwa_aiturtle --build-type ament_python
```
## Build
in your work space
```
cd ~/aiturtle_ws
colcon build --symlink-install --packages-select dwa_aiturtle
. ~/aiturtle_ws/install/local_setup.bash
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

# ROS_example_2
advanced example_1, two turtles pass waypoints and avoid each other
## rqt_graph
![rosgraph1](https://user-images.githubusercontent.com/67641480/149022946-50288f18-1630-41b2-8286-e71281ad533c.png)
## Run
package create, build as same as example1 but you should add numbering '2' like aiturtle2, dwa_aiturtle2 and launch
```
ros2 launch dwa_aiturtle2 aitrutle.launch.py
```
## Result
![스크린샷, 2022-01-12 05-14-36](https://user-images.githubusercontent.com/67641480/149023718-0c429f2a-feaa-437c-8144-66fdd22e2fbc.png)
The pathway will change as you modify the parameters
