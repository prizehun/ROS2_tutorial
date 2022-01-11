import math
import numpy as np
from numpy.lib.function_base import _create_arrays

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

class Aiturtle(Node):
    
    def __init__(self):
        super().__init__('Aiturtle')
        
        # robot parameter
        self.max_speed = 0.5  # [m/s]
        self.min_speed = -0.5  # [m/s]
        self.max_yaw_rate = 80.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 0.2  # [m/ss]
        self.max_delta_yaw_rate = 80.0 * math.pi / 180.0  # [rad/ss]
        self.v_resolution = 0.01  # [m/s]
        self.yaw_rate_resolution = 0.1 * math.pi / 180.0  # [rad/s]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.predict_time = 3.0  # [s]
        self.to_goal_cost_gain = 0.15
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.0
        self.robot_stuck_flag_cons = 0.001  # constant to prevent robot stucked
        self.ob2 = np.array([[1.0, 1.0]])
        
        self.counter1 = 0
        self.counter2 = 0

        # if turtle_type == RobotType.circle
        # Also used to check if goal is reached in both types
        self.robot_radius = 0.5  # [m] for collision check

        # if turtle_type == RobotType.rectangle
        self.robot_width = 1.0  # [m] for collision check
        self.robot_length = 1.0  # [m] for collision check

        # waypoints
        self.waypoints = np.array([[8.58, 2.5],
                          [2.5, 8.58],
                          [2.5, 2.5],
                          [8.58, 8.58]
                          ])        
        
        self.declare_parameter('qos_depth', 10)
        qos_depth = self.get_parameter('qos_depth').value

        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            durability=QoSDurabilityPolicy.VOLATILE)
        
        self.cp1_pub = self.create_publisher(
            Pose,
            '/turtle1/pose',
            QOS_RKL10V
        )
        
        self.cp1_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.cp1_sub_create,
            QOS_RKL10V
        )
        
        self.cp2_pub = self.create_publisher(
            Pose,
            '/turtle2/pose',
            QOS_RKL10V
        )            
        
        self.cp2_sub = self.create_subscription(
            Pose,
            'turtle2/pose',
            self.cp2_sub_create,
            QOS_RKL10V
        )
        
        self.moving2_pub = self.create_publisher(
            Twist,
            '/turtle2/cmd_vel',
            QOS_RKL10V
        )
        
        self.moving1_pub = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            QOS_RKL10V
        )
        
        
    def cp1_sub_create(self, msg):
        self.ob1 = np.array([[msg.x, msg.y]])
        twist1 = Twist()
        
        twist1.linear.y, twist1.linear.z = 0.0, 0.0
        twist1.angular.x, twist1.angular.y = 0.0, 0.0
        
        # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        x1 = np.array([msg.x, msg.y, msg.theta, msg.linear_velocity, msg.angular_velocity])
        
        trajectory1 = np.array(x1)
        
        # goal position [x(m), y(m)]
        gx = self.waypoints[self.counter1][0]
        gy = self.waypoints[self.counter1][1]
        goal = np.array([gx,gy])
        
        u1, predict_trajectory1 = dwa_control(x1, self, goal, self.ob2)
        x1 = motion(x1, u1, self.dt)
        trajectory1 = np.vstack((trajectory1, x1))
        
        # check reaching goal
        dist_to_goal = math.hypot(x1[0] - goal[0], x1[1] - goal[1])
        if dist_to_goal <= self.robot_width/2:
            self.get_logger().info('First turtle, Goal! way point : {0}'.format(self.counter1+1))
            if self.counter1 == 3:
                self.counter1=0
            else:
                self.counter1 += 1
        
        # from motion    
        twist1.linear.x = x1[3]
        twist1.angular.z = x1[4]
        
        self.moving1_pub.publish(twist1)
        
    def cp2_sub_create(self, msg):
        self.ob2 = np.array([[msg.x, msg.y]])
        twist = Twist()
        
        twist.linear.y, twist.linear.z = 0.0, 0.0
        twist.angular.x, twist.angular.y = 0.0, 0.0
        
        # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        x = np.array([msg.x, msg.y, msg.theta, msg.linear_velocity, msg.angular_velocity])
        
        trajectory = np.array(x)
        
        # goal position [x(m), y(m)]
        gx = self.waypoints[self.counter2][0]
        gy = self.waypoints[self.counter2][1]
        goal = np.array([gx,gy])
        
        u, predict_trajectory = dwa_control(x, self, goal, self.ob1)
        x = motion(x, u, self.dt)
        trajectory = np.vstack((trajectory, x))
        
        # check reaching goal
        dist_to_goal = math.hypot(x[0] - goal[0], x[1] - goal[1])
        if dist_to_goal <= self.robot_width/2:
            self.get_logger().info('Second turtle, Goal! way point : {0}'.format(self.counter2+1))
            if self.counter2 == 3:
                self.counter2=0
            else:
                self.counter2 += 1
        
        # from motion    
        twist.linear.x = x[3]
        twist.angular.z = x[4]
        
        self.moving2_pub.publish(twist)
        
def dwa_control(x, config, goal, ob):
    """
    Dynamic Window Approach control
    """
    dw = calc_dynamic_window(x, config)

    u, trajectory = calc_control_and_trajectory(x, dw, config, goal, ob)

    return u, trajectory


def motion(x, u, dt):
    """
    motion model
    """

    x[2] += u[1] * dt
    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[3] = u[0]
    x[4] = u[1]

    return x


def calc_dynamic_window(x, config):
    """
    calculation dynamic window based on current state x
    """

    # Dynamic window from robot specification
    Vs = [config.min_speed, config.max_speed,
        -config.max_yaw_rate, config.max_yaw_rate]

    # Dynamic window from motion model
    Vd = [x[3] - config.max_accel * config.dt,
        x[3] + config.max_accel * config.dt,
        x[4] - config.max_delta_yaw_rate * config.dt,
        x[4] + config.max_delta_yaw_rate * config.dt]

    #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
        max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    return dw


def predict_trajectory(x_init, v, y, config):
    """
    predict trajectory with an input
    """

    x = np.array(x_init)
    trajectory = np.array(x)
    time = 0
    while time <= config.predict_time:
        x = motion(x, [v, y], config.dt)
        trajectory = np.vstack((trajectory, x))
        time += config.dt

    return trajectory


def calc_control_and_trajectory(x, dw, config, goal, ob):
    """
    calculation final input with dynamic window
    """

    x_init = x[:]
    min_cost = float("inf")
    best_u = [0.0, 0.0]
    best_trajectory = np.array([x])

    # evaluate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1], config.v_resolution):
        for y in np.arange(dw[2], dw[3], config.yaw_rate_resolution):

            trajectory = predict_trajectory(x_init, v, y, config)
            # calc cost
            to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(trajectory, goal)
            speed_cost = config.speed_cost_gain * (config.max_speed - trajectory[-1, 3])
            ob_cost = config.obstacle_cost_gain * calc_obstacle_cost(trajectory, ob, config)

            final_cost = to_goal_cost + speed_cost + ob_cost  ### 최종 비용

            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                best_u = [v, y]
                best_trajectory = trajectory
                if abs(best_u[0]) < config.robot_stuck_flag_cons \
                        and abs(x[3]) < config.robot_stuck_flag_cons:
                    # to ensure the robot do not get stuck in
                    # best v=0 m/s (in front of an obstacle) and
                    # best omega=0 rad/s (heading to the goal with
                    # angle difference of 0)
                    best_u[1] = -config.max_delta_yaw_rate
    return best_u, best_trajectory


def calc_obstacle_cost(trajectory, ob, config):
    """
    calc obstacle cost inf: collision
    """
    ox = ob[:, 0]
    oy = ob[:, 1]
    dx = trajectory[:, 0] - ox[:, None]
    dy = trajectory[:, 1] - oy[:, None]
    r = np.hypot(dx, dy)

    yaw = trajectory[:, 2]
    rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    rot = np.transpose(rot, [2, 0, 1])
    local_ob = ob[:, None] - trajectory[:, 0:2]
    local_ob = local_ob.reshape(-1, local_ob.shape[-1])
    local_ob = np.array([local_ob @ x for x in rot])
    local_ob = local_ob.reshape(-1, local_ob.shape[-1])
    upper_check = local_ob[:, 0] <= config.robot_length / 2
    right_check = local_ob[:, 1] <= config.robot_width / 2
    bottom_check = local_ob[:, 0] >= -config.robot_length / 2
    left_check = local_ob[:, 1] >= -config.robot_width / 2
    if (np.logical_and(np.logical_and(upper_check, right_check),
                    np.logical_and(bottom_check, left_check))).any():
        return float("Inf")

    min_r = np.min(r)
    return 1.0 / min_r  # OK


def calc_to_goal_cost(trajectory, goal):
    """
        calc to goal cost with angle difference
    """

    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]
    error_angle = math.atan2(dy, dx)
    cost_angle = error_angle - trajectory[-1, 2]
    cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

    return cost


def main(args=None):
    rclpy.init(args=args)
    node = Aiturtle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
