from rvd_action_interfaces.msg import RVD
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy


class RVDSubscriber(Node):

    def __init__(self):

        super().__init__('RVD_subscriber')

        self.radius = 0.0
        self.velocity = 0.0
        self.direction = True

        self.linear = None
        self.angular = None
        self.twist = None

        self.lin_x = 0.0
        self.lin_y = 0.0
        self.lin_z = 0.0
        self.ang_x = 0.0
        self.ang_y = 0.0
        self.ang_z = 0.0

        self.declare_parameter('qos_depth', 10)
        qos_depth = self.get_parameter('qos_depth').value

        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            durability=QoSDurabilityPolicy.VOLATILE)

        self.rvd_subscriber = self.create_subscription(
            RVD,
            'rad_vel_dir',
            self.subscribe_rvd,
            QOS_RKL10V)

        self.rvd_subscriber

        self.cmd_publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            QOS_RKL10V)

        self.timer = self.create_timer(1.0, self.publish_cmd)

    def subscribe_rvd(self, msg):
        self.radius = msg.radius
        self.velocity = msg.velocity
        self.direction = msg.direction
        self.get_logger().info('Subscribed radius: {0}'.format(msg.radius))
        self.get_logger().info('Subscribed velocity: {0}'.format(msg.velocity))
        self.get_logger().info('Subscribed direction: {0}'.format(msg.direction))

    def publish_cmd(self):
        self.linear = Vector3()
        self.angular = Vector3()
        self.twist = Twist()

        self.lin_x = self.velocity
        if self.direction == True:
            self.ang_z = (self.velocity / (self.radius + 0.00001))
        else:
            self.ang_z = -(self.velocity / (self.radius + 0.00001))

        self.linear.x, self.linear.y, self.linear.z = self.lin_x, self.lin_y, self.lin_z
        self.angular.x, self.angular.y, self.angular.z = self.ang_x, self.ang_y, self.ang_z
        self.twist.linear = self.linear
        self.twist.angular = self.angular

        self.cmd_publisher.publish(self.twist)
        self.get_logger().info('Published linear velocity: {0}'.format(self.lin_x))
        self.get_logger().info('Published angular velocity: {0}'.format(self.ang_z))


def main(args=None):
    rclpy.init(args=args)
    try:
        node = RVDSubscriber()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()