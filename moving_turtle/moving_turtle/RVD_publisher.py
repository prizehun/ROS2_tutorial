from rvd_action_interfaces.msg import RVD

from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

class RVDPublisher(Node):
    
    def __init__(self):

        super().__init__('RVD_publisher')

        self.declare_parameter('qos_depth', 10)
        qos_depth = self.get_parameter('qos_depth').value

        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            durability=QoSDurabilityPolicy.VOLATILE)

        self.RVD_publisher = self.create_publisher(
            RVD,
            'rad_vel_dir',
            QOS_RKL10V)

        self.timer = self.create_timer(1.0, self.publish_rvd)

    def publish_rvd(self):
        msg = RVD()
        msg.radius = self.radius
        msg.velocity = self.velocity
        msg.direction = self.direction
        self.RVD_publisher.publish(msg)
        self.get_logger().info('Published radius: {0}'.format(msg.radius))
        self.get_logger().info('Published velocity: {0}'.format(msg.velocity))
        self.get_logger().info('Published direction: {0}'.format(msg.direction))


def main(args=None):
    rclpy.init(args=args)
    try:
        node = RVDPublisher()
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