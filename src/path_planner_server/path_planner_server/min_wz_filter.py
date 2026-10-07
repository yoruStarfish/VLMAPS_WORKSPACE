import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MinWzFilter(Node):
    def __init__(self):
        super().__init__('min_wz_filter')

        self.declare_parameter('min_wz', 0.08)
        self.declare_parameter('trigger_wz', 0.001)
        self.declare_parameter('low_vx_threshold', 0.03)

        self.min_wz = self.get_parameter('min_wz').value
        self.trigger_wz = self.get_parameter('trigger_wz').value
        self.low_vx_threshold = self.get_parameter('low_vx_threshold').value

        self.sub = self.create_subscription(
            Twist,
            'cmd_vel_in',
            self.cmd_callback,
            10
        )

        self.pub = self.create_publisher(
            Twist,
            'cmd_vel_out',
            10
        )

        self.get_logger().info(
            f"MinWzFilter started: min_wz={self.min_wz}, "
            f"trigger_wz={self.trigger_wz}, "
            f"low_vx_threshold={self.low_vx_threshold}"
        )

    def cmd_callback(self, msg):
        out = Twist()
        out.linear.x = msg.linear.x
        out.linear.y = msg.linear.y
        out.linear.z = msg.linear.z
        out.angular.x = msg.angular.x
        out.angular.y = msg.angular.y
        out.angular.z = msg.angular.z

        vx = out.linear.x
        wz = out.angular.z

        if abs(vx) < self.low_vx_threshold:
            if self.trigger_wz < abs(wz) < self.min_wz:
                old_wz = wz
                out.angular.z = self.min_wz if wz > 0 else -self.min_wz
                self.get_logger().info(
                    f"wz compensated: {old_wz:.4f} -> {out.angular.z:.4f}, vx={vx:.4f}"
                )

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MinWzFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()