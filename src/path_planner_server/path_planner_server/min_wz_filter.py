import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MinWzFilter(Node):
    def __init__(self):
        super().__init__('min_wz_filter')

        self.declare_parameter('min_wz', 0.8)
        self.declare_parameter('trigger_wz', 0.001)
        self.declare_parameter('low_vx_threshold', 0.03)
        self.min_wz = self.get_parameter('min_wz').value
        self.trigger_wz = self.get_parameter('trigger_wz').value
        self.low_vx_threshold = self.get_parameter('low_vx_threshold').value
        self.declare_parameter('turn_in_place_threshold', 0.2)
        self.declare_parameter('turn_in_place_release', 0.1)
        self.declare_parameter('command_timeout', 0.5)
        self.turn_threshold = self.get_parameter('turn_in_place_threshold').value
        self.turn_release = self.get_parameter('turn_in_place_release').value
        self.command_timeout = self.get_parameter('command_timeout').value
        if (not all(math.isfinite(v) for v in (
                self.turn_threshold, self.turn_release, self.command_timeout))
                or not 0 <= self.turn_release < self.turn_threshold
                or self.command_timeout <= 0):
            raise ValueError('Require 0 <= release < threshold and positive timeout')
        self.turning = False
        self.last_command_time = None

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
            f'MinWzFilter fixed compensation: min_wz={self.min_wz}, '
            f'trigger_wz={self.trigger_wz}, low_vx_threshold={self.low_vx_threshold}')
        self.get_logger().info(
            f'Turn in place: enter={self.turn_threshold}, release={self.turn_release}')
        self.watchdog = self.create_timer(0.05, self.check_timeout)

    def check_timeout(self):
        if (self.last_command_time is not None and
                time.monotonic() - self.last_command_time > self.command_timeout):
            self.pub.publish(Twist())
            self.turning = False
            self.last_command_time = None

    def cmd_callback(self, msg):
        now = time.monotonic()
        if (self.last_command_time is not None and
                now - self.last_command_time > self.command_timeout):
            self.turning = False
        self.last_command_time = now
        out = Twist()
        out.linear.x = msg.linear.x
        out.linear.y = msg.linear.y
        out.linear.z = msg.linear.z
        out.angular.x = msg.angular.x
        out.angular.y = msg.angular.y
        out.angular.z = msg.angular.z

        vx = out.linear.x
        wz = out.angular.z

        was_turning = self.turning
        if not math.isfinite(vx) or not math.isfinite(wz):
            self.turning = False
            self.pub.publish(Twist())
            return
        # 導航器仍要求移動時保留原命令，避免低速前進被轉成原地旋轉。
        # 原地旋轉補償只處理明確要求零線速度的命令。
        turning_in_place = abs(vx) <= 0.001
        if not turning_in_place or abs(wz) <= self.turn_release:
            self.turning = False
        elif abs(wz) >= self.turn_threshold:
            self.turning = True
        if self.turning:
            out.linear.x = 0.0
            out.linear.y = 0.0
            out.linear.z = 0.0
            out.angular.z = math.copysign(max(abs(wz), self.min_wz), wz)
        if was_turning != self.turning:
            self.get_logger().info(
                f'Turn in place {"ON" if self.turning else "OFF"}: '
                f'input vx={vx:.3f}, wz={wz:.3f}')

        if not self.turning and turning_in_place:
            if self.turn_release < abs(wz) < self.min_wz:
                out.angular.z = self.min_wz if wz > 0 else -self.min_wz
                self.get_logger().info(
                    f'wz compensated: {wz:.4f} -> {out.angular.z:.4f}, vx={vx:.4f}')
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MinWzFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
