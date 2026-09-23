#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Use merely cmd_vel to control the robot to move in a predefined path. This is a simple example to demonstrate how to use cmd_vel to control the robot's movement. The robot will move in a straight line, rotate, and then move back to the starting point.
class CmdVelExperiment(Node):

    def __init__(self):
        super().__init__('cmd_vel_experiment')

        # ========================================
        # Publisher
        # ========================================
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # ========================================
        # Experiment
        # ========================================
        self.EXPERIMENT = "A_B_A"

        # 可選：
        # "A_B_A"     : A → B → A
        # "A_C_A"     : A → C → A
        # "A_B_C_B_A" : A → B → C → B → A

        # ========================================
        # Robot control parameters
        # ========================================
        self.linear_speed = 0.10      # m/s
        self.angular_speed = 0.30     # rad/s

        # ========================================
        # Experiment geometry
        # ========================================
        self.A = (10.0, 1.0)
        self.B = (10.0, 4.0)
        self.C = (15.0, 4.0)

    # ============================================================
    # Publish velocity
    # ============================================================

    def publish_velocity(self, linear_x=0.0, angular_z=0.0):

        cmd = Twist()

        cmd.linear.x = linear_x
        cmd.angular.z = angular_z

        self.cmd_vel_pub.publish(cmd)

    # ============================================================
    # Stop robot
    # ============================================================

    def stop_robot(self):

        self.publish_velocity(0.0, 0.0)

        # 再送幾次 0，避免底盤沒有收到
        for _ in range(5):
            self.publish_velocity(0.0, 0.0)
            time.sleep(0.05)

    # ============================================================
    # Move straight
    # ============================================================

    def move_forward(self, distance):

        duration = abs(distance) / self.linear_speed

        direction = 1.0 if distance >= 0 else -1.0

        self.get_logger().info(
            f"Move {distance:.2f} m "
            f"({duration:.2f} sec)"
        )

        start_time = time.time()

        while time.time() - start_time < duration:

            self.publish_velocity(
                linear_x=direction * self.linear_speed,
                angular_z=0.0
            )

            time.sleep(0.05)

        self.stop_robot()

    # ============================================================
    # Rotate
    # ============================================================

    def rotate(self, angle):

        duration = abs(angle) / self.angular_speed

        direction = 1.0 if angle >= 0 else -1.0

        self.get_logger().info(
            f"Rotate {math.degrees(angle):.1f} deg "
            f"({duration:.2f} sec)"
        )

        start_time = time.time()

        while time.time() - start_time < duration:

            self.publish_velocity(
                linear_x=0.0,
                angular_z=direction * self.angular_speed
            )

            time.sleep(0.05)

        self.stop_robot()

    # ============================================================
    # Experiment 1
    # A → B → A
    # ============================================================

    def experiment_A_B_A(self):

        self.get_logger().info(
            "========== Experiment: A → B → A =========="
        )

        # A → B
        self.get_logger().info("A → B")
        self.move_forward(3.0)

        time.sleep(2.0)

        # B → A
        self.get_logger().info("B → A")
        self.move_forward(-3.0)

        self.stop_robot()

        self.get_logger().info(
            "========== Experiment Finished =========="
        )

    # ============================================================
    # Experiment 2
    # A → C → A
    # ============================================================

    def experiment_A_C_A(self):

        self.get_logger().info(
            "========== Experiment: A → C → A =========="
        )

        # A → C
        #
        # A(10,1) → C(15,4)
        #
        # dx = 5
        # dy = 3
        #
        # angle = atan2(3,5)
        # distance = sqrt(5² + 3²)

        angle = math.atan2(3.0, 5.0)
        distance = math.sqrt(5.0**2 + 3.0**2)

        self.get_logger().info(
            f"A → C: distance={distance:.2f} m, "
            f"angle={math.degrees(angle):.2f} deg"
        )

        # 轉向 C
        self.rotate(angle)

        # 前進到 C
        self.move_forward(distance)

        time.sleep(2.0)

        # C → A
        #
        # 反向旋轉回 A
        self.rotate(math.pi)

        # 回到 A
        self.move_forward(distance)

        # 恢復原本方向
        self.rotate(math.pi - angle)

        self.stop_robot()

        self.get_logger().info(
            "========== Experiment Finished =========="
        )

    # ============================================================
    # Experiment 3
    # A → B → C → B → A
    # ============================================================

    def experiment_A_B_C_B_A(self):

        self.get_logger().info(
            "========== Experiment: A → B → C → B → A =========="
        )

        # ----------------------------------------
        # A → B
        # ----------------------------------------

        self.get_logger().info("A → B")

        self.move_forward(3.0)

        time.sleep(1.0)

        # ----------------------------------------
        # B → C
        # ----------------------------------------
        #
        # B → C 是右轉 90 度
        #

        self.get_logger().info("B → C")

        self.rotate(math.pi / 2)

        self.move_forward(5.0)

        time.sleep(1.0)

        # ----------------------------------------
        # C → B
        # ----------------------------------------

        self.get_logger().info("C → B")

        self.rotate(math.pi)

        self.move_forward(5.0)

        time.sleep(1.0)

        # ----------------------------------------
        # B → A
        # ----------------------------------------

        self.get_logger().info("B → A")

        self.rotate(math.pi / 2)

        self.move_forward(3.0)

        self.stop_robot()

        self.get_logger().info(
            "========== Experiment Finished =========="
        )

    # ============================================================
    # Run selected experiment
    # ============================================================

    def run_experiment(self):

        # 確保一開始是停止狀態
        self.stop_robot()

        time.sleep(2.0)

        if self.EXPERIMENT == "A_B_A":

            self.experiment_A_B_A()

        elif self.EXPERIMENT == "A_C_A":

            self.experiment_A_C_A()

        elif self.EXPERIMENT == "A_B_C_B_A":

            self.experiment_A_B_C_B_A()

        else:

            self.get_logger().error(
                f"Unknown experiment: {self.EXPERIMENT}"
            )

        self.stop_robot()


def main(args=None):

    rclpy.init(args=args)

    node = CmdVelExperiment()

    try:

        node.run_experiment()

    except KeyboardInterrupt:

        node.get_logger().info(
            "Experiment interrupted."
        )

    finally:

        node.stop_robot()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()