import math

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped, PoseArray
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from tf2_geometry_msgs import do_transform_pose
from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.time import Time
import time
import tf_transformations
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

# Use ROS2 to navigate through poses and publish the robot's pose in the map frame. The code defines a `PosePublisher` class that updates and retrieves the robot's pose, a `wait_for_task` function to monitor navigation tasks, and a `LocalizationNode` class that listens for transforms and combines them to determine the robot's pose in the map frame. The `main` function initializes ROS2, sets up navigation goals based on an experiment type, and executes the navigation tasks while logging feedback.
class PosePublisher(Node):

    def __init__(self, tf_buffer, tf_listener):
        super().__init__('pose_publisher')
        self.tf_buffer = tf_buffer
        self.tf_listener = tf_listener
        self.pose = None

    def update_pose(self, trans):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = trans.transform.translation.x
        pose_msg.pose.position.y = trans.transform.translation.y
        pose_msg.pose.position.z = trans.transform.translation.z
        pose_msg.pose.orientation = trans.transform.rotation
        self.pose = pose_msg

    def get_pose(self):
        return self.pose

# def wait_for_task(navigator, timeout_s=180.0):
#     start = navigator.get_clock().now()
#     i = 0
    
#     while not navigator.isTaskComplete():
#         i += 1
#         feedback = navigator.getFeedback()
#         if feedback is not None:
#             # 顯示資訊 (每 5 次顯示一次)
#             if i % 5 == 0:
#                 print("----- NAV FEEDBACK -----")
#                 print("Elapsed:", (navigator.get_clock().now() - start).nanoseconds / 1e9, "s")
#                 try:
#                     print("Distance remaining:", feedback.distance_remaining)
#                 except AttributeError:
#                     print("No distance_remaining in feedback")
#                 print("Navigation time:", feedback.navigation_time.sec, "s")
#                 if hasattr(feedback, "current_pose"):
#                     cp = feedback.current_pose.pose.position
#                     print(f"Current pose: x={cp.x:.3f}, y={cp.y:.3f}")
#                 if hasattr(feedback, "current_behavior_tree"):
#                     print("BT:", feedback.current_behavior_tree)

#         rclpy.spin_once(navigator, timeout_sec=0.2)
        
#     # Add this line to correctly pass the result back to the main loop
#     return navigator.getResult()


def wait_for_task(navigator, timeout_s=80.0):
    start = navigator.get_clock().now()
    i = 0
    
    while not navigator.isTaskComplete():
        # 1. 取得當前經過的時間
        now = navigator.get_clock().now()
        elapsed_time = (now - start).nanoseconds / 1e9
        
        # 2. 檢查是否超時
        if elapsed_time > timeout_s:
            print(f"\n[強制完成] 導航時間已達 {elapsed_time:.1f} 秒，超過設定的 timeout ({timeout_s}s)。")
            print("視為任務已到達容許範圍，正在取消 Nav2 動作伺服器的剩餘目標...")
            # 必須呼叫 cancelTask，否則 Nav2 底層會一直卡在運算，影響下一次導航
            navigator.cancelTask()
            break
            
        i += 1
        feedback = navigator.getFeedback()
        if feedback is not None:
            # 顯示資訊 (每 5 次顯示一次)
            if i % 5 == 0:
                print("----- NAV FEEDBACK -----")
                print("Elapsed:", elapsed_time, "s")
                try:
                    print("Distance remaining:", feedback.distance_remaining)
                except AttributeError:
                    print("No distance_remaining in feedback")
                print("Navigation time:", feedback.navigation_time.sec, "s")
                if hasattr(feedback, "current_pose"):
                    cp = feedback.current_pose.pose.position
                    print(f"Current pose: x={cp.x:.3f}, y={cp.y:.3f}")
                if hasattr(feedback, "current_behavior_tree"):
                    print("BT:", feedback.current_behavior_tree)

        rclpy.spin_once(navigator, timeout_sec=0.2)
    return navigator.getResult()


def pause_navigation(t):
    time.sleep(t)


# ============================================================
# 你自己的 LocalizationNode
# ============================================================
# from your_package.localization_node import LocalizationNode
class LocalizationNode(Node):

    def __init__(self):
        super().__init__('localization_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.pose_publisher = PosePublisher(self.tf_buffer, self.tf_listener)

    def timer_callback(self):
        time_now = self.get_clock().now()
        seconds, nanoseconds = time_now.seconds_nanoseconds()
        target_time = Time(seconds=seconds, nanoseconds=nanoseconds) - Duration(seconds=1.0)

        map_to_odom = self.tf_buffer.lookup_transform(
            target_frame='map',
            source_frame='odom',
            time=Time(),
            timeout=Duration(seconds=2.0)
        )

        odom_to_base_link = self.tf_buffer.lookup_transform(
            target_frame='odom',
            source_frame='base_link',
            time=Time(),
            timeout=Duration(seconds=2.0)
        )

        combined_transform = self.combine_transforms(
            map_to_odom,
            odom_to_base_link
        )

        self.print_transform(combined_transform)

        if combined_transform:
            self.pose_publisher.update_pose(combined_transform)

    def combine_transforms(self, transform1, transform2):

        t1_translation = transform1.transform.translation
        t1_rotation = transform1.transform.rotation

        t2_translation = transform2.transform.translation
        t2_rotation = transform2.transform.rotation

        t1_matrix = tf_transformations.translation_matrix(
            [
                t1_translation.x,
                t1_translation.y,
                t1_translation.z
            ]
        )

        t1_rotation_matrix = tf_transformations.quaternion_matrix(
            [
                t1_rotation.x,
                t1_rotation.y,
                t1_rotation.z,
                t1_rotation.w
            ]
        )

        t1_matrix = tf_transformations.concatenate_matrices(
            t1_matrix,
            t1_rotation_matrix
        )

        t2_matrix = tf_transformations.translation_matrix(
            [
                t2_translation.x,
                t2_translation.y,
                t2_translation.z
            ]
        )

        t2_rotation_matrix = tf_transformations.quaternion_matrix(
            [
                t2_rotation.x,
                t2_rotation.y,
                t2_rotation.z,
                t2_rotation.w
            ]
        )

        t2_matrix = tf_transformations.concatenate_matrices(
            t2_matrix,
            t2_rotation_matrix
        )

        combined_matrix = tf_transformations.concatenate_matrices(
            t1_matrix,
            t2_matrix
        )

        combined_transform = transform1

        (
            combined_transform.transform.translation.x,
            combined_transform.transform.translation.y,
            combined_transform.transform.translation.z
        ) = tf_transformations.translation_from_matrix(
            combined_matrix
        )

        (
            combined_transform.transform.rotation.x,
            combined_transform.transform.rotation.y,
            combined_transform.transform.rotation.z,
            combined_transform.transform.rotation.w
        ) = tf_transformations.quaternion_from_matrix(
            combined_matrix
        )

        return combined_transform

    def print_transform(self, trans):

        translation = trans.transform.translation
        rotation = trans.transform.rotation

        rot_matrix = tf_transformations.quaternion_matrix(
            [
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w
            ]
        )

        self.get_logger().info(
            f'At time {trans.header.stamp.sec}.'
            f'{trans.header.stamp.nanosec}'
        )

        self.get_logger().info(
            f'- Translation: '
            f'[{translation.x:.3f}, '
            f'{translation.y:.3f}, '
            f'{translation.z:.3f}]'
        )

        self.get_logger().info(
            f'- Rotation: in Quaternion '
            f'[{rotation.x:.3f}, '
            f'{rotation.y:.3f}, '
            f'{rotation.z:.3f}, '
            f'{rotation.w:.3f}]'
        )

        rpy_rad = tf_transformations.euler_from_quaternion(
            [
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w
            ]
        )

        rpy_deg = [
            angle * 180.0 / 3.141592653589793
            for angle in rpy_rad
        ]

        self.get_logger().info(
            f'- Rotation: in RPY (radian) '
            f'[{rpy_rad[0]:.3f}, '
            f'{rpy_rad[1]:.3f}, '
            f'{rpy_rad[2]:.3f}]'
        )

        self.get_logger().info(
            f'- Rotation: in RPY (degree) '
            f'[{rpy_deg[0]:.3f}, '
            f'{rpy_deg[1]:.3f}, '
            f'{rpy_deg[2]:.3f}]'
        )

        self.get_logger().info('- Matrix:')

        for row in rot_matrix:
            self.get_logger().info(
                f' {row[0]:.3f} '
                f'{row[1]:.3f} '
                f'{row[2]:.3f} '
                f'{row[3]:.3f}'
            )


def create_pose(navigator, x, y, qz=0.0, qw=1.0):
    """
    建立一個 map frame 下的 PoseStamped
    """

    pose = PoseStamped()

    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()

    pose.pose.position.x = x
    pose.pose.position.y = y

    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw

    return pose


def get_yaw_from_quaternion(q):
    """將四元數轉換為 Yaw (弧度)"""
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def absolute_spin(navigator, tf_buffer, target_yaw_rad):
    """
    使用 TF2 取得絕對座標並計算所需旋轉角度
    """
    try:
        now = rclpy.time.Time()

        trans = tf_buffer.lookup_transform(
            'map',
            'base_link',
            now,
            timeout=rclpy.duration.Duration(seconds=1.0)
        )

        current_yaw = get_yaw_from_quaternion(
            trans.transform.rotation
        )

    except Exception as e:
        print(f"[Absolute Spin] 取得 TF 失敗: {e}")
        return

    diff_yaw = target_yaw_rad - current_yaw

    diff_yaw = math.atan2(
        math.sin(diff_yaw),
        math.cos(diff_yaw)
    )

    print(
        f"[Absolute Spin] "
        f"目前朝向: {math.degrees(current_yaw):.1f}° | "
        f"目標朝向: {math.degrees(target_yaw_rad):.1f}°"
    )

    print(
        f"[Absolute Spin] "
        f"正在執行精準補償旋轉: {math.degrees(diff_yaw):.1f}°"
    )

    navigator.spin(diff_yaw)

    while not navigator.isTaskComplete():
        rclpy.spin_once(navigator, timeout_sec=0.1)

def rotate_to_yaw(navigator, tf_buffer, target_yaw):
    from geometry_msgs.msg import Twist
    cmd_pub = navigator.create_publisher(Twist, '/cmd_vel', 10)

    rate = navigator.create_rate(20)

    kp = 1.0
    max_wz = 0.5
    yaw_tolerance = math.radians(2.0)

    while rclpy.ok():

        try:
            now = rclpy.time.Time()

            trans = tf_buffer.lookup_transform(
                'map',
                'base_link',
                now,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            current_yaw = get_yaw_from_quaternion(
                trans.transform.rotation
            )

        except Exception:
            continue

        # 計算最短角度差 [-pi, pi]
        error = target_yaw - current_yaw
        error = math.atan2(
            math.sin(error),
            math.cos(error)
        )

        print(
            f"current={math.degrees(current_yaw):.1f}°, "
            f"target={math.degrees(target_yaw):.1f}°, "
            f"error={math.degrees(error):.1f}°"
        )

        # 到達目標
        if abs(error) < yaw_tolerance:
            break

        twist = Twist()
        twist.linear.x = 0.0

        # P controller
        wz = kp * error
        wz = max(-max_wz, min(max_wz, wz))

        twist.angular.z = wz

        cmd_pub.publish(twist)

        rclpy.spin_once(navigator, timeout_sec=0.05)

    # 停止
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    cmd_pub.publish(twist)

    print("Rotation finished")


def main():
    # 重點： 需要先讓自己在A點，並且在 map frame 下的 PoseStamped 也要對應到 A 點
    rclpy.init()

    navigator = BasicNavigator()
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, navigator)

    for _ in range(10):
        rclpy.spin_once(navigator, timeout_sec=0.1)

    # 建立 LocalizationNode
    localization_node = LocalizationNode()

    # 等待 Nav2 啟動
    navigator.waitUntilNav2Active()

    # ========================================
    # 實驗設定
    # ========================================
    EXPERIMENT = "A_C_A"

    # A、B、C 座標
    # A = create_pose(navigator, 0.0, 0.0, 0.0, 1.0)
    # B = create_pose(navigator, 1.81016, -0.304, 0.0, 1.0)
    # # C = create_pose(navigator, 1.572, -3.11, -0.73, 0.67)
    # C = create_pose(navigator, 1.607, -2.862, -0.73, 0.67)

    # # B_turn_to_C: 停在 B 的位置，但將車頭轉向 C 的角度
    # B_turn_to_C = create_pose(navigator, 1.81016, -0.304, -0.73, 0.67)
    # B_facing_C = create_pose(navigator, 1.81016, -0.304, -0.73, 0.67) # 你的 C 點朝向

    # C = create_pose(navigator, 0.0, 0.0, 0.0, 1.0)
    # B = create_pose(navigator, 1.81016, -0.304, 0.68368, 0.7297)
    # A = create_pose(navigator, 1.5534, -3.134, 0.68368, 0.7297)

    A = create_pose(
        navigator,
        -3.4755,
        0.3025,
        -0.2607258865,
        0.9654128713
    )

    B = create_pose(
        navigator,
        -2.2122,
        -1.2800,
        -0.306,
        0.952
    )

    # C = create_pose(
    #     navigator,
    #     -3.441579813321171,
    #     -3.634506656366109,
    #     -0.9006843216947186,
    #     0.4344741104523083
    # )

    C = create_pose(
        navigator,
        -3.430,
        -3.4419,
        -0.87,
        0.49
    )
    B_turn_to_C = create_pose(navigator, -1.98351, -1.01548, -0.9006843216947186,0.4344741104523083)
    C_turn_to_B = create_pose(
        navigator,
        -3.430,
        -3.4419,
        0.515,
        0.857
    )
    B_oppo_C = create_pose(
        navigator,
        -2.2122,
        -1.2800,
        0.457,
        0.890
    )
    B_oppo_A = create_pose(
        navigator,
        -2.2122,
        -1.2800,
        0.952,
        0.306
    )
    A_turn_back = create_pose(
        navigator,
        -3.4755,
        0.3025,
        0.9654128713,
        0.2607258865
    )


    # ========================================
    # 根據實驗選擇路徑
    # ========================================
    if EXPERIMENT == "A_B_A": # 走直線

        goals = [
            ("A → B", B),
            ("B → A", A),
        ]

    elif EXPERIMENT == "A_C_A": # 走斜的

        goals = [
            ("A → C", C),
            ("C → A", A),
        ]

    elif EXPERIMENT == "A_B_C_B_A":

        goals = [
            ("A → B", B),
            ("B 原地轉向 (對準 C)", B_turn_to_C),
            ("B → C", C),
            ("C 原地轉向 (對準 B)", C_turn_to_B),
            ("C → B_oppo_C", B_oppo_C),
            ("B 原地轉向 (對準 A)", B),
            ("B 原地轉向 (對準 A 的反方向)", B_oppo_A),
            ("B → A", A_turn_back),
            ("A 原地轉向", A),
        ]

    else:
        print(f"Unknown experiment: {EXPERIMENT}")
        return

    # ========================================
    # 執行實驗
    # ========================================
    YAW_C = -1.713  # 請換成你 C 點實際的 Yaw 值
    YAW_A = 0.0      # A 點與 B 點的初始朝向是 0 度
    for name, goal in goals:

        print(f"\n========== {name} ==========")

        if name == "B 原地轉向 (對準 C)":
            target_yaw = get_yaw_from_quaternion(C.pose.orientation)
            rotate_to_yaw(navigator, tf_buffer, target_yaw)
            time.sleep(2)
            continue

        elif name == "B 原地轉向 (對準 A)":
            target_yaw = get_yaw_from_quaternion(A.pose.orientation)
            rotate_to_yaw(navigator, tf_buffer, target_yaw)
            time.sleep(2)
            continue
        elif name == "C 原地轉向 (對準 B)":
            target_yaw = get_yaw_from_quaternion(C_turn_to_B.pose.orientation)
            rotate_to_yaw(navigator, tf_buffer, target_yaw)
            time.sleep(2)
            continue
        elif name == "B 原地轉向 (對準 A)":
            target_yaw = get_yaw_from_quaternion(B.pose.orientation)
            rotate_to_yaw(navigator, tf_buffer, target_yaw)
            time.sleep(2)
            continue
        elif name == "B 原地轉向 (對準 A 的反方向)":
            target_yaw = get_yaw_from_quaternion(B_oppo_A.pose.orientation)
            rotate_to_yaw(navigator, tf_buffer, target_yaw)
            time.sleep(2)
            continue
        elif name == "A 原地轉向":
            target_yaw = get_yaw_from_quaternion(A.pose.orientation)
            rotate_to_yaw(navigator, tf_buffer, target_yaw)
            time.sleep(2)
            continue
        else:
            navigator.goToPose(goal)

        result = wait_for_task(navigator)

        if result == TaskResult.SUCCEEDED:
            print(f"{name} succeeded")
            input(
            f"\nRobot reached {name}.\n"
            "Measure x/y/yaw error now.\n"
            "Press ENTER to continue..."
        )
        else:
            print(f"{name} failed")
            input(
                f"\nRobot reached {name}.\n"
                "Measure x/y/yaw error now.\n"
                "Press ENTER to continue..."
            )
            

        time.sleep(2)
    print("\n========== Experiment Finished ==========")

    navigator.lifecycleShutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()