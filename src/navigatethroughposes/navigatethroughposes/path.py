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

def get_fresh_yaw(navigator, tf_buffer, max_tf_age_s=0.5):
    """讀取最新 TF；使用 navigator 的 ROS clock 比較時間戳。"""
    trans = tf_buffer.lookup_transform('map', 'base_link', Time())
    stamp = trans.header.stamp
    stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
    age_s = (navigator.get_clock().now().nanoseconds - stamp_ns) / 1e9
    if stamp_ns <= 0 or age_s > max_tf_age_s or age_s < -0.05:
        raise ValueError(f"TF 時間戳無效或過舊: age={age_s:.3f}s")
    yaw = get_yaw_from_quaternion(trans.transform.rotation)
    if not math.isfinite(yaw):
        raise ValueError("TF yaw 不是有限數值")
    return yaw


def rotate_to_yaw(
    navigator, tf_buffer, target_yaw=None, *, relative_angle_deg=None,
    timeout_s=30.0, tf_failure_timeout_s=2.0, max_tf_age_s=0.5,
):
    """旋轉完成或一般旋轉超時回傳絕對目標 yaw；TF／其他失敗回傳 None。

    絕對 target_yaw 與相對 relative_angle_deg 二擇一。相對目標在
    本函式處理 callbacks 並取得新鮮 TF 後才計算，避免 input 後使用舊姿態。
    總超時及 TF 連續失敗超時使用 monotonic clock，不受 ROS 時間暫停影響。
    """
    from geometry_msgs.msg import Twist

    cmd_pub = navigator.create_publisher(Twist, '/cmd_vel_filtered', 10)
    start = time.monotonic()
    tf_failure_start = None
    kp = 1.0
    max_wz = 0.5
    yaw_tolerance = math.radians(2.0)

    try:
        if (target_yaw is None) == (relative_angle_deg is None):
            raise ValueError("請指定絕對 yaw 或相對角度其中一個")
        angle = target_yaw if target_yaw is not None else relative_angle_deg
        if not math.isfinite(angle):
            raise ValueError("旋轉角度必須為有限數值")
        if not all(math.isfinite(v) and v > 0 for v in (
                timeout_s, tf_failure_timeout_s, max_tf_age_s)):
            raise ValueError("超時及 TF 新鮮度門檻必須為正數")

        # 暫停量測後先處理排隊的 TF callbacks；此階段只發送停止命令。
        for _ in range(5):
            if not rclpy.ok():
                return None
            cmd_pub.publish(Twist())
            rclpy.spin_once(navigator, timeout_sec=0.1)

        while rclpy.ok():
            if time.monotonic() - start >= timeout_s:
                # 原本的處理：旋轉超時即終止整個任務。
                # print(f"Rotation timeout ({timeout_s}s), stop mission")
                # return None

                # 改為停止本次旋轉並繼續任務；TF 無效仍視為失敗。
                cmd_pub.publish(Twist())
                current_yaw = get_fresh_yaw(navigator, tf_buffer, max_tf_age_s)
                if target_yaw is None:
                    print("Rotation target unavailable, stop mission")
                    return None
                error = math.atan2(
                    math.sin(target_yaw - current_yaw),
                    math.cos(target_yaw - current_yaw),
                )
                print(
                    f"Rotation timeout ({timeout_s}s), "
                    f"remaining error={math.degrees(error):.1f}°, "
                    "stop rotation and continue mission"
                )
                # 保留原定路段朝向，不將未轉完的實際朝向設為下一點目標。
                return target_yaw

            # 先更新 TF buffer，再做非阻塞查詢，避免等待 TF 時堵住 callback。
            rclpy.spin_once(navigator, timeout_sec=0.05)
            try:
                current_yaw = get_fresh_yaw(navigator, tf_buffer, max_tf_age_s)
            except Exception as exc:
                cmd_pub.publish(Twist())
                if tf_failure_start is None:
                    tf_failure_start = time.monotonic()
                    print(f"Rotation TF unavailable, stopping: {exc}")
                if time.monotonic() - tf_failure_start >= tf_failure_timeout_s:
                    print("Rotation TF recovery timeout, stop mission")
                    return None
                continue

            tf_failure_start = None
            if target_yaw is None:
                target_yaw = current_yaw + math.radians(relative_angle_deg)
            target_yaw = math.atan2(math.sin(target_yaw), math.cos(target_yaw))
            error = math.atan2(
                math.sin(target_yaw - current_yaw),
                math.cos(target_yaw - current_yaw),
            )
            print(
                f"current={math.degrees(current_yaw):.1f}°, "
                f"target={math.degrees(target_yaw):.1f}°, "
                f"error={math.degrees(error):.1f}°"
            )
            if abs(error) < yaw_tolerance:
                print("Rotation finished")
                return target_yaw

            twist = Twist()
            twist.angular.z = max(-max_wz, min(max_wz, kp * error))
            cmd_pub.publish(twist)

        return None
    except Exception as exc:
        print(f"Rotation failed, stop mission: {exc}")
        return None
    finally:
        # 包含正常完成、超時、例外及 KeyboardInterrupt，皆嘗試送出零速度。
        # ROS context 若已關閉，發布可能失敗；不可讓清理錯誤蓋過原始例外。
        try:
            cmd_pub.publish(Twist())
        except Exception as exc:
            print(f"Unable to publish rotation stop: {exc}")
        finally:
            try:
                navigator.destroy_publisher(cmd_pub)
            except Exception as exc:
                print(f"Unable to destroy rotation publisher: {exc}")


def wait_for_task_until_close(
    navigator,
    tf_buffer,
    goal,
    pos_tol=0.08,
    yaw_tol=math.radians(8.0),
    timeout_s=60.0
):
    import time
    import math
    import rclpy
    from nav2_simple_commander.robot_navigator import TaskResult

    start = time.time()

    while rclpy.ok():

        if time.time() - start > timeout_s:
            print("Navigation timeout, cancel task")
            navigator.cancelTask()
            pause_navigation(1.0)
            return None

        if navigator.isTaskComplete():
            result = navigator.getResult()
            pause_navigation(1.0)
            return result

        try:
            trans = tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            x = trans.transform.translation.x
            y = trans.transform.translation.y
            current_yaw = get_yaw_from_quaternion(trans.transform.rotation)

            target_x = goal.pose.position.x
            target_y = goal.pose.position.y
            target_yaw = get_yaw_from_quaternion(goal.pose.orientation)

            dx = x - target_x
            dy = y - target_y
            dist = math.sqrt(dx * dx + dy * dy)

            yaw_error = target_yaw - current_yaw
            yaw_error = math.atan2(
                math.sin(yaw_error),
                math.cos(yaw_error)
            )

            print(
                f"dist_to_goal={dist:.3f} m, "
                f"yaw_error={math.degrees(yaw_error):.1f}°"
            )

            if dist < pos_tol and abs(yaw_error) < yaw_tol:
                print(
                    f"Close enough: dist={dist:.3f} m < {pos_tol:.3f} m, "
                    f"yaw_error={math.degrees(yaw_error):.1f}° < {math.degrees(yaw_tol):.1f}°, "
                    "cancel Nav2 and stop"
                )
                navigator.cancelTask()
                pause_navigation(1.0)
                return TaskResult.SUCCEEDED

        except Exception as e:
            print(f"wait close TF failed: {e}")

        rclpy.spin_once(navigator, timeout_sec=0.05)

# map frame 的固定點位，格式為 (x, y)，單位為公尺。
# D、E、F 目前僅為示範假座標，不代表實際路線幾何；實機執行前請替換。
WAYPOINTS = {
    "A": (-3.5211, -0.0608),
    "B": (-2.1679, -0.7594),
    "C": (2.7608, -3.0738),
    "D": (1.3589, -6.0476),  # TODO: 替換為 D 的實際座標
    "E": (-2.9595, -3.7342),  # TODO: 替換為 E 的實際座標
    "F": (-4.7877, -2.8884),  # TODO: 替換為 F 的實際座標
}

# 保留原本 A 的起始朝向。啟動前，機器人必須位於 A 並朝此方向。
INITIAL_YAW = math.atan2(
    2.0 * 0.9636 * -0.2672,
    1.0 - 2.0 * (-0.2672) ** 2,
)

# (動作, 點位, 相對旋轉角度)：正值向左、負值向右，單位為度。
# A 是起點，不重複導航至 A；rotate 必須在指定點位到達後執行。
MISSIONS = {
    "1": {
        "name": "任務一 A-B-C-D-C-B-A",
        "steps": (
            ("move", "B", 0),
            ("move", "C", 0),
            ("rotate", "C", -90),
            ("move", "D", 0),
            ("rotate", "D", 180),
            ("move", "C", 0),
            ("rotate", "C", 90),
            ("move", "B", 0),
            ("move", "A", 0),
        ),
    },
    "2": {
        "name": "任務二 A-F-E-D-E-F-A",
        "steps": (
            ("rotate", "A", -90),
            ("move", "F", 0),
            ("rotate", "F", 90),
            ("move", "E", 0),
            ("move", "D", 0),
            ("rotate", "D", 180),
            ("move", "E", 0),
            ("move", "F", 0),
            ("rotate", "F", -90),
            ("move", "A", 0),
        ),
    },
}


def validate_waypoints(mission, waypoints):
    """在啟動導航前檢查任務所需座標。"""
    required = {"A"} | {point for _, point, _ in mission["steps"]}
    for point in sorted(required):
        coordinates = waypoints.get(point)
        if (coordinates is None or len(coordinates) != 2
                or not all(math.isfinite(value) for value in coordinates)):
            raise ValueError(f"請先設定 WAYPOINTS[{point!r}] 的實際 (x, y) 座標")


def pause_for_measurement(point):
    """在點位量測誤差，按 Enter 後才允許下一個移動或旋轉步驟。"""
    input(
        f"\n目前位於 {point}，請量測 x/y/yaw 誤差。\n"
        "若下一步需要旋轉，將於量測結束後執行。\n"
        "按 ENTER 繼續..."
    )


def execute_mission(navigator, tf_buffer, mission, waypoints):
    """依序導航及原地旋轉；任一步驟失敗就停止，不跳到下一點。"""
    validate_waypoints(mission, waypoints)
    current_point = "A"
    travel_yaw = INITIAL_YAW
    # 起點也先量測，尤其任務二必須在 A 的第一次旋轉前暫停。
    pause_for_measurement(current_point)

    for action, point, angle_deg in mission["steps"]:
        if not rclpy.ok():
            return False

        if action == "rotate":
            if point != current_point:
                raise ValueError(f"旋轉點 {point} 與目前點位 {current_point} 不符")
            print(f"\n========== {point} 原地旋轉 {angle_deg:+}° ==========")
            # 在量測結束後，由旋轉函式取得新鮮朝向並計算相對目標。
            reached_yaw = rotate_to_yaw(
                navigator, tf_buffer, relative_angle_deg=angle_deg,
            )
            if reached_yaw is None or not rclpy.ok():
                return False
            travel_yaw = reached_yaw

        elif action == "move":
            x, y = waypoints[point]
            # 到達時保留此路段朝向。轉彎另由 rotate_to_yaw 執行，
            # 不把下一路段朝向放進轉彎點的導航目標。
            goal = create_pose(
                navigator, x, y,
                math.sin(travel_yaw / 2.0), math.cos(travel_yaw / 2.0),
            )
            print(f"\n========== {current_point} → {point} ==========")
            navigator.goToPose(goal)
            result = wait_for_task_until_close(
                navigator, tf_buffer, goal,
                pos_tol=0.08, yaw_tol=math.radians(10.0), timeout_s=60.0,
            )
            if result != TaskResult.SUCCEEDED:
                print(f"前往 {point} 失敗或逾時，停止任務")
                return False
            current_point = point
            # 到點後先量測；下一輪才執行旋轉或前往下一點。
            # 回程重訪點位及最後回到 A 時也會暫停。
            pause_for_measurement(current_point)
        else:
            raise ValueError(f"未知的任務動作: {action}")

    return True


def main():
    selection = input("選擇任務 (1: A-B-C-D / 2: A-F-E-D): ").strip()
    mission = MISSIONS.get(selection)
    if mission is None:
        print(f"未知任務: {selection}，請輸入 1 或 2")
        return

    try:
        validate_waypoints(mission, WAYPOINTS)
    except ValueError as exc:
        print(exc)
        return

    # 啟動前請先將機器人定位於 A，朝向 INITIAL_YAW。
    rclpy.init()
    navigator = BasicNavigator()
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, navigator)
    try:
        for _ in range(10):
            rclpy.spin_once(navigator, timeout_sec=0.1)
        navigator.waitUntilNav2Active()
        print(f"開始 {mission['name']}")
        succeeded = execute_mission(navigator, tf_buffer, mission, WAYPOINTS)
        print("任務完成" if succeeded else "任務已停止，未完成完整路線")
    finally:
        if rclpy.ok():
            navigator.cancelTask()
            navigator.lifecycleShutdown()
        navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
