#!/usr/bin/env python3
"""RTAB-Map TF + Nav2：任務 laptop 到 laptop；任務 between 到 Before_between 後右轉，以 Twist 到 between。

先啟動 RTAB-Map 定位及 Nav2，確認 map -> odom -> base_link 存在。
本程式不設定 AMCL initialpose，也不負責啟動或關閉導航系統。
座標是虛擬值，實機使用前請修改 WAYPOINTS 或 ROS parameters。
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


# 直接在此選擇任務，不需要透過 --ros-args 傳入：
# 'laptop'：Nav2 到 laptop。
# 'between'：Nav2 到 Before_between，右轉 90 度，再以 cmd_vel 到 between。
mission = 'laptop'


# 格式：(x, y, qz, qw)，位置單位 m；四元數 qx、qy 固定為 0。
# 朝向以 map 為準：(qz, qw)=(0, 1) 為 0°，(-0.7071, 0.7071) 為 -90°。
# Before_between 的朝向是右轉前的到點朝向；between 是最後停止時的朝向。
WAYPOINTS = {
    # 'laptop': (0.782, -2.304, -0.499, 0.866),
    'laptop': (0.554, -2.3344, -0.22, 0.9738),
    'Before_between': (2.0, 0.0, 0.0, 1.0),
    'between': (2.0, -1.0, -0.70710678, 0.70710678),
}


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def get_yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def create_pose(navigator, x, y, qz, qw, frame='map'):
    """沿用 path.py 的 PoseStamped 寫法，以四元數設定到點朝向。"""
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


class PureNavigation:
    def __init__(self, navigator):
        self.navigator = navigator
        defaults = {
            'map_frame': 'map', 'base_frame': 'base_link',
            # 沿用 path.py：collision_monitor 會再輸出至 /cmd_vel。
            'cmd_vel_topic': '/cmd_vel_filtered',
            'linear_speed': 0.15, 'angular_speed': 0.4,
            'position_tolerance': 0.08, 'yaw_tolerance_deg': 2.0,
            'nav_timeout': 180.0, 'motion_timeout': 60.0,
            'tf_timeout': 2.0, 'max_tf_age': 0.5,
        }
        for name, (x, y, qz, qw) in WAYPOINTS.items():
            defaults[f'{name.lower()}_x'] = x
            defaults[f'{name.lower()}_y'] = y
            defaults[f'{name.lower()}_qz'] = qz
            defaults[f'{name.lower()}_qw'] = qw
        self.config = {
            key: navigator.declare_parameter(key, value).value
            for key, value in defaults.items()
        }
        self.config['mission'] = mission
        self.validate()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, navigator)
        self.cmd_pub = navigator.create_publisher(
            Twist, self.config['cmd_vel_topic'], 10)

    def validate(self):
        if self.config['mission'] not in ('laptop', 'between'):
            raise ValueError('mission 必須為 laptop 或 between')
        for key, value in self.config.items():
            if isinstance(value, (float, int)) and not math.isfinite(value):
                raise ValueError(f'{key} 必須是有限數值')
        for key in ('linear_speed', 'angular_speed', 'position_tolerance',
                    'yaw_tolerance_deg', 'nav_timeout', 'motion_timeout',
                    'tf_timeout', 'max_tf_age'):
            if self.config[key] <= 0:
                raise ValueError(f'{key} 必須大於零')
        for name in WAYPOINTS:
            self.quaternion(name)  # 啟動前驗證四元數，拒絕零長度或無效值。
        if math.dist(self.point('Before_between'), self.point('between')) <= self.config['position_tolerance']:
            raise ValueError('Before_between、between 距離必須大於到點容許誤差')

    def point(self, name):
        return (self.config[f'{name.lower()}_x'],
                self.config[f'{name.lower()}_y'])

    def quaternion(self, name):
        qz = self.config[f'{name.lower()}_qz']
        qw = self.config[f'{name.lower()}_qw']
        norm = math.hypot(qz, qw)
        if not math.isfinite(norm) or norm < 1e-12:
            raise ValueError(f'{name} 的四元數必須有效，qz、qw 不可同時為零')
        return qz / norm, qw / norm  # 容許輸入經小數截斷的四元數。

    def yaw(self, name):
        qz, qw = self.quaternion(name)
        return normalize_angle(2.0 * math.atan2(qz, qw))

    def stop(self):
        if rclpy.ok():
            self.cmd_pub.publish(Twist())

    def get_pose(self):
        """查詢最新 map->base TF；過期 TF 不可用來繼續發送速度。"""
        trans = self.tf_buffer.lookup_transform(
            self.config['map_frame'], self.config['base_frame'], Time())
        stamp = trans.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        age = (self.navigator.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if stamp_ns <= 0 or not -0.05 <= age <= self.config['max_tf_age']:
            raise ValueError(f'TF 過期或時間無效：age={age:.3f}s')
        pos = trans.transform.translation
        result = (pos.x, pos.y, get_yaw_from_quaternion(trans.transform.rotation))
        if not all(math.isfinite(value) for value in result):
            raise ValueError('TF 位姿不是有限數值')
        return result

    def wait_for_pose(self):
        deadline = time.monotonic() + self.config['tf_timeout']
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self.navigator, timeout_sec=0.05)
            try:
                return self.get_pose()
            except (TransformException, ValueError):
                pass
        raise RuntimeError('無法取得新鮮的 map -> base TF，請確認 RTAB-Map 定位')

    def navigate(self, name):
        self.navigator.get_logger().info(f'Nav2 前往 {name}: {self.point(name)}')
        goal = create_pose(self.navigator, *self.point(name), *self.quaternion(name),
                           frame=self.config['map_frame'])
        if not self.navigator.goToPose(goal):
            raise RuntimeError(f'Nav2 拒絕前往 {name}')
        deadline = time.monotonic() + self.config['nav_timeout']
        while rclpy.ok():
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                if result != TaskResult.SUCCEEDED:
                    reason = {
                        TaskResult.FAILED: 'Nav2 回報 FAILED（導航失敗）',
                        TaskResult.CANCELED: 'Nav2 回報 CANCELED（導航被取消）',
                    }.get(result, f'Nav2 回報非成功結果：{result}')
                    raise RuntimeError(
                        f'前往 {name}：{reason}；'
                        '請查看同時段 controller_server、planner_server、'
                        'behavior_server 與 collision_monitor 日誌')
                return
            if time.monotonic() >= deadline:
                self.navigator.cancelTask()
                raise RuntimeError(f'前往 {name} 超時')
            rclpy.spin_once(self.navigator, timeout_sec=0.05)
        raise RuntimeError('ROS 已關閉')

    def control_motion(self, command):
        """command(pose) 回傳 Twist；回傳 None 代表到達，失敗則停止任務。"""
        deadline = time.monotonic() + self.config['motion_timeout']
        tf_failure_start = None
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                cycle_start = time.monotonic()
                rclpy.spin_once(self.navigator, timeout_sec=0.01)
                try:
                    pose = self.get_pose()
                except (TransformException, ValueError) as exc:
                    self.stop()
                    if tf_failure_start is None:
                        tf_failure_start = time.monotonic()
                    if time.monotonic() - tf_failure_start >= self.config['tf_timeout']:
                        raise RuntimeError(f'TF 持續無效：{exc}') from exc
                else:
                    tf_failure_start = None
                    twist = command(pose)
                    if twist is None:
                        return
                    self.cmd_pub.publish(twist)
                # 固定約 20 Hz，避免 TF callbacks 很多時無限制發布。
                time.sleep(max(0.0, 0.05 - (time.monotonic() - cycle_start)))
            raise RuntimeError('cmd_vel 動作超時或 ROS 已關閉')
        finally:
            self.stop()

    def rotate_right(self):
        # 以 Before_between 到點後的實際朝向計算相對右轉 90 度。
        target = normalize_angle(self.wait_for_pose()[2] - math.pi / 2.0)
        self.navigator.get_logger().info('cmd_vel 原地右轉 90 度')

        def command(pose):
            error = normalize_angle(target - pose[2])
            if abs(error) <= math.radians(self.config['yaw_tolerance_deg']):
                return None
            twist = Twist()
            limit = self.config['angular_speed']
            twist.angular.z = max(-limit, min(limit, error))
            return twist

        self.control_motion(command)

    def drive_to_between(self):
        """以 map TF 到 between 並對準指定朝向；無路徑規劃。"""
        x, y = self.point('between')
        target_yaw = self.yaw('between')
        self.navigator.get_logger().info(f'cmd_vel 前往 between: {(x, y)}')

        def command(pose):
            dx, dy = x - pose[0], y - pose[1]
            distance = math.hypot(dx, dy)
            if distance <= self.config['position_tolerance']:
                error = normalize_angle(target_yaw - pose[2])
                if abs(error) <= math.radians(self.config['yaw_tolerance_deg']):
                    return None
                twist = Twist()
                limit = self.config['angular_speed']
                twist.angular.z = max(-limit, min(limit, error))
                return twist
            error = normalize_angle(math.atan2(dy, dx) - pose[2])
            twist = Twist()
            limit = self.config['angular_speed']
            twist.angular.z = max(-limit, min(limit, error))
            if abs(error) < math.radians(15.0):
                twist.linear.x = min(self.config['linear_speed'], 0.5 * distance)
            return twist

        self.control_motion(command)

    def run(self):
        # Humble 的 waitUntilNav2Active 會等待 localizer 的 lifecycle。
        # RTAB-Map 非 AMCL lifecycle node，改等待 bt_navigator 並自行確認 TF。
        self.navigator.waitUntilNav2Active(localizer='bt_navigator')
        self.wait_for_pose()
        if self.config['mission'] == 'laptop':
            self.navigate('laptop')
        else:
            self.navigate('Before_between')
            # 等待速度平滑器先前的 Nav2 指令消退，再接手手動控制。
            deadline = time.monotonic() + 1.0
            while rclpy.ok() and time.monotonic() < deadline:
                self.stop()
                rclpy.spin_once(self.navigator, timeout_sec=0.05)
            self.rotate_right()
            self.drive_to_between()
        self.navigator.get_logger().info('任務完成')


def main(args=None):
    rclpy.init(args=args)
    navigator = BasicNavigator()
    mission = None
    try:
        mission = PureNavigation(navigator)
        mission.run()
    except KeyboardInterrupt:
        navigator.get_logger().info('使用者中止任務')
    except Exception as exc:
        navigator.get_logger().error(f'任務停止：{exc}')
        raise
    finally:
        try:
            if rclpy.ok():
                try:
                    navigator.cancelTask()
                finally:
                    if mission is not None:
                        mission.stop()
        finally:
            navigator.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
