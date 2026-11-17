#!/usr/bin/env python3
"""RTAB-Map TF + Nav2：任務 laptop 到 laptop；任務 between 到 Before_between 後以 Twist 直行指定距離。

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
# 'between'：Nav2 到 Before_between，再以 cmd_vel 直行兩個 waypoint 之間的距離。
mission = 'between'


# 格式：(x, y, qz, qw)，位置單位 m；四元數 qx、qy 固定為 0。
# 朝向以 map 為準：(qz, qw)=(0, 1) 為 0°，(-0.7071, 0.7071) 為 -90°。
# Before_between 的朝向是右轉前的到點朝向；between 是最後停止時的朝向。
WAYPOINTS = {
    'laptop': (0.782, -2.304, -0.499, 0.866),
    # 'laptop': (0.554, -2.3344, -0.22, 0.9738),
    # 'Before_between': (0.6532, -2.0126, -0.184, 0.982),
    'Before_between': (0.6532, -2.0126, -0.8823, 0.4705),
    'between': (0.1555, 3.184, -0.8823, 0.4705),
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
            'map_frame': 'map', 'base_frame': 'base_link', 'odom_frame': 'odom',
            # 直接送底盤；此直行／旋轉指令不經 collision_monitor。
            'cmd_vel_topic': '/cmd_vel',
            'linear_speed': 0.15, 'angular_speed': 0.4,
            'position_tolerance': 0.05, 'yaw_tolerance_deg': 2.0,
            'nav_timeout': 60.0, 'motion_timeout': 60.0,
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
        # 只有呼叫 TF 定位功能時才建立訂閱；定時直行不使用 TF。
        self.tf_buffer = None
        self.tf_listener = None
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

    def get_pose(self, frame=None):
        """查詢指定 frame 的最新位姿；預設 map，直行使用 odom。"""
        if self.tf_buffer is None:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self.navigator)
        frame = frame or self.config['map_frame']
        trans = self.tf_buffer.lookup_transform(
            frame, self.config['base_frame'], Time())
        stamp = trans.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        age = (self.navigator.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if stamp_ns <= 0 or not -0.05 <= age <= self.config['max_tf_age']:
            raise ValueError(
                f'{frame} -> {self.config["base_frame"]} TF 過期或時間無效：'
                f'age={age:.3f}s')
        pos = trans.transform.translation
        result = (pos.x, pos.y, get_yaw_from_quaternion(trans.transform.rotation))
        if not all(math.isfinite(value) for value in result):
            raise ValueError('TF 位姿不是有限數值')
        return result

    def wait_for_pose(self, pose_reader=None, timeout_s=None):
        """參考 tracer_lang_robot：限時持續處理 callbacks，排空舊 TF。"""
        pose_reader = pose_reader or self.get_pose
        timeout_s = self.config['tf_timeout'] if timeout_s is None else timeout_s
        deadline = time.monotonic() + timeout_s
        last_problem = '尚未收到 TF'
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self.navigator, timeout_sec=0.02)
            try:
                return pose_reader()
            except (TransformException, ValueError) as exc:
                # 等待新資料時保持停止；不在重試之間 sleep，繼續處理 TF callbacks。
                self.stop()
                last_problem = str(exc)
        raise RuntimeError(f'等待新鮮 TF 逾時：{last_problem}')

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
                if name == 'Before_between':
                    # 確認 Nav2 已結束，才讓後續 cmd_vel 接手。
                    cancel_deadline = time.monotonic() + 5.0
                    while rclpy.ok():
                        if self.navigator.isTaskComplete():
                            self.stop()
                            self.navigator.get_logger().warn(
                                f'前往 {name} 超過 {self.config["nav_timeout"]:.1f} 秒；'
                                'Nav2 已結束，未確認到點，從目前位置繼續直行')
                            return
                        if time.monotonic() >= cancel_deadline:
                            raise RuntimeError('Nav2 取消後 5 秒仍未結束，無法接手直行')
                        rclpy.spin_once(self.navigator, timeout_sec=0.05)
                    raise RuntimeError('ROS 已關閉')
                raise RuntimeError(f'前往 {name} 超時')
            rclpy.spin_once(self.navigator, timeout_sec=0.05)
        raise RuntimeError('ROS 已關閉')

    def drive_forward(self, distance_m):
        """只依距離 / 速度定時發布 cmd_vel，不查詢 TF 或 odom。"""
        if not math.isfinite(distance_m) or distance_m < 0:
            raise ValueError('前進距離必須為非負有限值')
        speed = self.config['linear_speed']
        if not math.isfinite(speed) or speed <= 0:
            raise ValueError('直行速度必須為正有限值')
        duration = distance_m / speed
        self.navigator.get_logger().info(
            f'定時直行：距離 {distance_m:.3f} m / 速度 {speed:.3f} m/s'
            f' = {duration:.2f} 秒（不使用 TF，距離為估計值）')
        twist = Twist()
        twist.linear.x = speed
        start = time.monotonic()
        try:
            while rclpy.ok():
                elapsed = time.monotonic() - start
                if elapsed >= duration:
                    self.navigator.get_logger().info('定時直行結束')
                    return
                if elapsed >= self.config['motion_timeout']:
                    raise RuntimeError('定時直行超時')
                self.cmd_pub.publish(twist)
                time.sleep(min(0.05, duration - elapsed,
                               self.config['motion_timeout'] - elapsed))
            raise RuntimeError('ROS 已關閉，直行中止')
        finally:
            self.stop()

    def control_motion(self, command, pose_reader=None):
        """command(pose) 回傳 Twist；回傳 None 代表到達，失敗則停止任務。"""
        pose_reader = pose_reader or self.get_pose
        deadline = time.monotonic() + self.config['motion_timeout']
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                cycle_start = time.monotonic()
                pose = self.wait_for_pose(
                    pose_reader=pose_reader,
                    timeout_s=min(self.config['tf_timeout'],
                                  max(0.0, deadline - time.monotonic())))
                if time.monotonic() >= deadline:
                    raise RuntimeError('cmd_vel 動作超時')
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

        # 新鮮 TF 的等待僅在此座標導航流程中使用。
        self.control_motion(command, pose_reader=self.get_pose)

    def run(self):
        # Humble 的 waitUntilNav2Active 會等待 localizer 的 lifecycle。
        # 等待 bt_navigator；本程式的定時直行不查詢或等待 TF。
        self.navigator.waitUntilNav2Active(localizer='bt_navigator')
        if self.config['mission'] == 'laptop':
            self.navigate('laptop')
        else:
            self.navigate('Before_between')
            # 等待速度平滑器先前的 Nav2 指令消退，再接手手動控制。
            # deadline = time.monotonic() + 1.0
            # while rclpy.ok() and time.monotonic() < deadline:
            #     self.stop()
            #     rclpy.spin_once(self.navigator, timeout_sec=0.05)
            # self.rotate_right()
            distance = math.dist(self.point('Before_between'), self.point('between'))
            self.navigator.get_logger().info(
                f'依 WAYPOINTS 計算 Before_between → between 距離：{distance:.3f} m')
            distance = 1.35
            self.drive_forward(distance)
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
