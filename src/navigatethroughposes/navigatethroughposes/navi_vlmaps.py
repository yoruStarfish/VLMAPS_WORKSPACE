#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from datetime import datetime
import math
import sys

from click import command
from std_msgs.msg import String

from omegaconf import OmegaConf
import rclpy
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped, PoseArray
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from tf2_geometry_msgs import do_transform_pose
from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.time import Time
import time
from cv_bridge import CvBridge
import tf_transformations
import os
import cv2
import threading
from sensor_msgs.msg import Image

base_dir = os.path.expanduser("~/vlmaps_ws")
sys_core_dir = os.path.join(base_dir,"vlmapsSrc")

sys.path.insert(0, sys_core_dir)
from vlmaps.map.vlmap_tracer import VLMapTracer
from vlmaps.robot.tracer_lang_robot import TracerLangRobot

import numpy as np


import tf_transformations

from navigatethroughposes.llm_brain import get_code_from_llm

# ==========================================
# 全域系統變數 (供 LLM 的 API 呼叫使用)
# ==========================================
_navigator = None
_localization_node = None
_lang_robot = None
_pending_command = None  # 用來存放接收到的文字指令

def get_best_safe_pose(
    navigator,
    obj_x,
    obj_y,
    current_x,
    current_y,
    current_yaw,
    min_radius=0.65,
    max_radius=1.20,
    radius_step=0.15,
    angle_step=15
):
    """
    在物體周圍尋找最佳觀測位置。

    搜尋策略：

    1. 從最近的距離 min_radius 開始搜尋。
    2. 每個半徑產生多個候選位置。
    3. 使用 Nav2 getPath() 判斷候選位置是否可達。
    4. 只要某一個半徑有「可達候選點」，就停止往外搜尋。
    5. 在這一圈的可達候選點中，
       不選最短 Path，而選 Path 長度的「中位數」。
    6. 機器人最後朝向物體。
    """

    # ============================================================
    # 1. 建立正確的起點 Pose
    # ============================================================
    start_pose = PoseStamped()

    start_pose.header.frame_id = "map"
    start_pose.header.stamp = (
        navigator.get_clock().now().to_msg()
    )

    start_pose.pose.position.x = float(current_x)
    start_pose.pose.position.y = float(current_y)

    # 使用機器人目前的 yaw
    start_q = tf_transformations.quaternion_from_euler(
        0,
        0,
        current_yaw
    )

    start_pose.pose.orientation.x = start_q[0]
    start_pose.pose.orientation.y = start_q[1]
    start_pose.pose.orientation.z = start_q[2]
    start_pose.pose.orientation.w = start_q[3]

    # ============================================================
    # 2. 從最近距離開始，一圈一圈搜尋
    # ============================================================
    radius = min_radius

    while radius <= max_radius + 0.001:

        print(
            f"\n🔎 搜尋距離物體 {radius:.2f} m 的候選位置...",
            flush=True
        )

        # --------------------------------------------------------
        # 只儲存「這一圈」的可達候選點
        # --------------------------------------------------------
        candidates = []

        # --------------------------------------------------------
        # 每 angle_step 度產生一個候選點
        # 15 度 = 24 個候選方向
        # --------------------------------------------------------
        for angle_deg in range(0, 360, angle_step):

            angle_rad = math.radians(angle_deg)

            # ----------------------------------------------------
            # 候選位置
            # ----------------------------------------------------
            cand_x = (
                obj_x
                + radius * math.cos(angle_rad)
            )

            cand_y = (
                obj_y
                + radius * math.sin(angle_rad)
            )

            # ----------------------------------------------------
            # 機器人朝向物體
            # ----------------------------------------------------
            yaw = math.atan2(
                obj_y - cand_y,
                obj_x - cand_x
            )

            q = tf_transformations.quaternion_from_euler(
                0,
                0,
                yaw
            )

            # ----------------------------------------------------
            # 建立測試 Goal
            # ----------------------------------------------------
            test_goal = PoseStamped()

            test_goal.header.frame_id = "map"
            test_goal.header.stamp = (
                navigator.get_clock().now().to_msg()
            )

            test_goal.pose.position.x = float(cand_x)
            test_goal.pose.position.y = float(cand_y)

            test_goal.pose.orientation.x = q[0]
            test_goal.pose.orientation.y = q[1]
            test_goal.pose.orientation.z = q[2]
            test_goal.pose.orientation.w = q[3]

            # ----------------------------------------------------
            # 讓 Nav2 試著規劃
            # ----------------------------------------------------
            try:

                print(
                    f"   🔄 Getting path "
                    f"angle={angle_deg:3d}° "
                    f"X={cand_x:.2f} "
                    f"Y={cand_y:.2f}",
                    flush=True
                )

                path = navigator.getPath(
                    start_pose,
                    test_goal
                )

            except Exception as e:

                print(
                    f"   ⚠️ getPath 發生錯誤: {e}",
                    flush=True
                )

                continue

            # ----------------------------------------------------
            # 沒有有效 Path
            # ----------------------------------------------------
            if path is None or len(path.poses) < 2:

                print(
                    "      ❌ 不可達",
                    flush=True
                )

                continue

            # ----------------------------------------------------
            # 有 Path
            # ----------------------------------------------------
            print(
                f"      ✅ 可達 "
                f"(poses={len(path.poses)})",
                flush=True
            )

            # ====================================================
            # 3. 計算真正的 Path 長度
            # ====================================================
            path_length = 0.0

            for i in range(1, len(path.poses)):

                p1 = path.poses[i - 1].pose.position
                p2 = path.poses[i].pose.position

                dx = p2.x - p1.x
                dy = p2.y - p1.y

                path_length += math.hypot(dx, dy)

            # ====================================================
            # 4. 計算其他資訊
            # ====================================================
            object_distance = math.hypot(
                cand_x - obj_x,
                cand_y - obj_y
            )

            direct_distance = math.hypot(
                cand_x - current_x,
                cand_y - current_y
            )

            candidates.append({
                "radius": object_distance,
                "path_length": path_length,
                "direct_distance": direct_distance,
                "x": cand_x,
                "y": cand_y,
                "q": q,
                "angle": angle_deg
            })

        # ========================================================
        # 5. 這一圈有可達點
        #
        # 非常重要：
        # 找到第一圈可達點後，就「停止往外找」
        # ========================================================
        if candidates:

            print(
                f"\n✅ {radius:.2f} m 這一圈找到 "
                f"{len(candidates)} 個可達候選點。",
                flush=True
            )

            # ====================================================
            # 6. 按照 Path 長度排序
            # ====================================================
            candidates.sort(
                key=lambda candidate: candidate["path_length"]
            )

            print(
                "📊 這一圈的 Path 長度：",
                flush=True
            )

            for candidate in candidates:

                print(
                    f"   {candidate['path_length']:.2f} m "
                    f"→ angle={candidate['angle']}° "
                    f"X={candidate['x']:.2f} "
                    f"Y={candidate['y']:.2f}",
                    flush=True
                )

            # ====================================================
            # 7. 計算 Path 長度的中位數
            # ====================================================
            path_lengths = [
                candidate["path_length"]
                for candidate in candidates
            ]

            n = len(path_lengths)

            if n % 2 == 1:

                # 奇數個
                median_path = path_lengths[n // 2]

            else:

                # 偶數個
                median_path = (
                    path_lengths[n // 2 - 1]
                    + path_lengths[n // 2]
                ) / 2.0

            # ====================================================
            # 8. 找「最接近中位數」的實際候選點
            # ====================================================
            best = min(
                candidates,
                key=lambda candidate:
                    abs(
                        candidate["path_length"]
                        - median_path
                    )
            )

            # ====================================================
            # 9. 顯示最終結果
            # ====================================================
            print(
                "\n🎯 使用 Path 中位數選擇觀測位置！",
                flush=True
            )

            print(
                f"   X = {best['x']:.2f}",
                flush=True
            )

            print(
                f"   Y = {best['y']:.2f}",
                flush=True
            )

            print(
                f"   距離物體 = "
                f"{best['radius']:.2f} m",
                flush=True
            )

            print(
                f"   中位數 Path = "
                f"{median_path:.2f} m",
                flush=True
            )

            print(
                f"   實際選擇 Path = "
                f"{best['path_length']:.2f} m",
                flush=True
            )

            print(
                f"   方向 = "
                f"{best['angle']}°",
                flush=True
            )

            return (
                best["x"],
                best["y"],
                best["q"]
            )

        # ========================================================
        # 這一圈完全沒有可達點
        # ========================================================
        print(
            f"   ❌ {radius:.2f} m 沒有任何可達位置",
            flush=True
        )

        # 往外增加半徑
        radius += radius_step

    # ============================================================
    # 10. 所有半徑都沒有可達點
    # ============================================================
    print(
        "\n⚠️ 從最近距離一路搜尋到最大距離，"
        "都找不到 Nav2 可達的位置。",
        flush=True
    )

    return None, None, None


def go_to_object(target_name: String):
    """
    前往 VLMaps 找到的物體附近，
    自動尋找「靠近物體、Nav2 可達、且路徑合理」的最佳觀測位置。
    """

    global _navigator, _localization_node, _lang_robot

    print(f"\n🤖 [系統指令] 準備前往目標: '{target_name}'...", flush=True)

    # ============================================================
    # 1. 取得機器人目前位置
    # ============================================================
    rclpy.spin_once(_localization_node, timeout_sec=0.5)

    current_pose = _localization_node.pose_publisher.get_pose()

    if current_pose is None:
        print("⚠️ 無法取得機器人當前座標，放棄導航。", flush=True)
        return False

    current_world_x = current_pose.pose.position.x
    current_world_y = current_pose.pose.position.y

    # 取得目前機器人的 yaw
    current_q = current_pose.pose.orientation
    _, _, current_yaw = tf_transformations.euler_from_quaternion([
        current_q.x,
        current_q.y,
        current_q.z,
        current_q.w
    ])

    print(
        f"📍 機器人當前座標 -> "
        f"X: {current_world_x:.2f}, "
        f"Y: {current_world_y:.2f}",
        flush=True
    )

    # ============================================================
    # 2. VLMaps 找物體
    # ============================================================
    grid_size = _lang_robot.map.gs
    cell_size = _lang_robot.map.cs
    center_offset = grid_size / 2.0

    current_col = int(
        (current_world_x / cell_size) + center_offset
    )

    current_row = int(
        (-current_world_y / cell_size) + center_offset
    )

    _lang_robot.curr_pos_on_map = [current_row, current_col]

    grid_center = _lang_robot.get_pos(target_name)

    if grid_center is None:
        print(
            f"❌ 地圖上找不到物件: {target_name}",
            flush=True
        )
        return False

    row, col = grid_center

    obj_x = (col - center_offset) * cell_size
    obj_y = -(row - center_offset) * cell_size

    print(
        f"🎯 物體中心 -> X: {obj_x:.2f}, Y: {obj_y:.2f}",
        flush=True
    )

    # ============================================================
    # 3. 尋找最佳安全位置
    # ============================================================
    safe_world_x, safe_world_y, safe_q = get_best_safe_pose(
        _navigator,
        obj_x,
        obj_y,
        current_world_x,
        current_world_y,
        current_yaw
    )

    if safe_world_x is None:
        print(
            "⚠️ 找不到合適的安全觀測位置，放棄前往。",
            flush=True
        )
        return False

    # ============================================================
    # 4. 發送 Nav2 目標
    # ============================================================
    goal_pose = PoseStamped()

    goal_pose.header.frame_id = "map"
    goal_pose.header.stamp = (
        _navigator.get_clock().now().to_msg()
    )

    goal_pose.pose.position.x = float(safe_world_x)
    goal_pose.pose.position.y = float(safe_world_y)

    goal_pose.pose.orientation.x = safe_q[0]
    goal_pose.pose.orientation.y = safe_q[1]
    goal_pose.pose.orientation.z = safe_q[2]
    goal_pose.pose.orientation.w = safe_q[3]

    print(
        f"🎯 最佳觀測位置 -> "
        f"X: {safe_world_x:.2f}, "
        f"Y: {safe_world_y:.2f}",
        flush=True
    )

    print(
        f"📏 距離物體: "
        f"{math.hypot(safe_world_x - obj_x, safe_world_y - obj_y):.2f} m",
        flush=True
    )

    print(
        f"🚗 Nav2 啟動，前往 {target_name} 最佳觀測位置...",
        flush=True
    )

    _navigator.goToPose(goal_pose)

    # ============================================================
    # 5. 等待導航完成
    # ============================================================
    wait_for_task(_navigator)

    print(
        f"🎉 成功抵達 {target_name} 附近！\n",
        flush=True
    )

    return True



class LocalizationNode(Node):

    def __init__(self):
        super().__init__('localization_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.pose_publisher = PosePublisher(self.tf_buffer, self.tf_listener)

    def timer_callback(self):
        #try:
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
        combined_transform = self.combine_transforms(map_to_odom, odom_to_base_link)
        self.print_transform(combined_transform)
        if combined_transform:
            self.pose_publisher.update_pose(combined_transform)
        # except TransformException as ex:
        #     self.get_logger().warn('%s; retrying...' % ex)

    def combine_transforms(self, transform1, transform2):
        t1_translation = transform1.transform.translation
        t1_rotation = transform1.transform.rotation
        t2_translation = transform2.transform.translation
        t2_rotation = transform2.transform.rotation

        t1_matrix = tf_transformations.translation_matrix(
            [t1_translation.x, t1_translation.y, t1_translation.z])
        t1_rotation_matrix = tf_transformations.quaternion_matrix(
            [t1_rotation.x, t1_rotation.y, t1_rotation.z, t1_rotation.w])
        t1_matrix = tf_transformations.concatenate_matrices(t1_matrix, t1_rotation_matrix)

        t2_matrix = tf_transformations.translation_matrix(
            [t2_translation.x, t2_translation.y, t2_translation.z])
        t2_rotation_matrix = tf_transformations.quaternion_matrix(
            [t2_rotation.x, t2_rotation.y, t2_rotation.z, t2_rotation.w])
        t2_matrix = tf_transformations.concatenate_matrices(t2_matrix, t2_rotation_matrix)

        combined_matrix = tf_transformations.concatenate_matrices(t1_matrix, t2_matrix)

        combined_transform = transform1
        combined_transform.transform.translation.x, combined_transform.transform.translation.y, combined_transform.transform.translation.z = tf_transformations.translation_from_matrix(
            combined_matrix)
        combined_transform.transform.rotation.x, combined_transform.transform.rotation.y, combined_transform.transform.rotation.z, combined_transform.transform.rotation.w = tf_transformations.quaternion_from_matrix(
            combined_matrix)

        return combined_transform

    def print_transform(self, trans):
        translation = trans.transform.translation
        rotation = trans.transform.rotation
        rot_matrix = tf_transformations.quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])

        self.get_logger().info(f'At time {trans.header.stamp.sec}.{trans.header.stamp.nanosec}')
        self.get_logger().info(f'- Translation: [{translation.x:.3f}, {translation.y:.3f}, {translation.z:.3f}]')
        self.get_logger().info(f'- Rotation: in Quaternion [{rotation.x:.3f}, {rotation.y:.3f}, {rotation.z:.3f}, {rotation.w:.3f}]')
        rpy_rad = tf_transformations.euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])
        rpy_deg = [angle * 180.0 / 3.141592653589793 for angle in rpy_rad]
        self.get_logger().info(f'- Rotation: in RPY (radian) [{rpy_rad[0]:.3f}, {rpy_rad[1]:.3f}, {rpy_rad[2]:.3f}]')
        self.get_logger().info(f'- Rotation: in RPY (degree) [{rpy_deg[0]:.3f}, {rpy_deg[1]:.3f}, {rpy_deg[2]:.3f}]')
        self.get_logger().info('- Matrix:')
        for row in rot_matrix:
            self.get_logger().info(f' {row[0]:.3f} {row[1]:.3f} {row[2]:.3f} {row[3]:.3f}')


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

def wait_for_task(navigator, timeout_s=180.0):
    start = navigator.get_clock().now()
    i = 0
    # 用來偵測卡住的變數
    stuck_start_time = None
    # 設定你覺得「可以接受」的誤差範圍
    ACCEPTABLE_DIST = 0.45  # 0.5公尺內就算到了
    STUCK_TIMEOUT = 5.0    # 如果在範圍內卡了5秒，就強制結束
    while not navigator.isTaskComplete():
        # i += 1
        # feedback = navigator.getFeedback()
        # if feedback is not None:
        #     # print everything useful every second
        #     # 顯示資訊 (每 5 次顯示一次)
        #     if i % 5 == 0:
        #         print("----- NAV FEEDBACK -----")
        #         print("Elapsed:", (navigator.get_clock().now() - start).nanoseconds / 1e9, "s")
        #         try:
        #             print("Distance remaining:", feedback.distance_remaining)
        #         except AttributeError:
        #             print("No distance_remaining in feedback")
        #         print("Navigation time:", feedback.navigation_time.sec, "s")
        #         # Some feedback implementations include current_pose; print if present
        #         if hasattr(feedback, "current_pose"):
        #             cp = feedback.current_pose.pose.position
        #             print(f"Current pose: x={cp.x:.3f}, y={cp.y:.3f}")
        #         # Print behavior tree status if available
        #         if hasattr(feedback, "current_behavior_tree"):
        #             print("BT:", feedback.current_behavior_tree)

        #     # --- [新增邏輯]：Close Enough Check (夠近就放行) ---
        #     # 如果距離小於 0.5 公尺 (可自行調整)
        #     if feedback.distance_remaining < ACCEPTABLE_DIST:
        #         # 如果這是我們第一次進入「夠近範圍」，或者距離沒有顯著變化
        #         if stuck_start_time is None:
        #             stuck_start_time = navigator.get_clock().now()
                
        #         # 計算在這個距離卡了多久
        #         stuck_duration = (navigator.get_clock().now() - stuck_start_time).nanoseconds / 1e9
                
        #         # 如果在 0.5m 內卡了超過 5 秒，就當作成功，強制結束
        #         if stuck_duration > STUCK_TIMEOUT:
        #             print(f"Near goal ({feedback.distance_remaining:.3f}m) for 5s. Forcing success!")
        #             navigator.cancelTask() # 取消當前任務，讓程式往下跑
        #             return # 離開函式，繼續下一個動作
        #     else:
        #         # 如果距離還很遠，重置計時器
        #         stuck_start_time = None
        # timeout / cancel logic
        elapsed = (navigator.get_clock().now() - start).nanoseconds / 1e9
        if elapsed > timeout_s:
            print("Navigation timeout — cancelling")
            navigator.cancelTask()
            break
        rclpy.spin_once(navigator, timeout_sec=0.2)

def save_current_image(node, topic='/camera/camera/color/image_raw', count=1):
    """
    Saves one image from the specified topic.
    :param node: Any rclpy Node (e.g., your navigation node)
    :param topic: The camera image topic name
    """
    bridge = CvBridge() # CvBridge() converts between ROS2 Image messages and OpenCV (cv2) images
    msg = None
    timeout_sec = 3.0  # wait for up to 3 seconds

    node.get_logger().info(f"Waiting for one image from {topic} ...")
    msg = node.create_subscription(Image, topic, lambda m: node.get_logger().info(""), 10)
    # But we need to actually use rclpy.wait_for_message — but ROS2 doesn’t have it built-in.
    # So we do a spin_until_future_complete trick below:

    from rclpy.task import Future
    future = Future()
    '''
    ROS2 doesn’t have wait_for_message() like ROS1.

    So we simulate it:

    Create a Future object (a promise of a result).

    When an image message comes in, image_callback() sets that message as the future’s result.

    Once that happens, future.done() will be True.
    '''

    def image_callback(msg_in):
        future.set_result(msg_in)

    sub = node.create_subscription(Image, topic, image_callback, 10)

    start_time = node.get_clock().now().nanoseconds
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1) # keeps the node spinning for short intervals (0.1s)
        if future.done(): # If an image message has been received then break
            msg = future.result()
            break
        if (node.get_clock().now().nanoseconds - start_time) / 1e9 > timeout_sec: 
            # If no image arrives within timeout_sec, print a warning and stop
            node.get_logger().warn(f"No image received from {topic} within {timeout_sec}s")
            return

    if msg is None:
        node.get_logger().warn("No image received; skipping save.")
        return

    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') # Converts the ROS2 sensor_msgs/Image message into a NumPy array in OpenCV format (BGR).

    # Save to ~/rtab_ws/saved_images
    save_dir = os.path.join(os.path.expanduser('~'), 'vlmaps_ws', 'saved_imgs')
    os.makedirs(save_dir, exist_ok=True)
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    filepath = os.path.join(save_dir, f'image_{timestamp}_Checkpoint{count}.png')
    cv2.imwrite(filepath, cv_image)
    node.get_logger().info(f"Saved one image to: {filepath}") # Unsubscribes to avoid keeping unnecessary subscriptions open

    # Clean up
    node.destroy_subscription(sub)


def pause_navigation(t):
    time.sleep(t)


from std_srvs.srv import Trigger
def trigger_snapshot_service(node, client):
    """
    Call the /trigger_snapshot service asynchronously.
    """
    if not client.service_is_ready():
        node.get_logger().warn("Snapshot service is not available. Is snapshot_server running?")
        return

    request = Trigger.Request()
    future = client.call_async(request)
    # 這裡我們不等待結果 (non-blocking)，避免卡住導航
    # 如果你想要確認成功與否，可以加 callback，但通常不需要
    node.get_logger().info("Triggered snapshot service.")


class UserCommandSubscriber(Node): 
    def __init__(self): 
        super().__init__('user_command_subscriber') 
        self.subscription = self.create_subscription( String, '/user_command', self.command_callback, 10 ) 
        self.get_logger().info('Listening to /user_command...') 

    def command_callback(self, msg: String): 
        global _pending_command

        _pending_command = msg.data 
        self.get_logger().info(f'Received command: {_pending_command}') # 在這裡處理 command if command == 'move_forward': self.move_forward() elif command == 'stop': self.stop()



# def main():
#     rclpy.init()
    
#     # 1. 啟動 Nav2[cite: 4]
#     navigator = BasicNavigator()
#     navigator.waitUntilNav2Active()
    
#     # 2. 啟動定位節點，確保我們能拿到 TF 樹上的絕對座標[cite: 4]
#     print("啟動 LocalizationNode 獲取真實座標...")
#     localization_node = LocalizationNode()
    
#     # 轉動節點直到成功取得第一次的座標
#     while rclpy.ok() and not localization_node.pose_publisher.get_pose():
#         rclpy.spin_once(localization_node, timeout_sec=0.5)
#     print("✅ 成功取得初始座標！")
    
#     # 3. 初始化 VLMaps 大腦[cite: 4]
#     print("🧠 初始化 VLMaps 系統...") 
#     robot_api = TracerLangRobot(config=None) 
#     map_config = OmegaConf.load("vlmapsSrc/config/map_config/vlmaps_tracer.yaml")
#     data_dir = os.path.join(base_dir, "dataset")
#     robot_api.load_scene_map(data_dir=data_dir, map_config=map_config)
    
#     # 4. 開始執行高階語意導航任務！[cite: 4]
#     targets_to_find = ["laptop"]  # 你想要找的物品清單
    
#     for count, target in enumerate(targets_to_find):
#         # 每次前往新目標前，刷新一次當前座標
#         rclpy.spin_once(localization_node, timeout_sec=0.5)
#         current_pose = localization_node.pose_publisher.get_pose()
        
#         if current_pose is None:
#             print("⚠️ 無法取得機器人當前座標，跳過此目標。")
#             continue
            
#         current_world_x = current_pose.pose.position.x
#         current_world_y = current_pose.pose.position.y
        
#         # 將當前座標傳入 go_to_object
#         success = go_to_object(robot_api, navigator, target, current_world_x, current_world_y)
        
#         if success:
#             wait_for_task(navigator) 
#             save_current_image(navigator, topic='/camera/camera/color/image_raw', count=count+1)
#             print(f"🎉 成功抵達 {target}！")
#         else:
#             print(f"⚠️ 跳過 {target}，尋找下一個目標。")

#     # 結束清理[cite: 4]
#     localization_node.destroy_node()
#     navigator.lifecycleShutdown()
#     rclpy.shutdown()
#     exit(0)


def main():
    global _navigator, _localization_node, _lang_robot, _pending_command
    
    rclpy.init()
    
    # --- 系統初始化區塊 ---
    print("啟動 Nav2 導航器...", flush=True)
    _navigator = BasicNavigator()
    _navigator.waitUntilNav2Active()
    
    print("啟動 LocalizationNode 獲取真實座標...", flush=True)
    _localization_node = LocalizationNode()
    while rclpy.ok() and not _localization_node.pose_publisher.get_pose():
        rclpy.spin_once(_localization_node, timeout_sec=0.5)
    print("✅ 成功取得初始座標！", flush=True)
    
    print("🧠 初始化 VLMaps 系統...", flush=True) 
    _lang_robot = TracerLangRobot(config=None) 
    map_config = OmegaConf.load("vlmapsSrc/config/map_config/vlmaps_tracer.yaml")
    data_dir = os.path.join(base_dir, "dataset")
    _lang_robot.load_scene_map(data_dir=data_dir, map_config=map_config)
    
    # ==========================================
    # ⚡ LLM 測試區塊 (這裡就是未來放 LLM 程式碼的地方)
    # ==========================================
    # --- 互動指令迴圈 ---
    print("\n" + "="*50, flush=True)
    print("🤖 語音/文字導航助理已啟動！", flush=True)
    print("="*50, flush=True)

    # 啟動背景輸入小精靈
    cmd_node = UserCommandSubscriber()

    while rclpy.ok():
        # print("main loop running...", flush=True)
        # 1. 持續更新機器人座標，防止 ROS 2 當機！
        if _localization_node is not None:
            rclpy.spin_once(_localization_node, timeout_sec=0.1)
        rclpy.spin_once(cmd_node, timeout_sec=0.1) # 讓 LocalizationNode 和 cmd_node 保持運作，接收座標與指令

        # print(
        #     f"(O_O) pending command = {_pending_command}",
        #     flush=True
        # )
        
        # 2. 檢查背景精靈有沒有收到新指令
        if _pending_command is not None:
            cmd = _pending_command.strip()
            _pending_command = None  # 清空指令

            # 處理離開指令
            if cmd.lower() in ['exit', 'quit']:
                break
                
            # 處理導航指令
            if cmd:
                print(f"\n🧠 收到指令，開始分析: {cmd}", flush=True)
                code_string = get_code_from_llm(cmd)
                print(f"📝 LLM 決定執行的程式碼:\n{code_string}", flush=True)

                print("⚡ 開始導航...", flush=True)
                try:
                    exec(code_string)
                except Exception as e:
                    print(f"❌ 程式碼執行失敗: {e}", flush=True)
            
            # 執行完畢，清空指令，讓背景精靈顯示下一次的輸入框
            _user_input_cmd = None
    
    # --- 結束清理 ---
    _localization_node.destroy_node()
    _navigator.lifecycleShutdown()
    rclpy.shutdown()
    exit(0)

if __name__ == '__main__':
    main()
