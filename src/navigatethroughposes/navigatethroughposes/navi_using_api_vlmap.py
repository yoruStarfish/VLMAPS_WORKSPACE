#! /usr/bin/env python3
from datetime import datetime
import math
import sys
import os
import time
import threading
import cv2
import numpy as np

from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, PoseArray

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from cv_bridge import CvBridge
import tf_transformations
from omegaconf import OmegaConf

base_dir = os.path.expanduser("~/vlmaps_ws")
sys_core_dir = os.path.join(base_dir, "vlmapsSrc")
sys.path.insert(0, sys_core_dir)

from vlmaps.map.vlmap_tracer import VLMapTracer
from vlmaps.robot.tracer_lang_robot import TracerLangRobot
from navigatethroughposes.llm_brain import get_code_from_llm

# ==========================================
# 全域系統變數 (供 LLM 的 API 呼叫使用)
# ==========================================
_navigator = None
_localization_node = None
_lang_robot = None
_pending_command = None


def go_to_object(target_name: str):
    """
    向下相容的包裝函式：若 LLM 仍呼叫舊的 go_to_object，自動轉交給 TracerLangRobot
    """
    global _lang_robot
    print(f"\n[系統指令] 準備前往目標: '{target_name}'...", flush=True)
    _lang_robot.move_to_object(target_name)
    return True


class LocalizationNode(Node):

    def __init__(self):
        super().__init__('localization_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.pose_publisher = PosePublisher(self.tf_buffer, self.tf_listener)

    def timer_callback(self):
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
    while not navigator.isTaskComplete():
        elapsed = (navigator.get_clock().now() - start).nanoseconds / 1e9
        if elapsed > timeout_s:
            print("Navigation timeout — cancelling", flush=True)
            navigator.cancelTask()
            break
        rclpy.spin_once(navigator, timeout_sec=0.2)


def save_current_image(node, topic='/camera/camera/color/image_raw', count=1):
    bridge = CvBridge()
    msg = None
    timeout_sec = 3.0

    node.get_logger().info(f"Waiting for one image from {topic} ...")
    from rclpy.task import Future
    future = Future()

    def image_callback(msg_in):
        future.set_result(msg_in)

    sub = node.create_subscription(Image, topic, image_callback, 10)

    start_time = node.get_clock().now().nanoseconds
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if future.done():
            msg = future.result()
            break
        if (node.get_clock().now().nanoseconds - start_time) / 1e9 > timeout_sec: 
            node.get_logger().warn(f"No image received from {topic} within {timeout_sec}s")
            node.destroy_subscription(sub)
            return

    if msg is None:
        node.get_logger().warn("No image received; skipping save.")
        node.destroy_subscription(sub)
        return

    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    save_dir = os.path.join(os.path.expanduser('~'), 'vlmaps_ws', 'saved_imgs')
    os.makedirs(save_dir, exist_ok=True)
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    filepath = os.path.join(save_dir, f'image_{timestamp}_Checkpoint{count}.png')
    cv2.imwrite(filepath, cv_image)
    node.get_logger().info(f"Saved one image to: {filepath}")
    node.destroy_subscription(sub)


def pause_navigation(t):
    time.sleep(t)


def trigger_snapshot_service(node, client):
    if not client.service_is_ready():
        node.get_logger().warn("Snapshot service is not available. Is snapshot_server running?")
        return

    request = Trigger.Request()
    future = client.call_async(request)
    node.get_logger().info("Triggered snapshot service.")


class UserCommandSubscriber(Node): 
    def __init__(self): 
        super().__init__('user_command_subscriber') 
        self.subscription = self.create_subscription(String, '/user_command', self.command_callback, 10) 
        self.get_logger().info('Listening to /user_command...') 

    def command_callback(self, msg: String): 
        global _pending_command
        _pending_command = msg.data 
        self.get_logger().info(f'Received command: {_pending_command}')


def main():
    global _navigator, _localization_node, _lang_robot, _pending_command
    
    rclpy.init()
    
    # 1. 啟動 Nav2 導航器
    print("啟動 Nav2 導航器...", flush=True)
    _navigator = BasicNavigator()
    _navigator.waitUntilNav2Active()
    
    # 2. 啟動 LocalizationNode 獲取真實座標
    print("啟動 LocalizationNode 獲取真實座標...", flush=True)
    _localization_node = LocalizationNode()
    while rclpy.ok() and not _localization_node.pose_publisher.get_pose():
        rclpy.spin_once(_localization_node, timeout_sec=0.5)
    print("成功取得初始座標。", flush=True)
    
    # 3. 初始化 VLMaps 系統並綁定 ROS 2 節點
    print("初始化 VLMaps 系統...", flush=True) 
    _lang_robot = TracerLangRobot(
        config=None,
        navigator=_navigator,
        localization_node=_localization_node,
        wait_fn=wait_for_task
    ) 
    map_config = OmegaConf.load("vlmapsSrc/config/map_config/vlmaps_tracer.yaml")
    data_dir = os.path.join(base_dir, "dataset")
    _lang_robot.load_scene_map(data_dir=data_dir, map_config=map_config)
    
    # 4. 啟動互動指令迴圈
    print("\n" + "="*50, flush=True)
    print("語音/文字導航助理已啟動", flush=True)
    print("="*50, flush=True)

    cmd_node = UserCommandSubscriber()

    while rclpy.ok():
        # 持續更新機器人座標與指令佇列
        if _localization_node is not None:
            rclpy.spin_once(_localization_node, timeout_sec=0.1)
        rclpy.spin_once(cmd_node, timeout_sec=0.1)
        
        # 檢查是否收到新指令
        if _pending_command is not None:
            cmd = _pending_command.strip()
            _pending_command = None

            if cmd.lower() in ['exit', 'quit']:
                break
                
            if cmd:
                print(f"\n收到指令，開始分析: {cmd}", flush=True)
                code_string = get_code_from_llm(cmd)
                print(f"LLM 決定執行的程式碼:\n{code_string}", flush=True)

                print("開始導航...", flush=True)
                try:
                    # 同時提供 robot 與 _lang_robot 別名，確保 LLM 生成的各種變數名稱皆可順利執行
                    exec_env = {
                        "robot": _lang_robot,
                        "_lang_robot": _lang_robot,
                        "go_to_object": go_to_object
                    }
                    exec(code_string, globals(), exec_env)
                except Exception as e:
                    print(f"程式碼執行失敗: {e}", flush=True)
    
    # 結束清理
    _localization_node.destroy_node()
    cmd_node.destroy_node()
    _navigator.lifecycleShutdown()
    rclpy.shutdown()
    exit(0)

if __name__ == '__main__':
    main()