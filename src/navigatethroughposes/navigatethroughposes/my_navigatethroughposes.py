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
from goto_aruco.GoToArucoActionClient import GoToArucoActionClient
import os
import cv2
from sensor_msgs.msg import Image
from ros2_aruco.aruco_node import ArucoNode

class NavigateToARUco(rclpy.node.Node):
    def __init__(self):
        super().__init__("NavigateToARUco")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.subscription = self.create_subscription(
            PoseArray,
            '/aruco_poses',
            self.aruco_pose_callback,
            10
        )
        self.pose_publisher = ARUcoPosePublisher(self.tf_buffer, self.tf_listener)

    def aruco_pose_callback(self, msg: PoseArray):
        if msg.poses and not self.pose_publisher.get_pose():
            print("setting pose of aruco")
            aruco_pose = msg.poses[0]
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose.position.x = aruco_pose.position.x
            pose_stamped.pose.position.y = aruco_pose.position.y - 0.55
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z =  -0.707    #1.0
            pose_stamped.pose.orientation.w =  0.707    #0.0
            self.pose_publisher.update_pose(pose_stamped)


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

class ARUcoPosePublisher(Node):

    def __init__(self, tf_buffer, tf_listener):
        super().__init__('ARUco_pose_publisher')
        self.tf_buffer = tf_buffer
        self.tf_listener = tf_listener
        self.pose = None

    def update_pose(self, trans):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = trans.pose.position.x
        pose_msg.pose.position.y = trans.pose.position.y
        pose_msg.pose.position.z = trans.pose.position.z
        pose_msg.pose.orientation.x = trans.pose.orientation.x
        pose_msg.pose.orientation.y = trans.pose.orientation.y
        pose_msg.pose.orientation.z = trans.pose.orientation.z
        pose_msg.pose.orientation.w = trans.pose.orientation.w
        self.pose = pose_msg

    def get_pose(self):
        return self.pose

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
        i += 1
        feedback = navigator.getFeedback()
        if feedback is not None:
            # print everything useful every second
            # 顯示資訊 (每 5 次顯示一次)
            if i % 5 == 0:
                print("----- NAV FEEDBACK -----")
                print("Elapsed:", (navigator.get_clock().now() - start).nanoseconds / 1e9, "s")
                try:
                    print("Distance remaining:", feedback.distance_remaining)
                except AttributeError:
                    print("No distance_remaining in feedback")
                print("Navigation time:", feedback.navigation_time.sec, "s")
                # Some feedback implementations include current_pose; print if present
                if hasattr(feedback, "current_pose"):
                    cp = feedback.current_pose.pose.position
                    print(f"Current pose: x={cp.x:.3f}, y={cp.y:.3f}")
                # Print behavior tree status if available
                if hasattr(feedback, "current_behavior_tree"):
                    print("BT:", feedback.current_behavior_tree)

            # --- [新增邏輯]：Close Enough Check (夠近就放行) ---
            # 如果距離小於 0.5 公尺 (可自行調整)
            if feedback.distance_remaining < ACCEPTABLE_DIST:
                # 如果這是我們第一次進入「夠近範圍」，或者距離沒有顯著變化
                if stuck_start_time is None:
                    stuck_start_time = navigator.get_clock().now()
                
                # 計算在這個距離卡了多久
                stuck_duration = (navigator.get_clock().now() - stuck_start_time).nanoseconds / 1e9
                
                # 如果在 0.5m 內卡了超過 5 秒，就當作成功，強制結束
                if stuck_duration > STUCK_TIMEOUT:
                    print(f"Near goal ({feedback.distance_remaining:.3f}m) for 5s. Forcing success!")
                    navigator.cancelTask() # 取消當前任務，讓程式往下跑
                    return # 離開函式，繼續下一個動作
            else:
                # 如果距離還很遠，重置計時器
                stuck_start_time = None
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
    save_dir = os.path.join(os.path.expanduser('~'), 'rtab_ws', 'saved_imgs')
    os.makedirs(save_dir, exist_ok=True)
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    filepath = os.path.join(save_dir, f'image_{timestamp}_Checkpoint{count}.png')
    cv2.imwrite(filepath, cv_image)
    node.get_logger().info(f"Saved one image to: {filepath}") # Unsubscribes to avoid keeping unnecessary subscriptions open

    # Clean up
    node.destroy_subscription(sub)


def pause_navigation(t):
    time.sleep(t)

# def go_to_aruco(navigator):
#     aruco_node = ArucoNode()
#     aruco_navigator = NavigateToARUco()
#     print("getting pose from aruco node")
#     executor = rclpy.executors.MultiThreadedExecutor()
#     executor.add_node(aruco_node)
#     executor.add_node(aruco_navigator)
#     print("Getting pose from aruco node...")
#     while rclpy.ok() and not aruco_navigator.pose_publisher.get_pose():
#         print("spin once again")
#         executor.spin_once(timeout_sec=0.1)
    
#     nav_start = navigator.get_clock().now()
#     print("navigate to aruco pose")
#     navigator.goToPose(aruco_navigator.pose_publisher.get_pose())
#     i = 0
#     while not navigator.isTaskComplete():
#         i += 1
#         feedback = navigator.getFeedback()
#         if feedback and i % 5 == 0:
#             print('Executing aruco pose: ')
#             now = navigator.get_clock().now()

#             # Cancel task if it takes too long
#             if now - nav_start > Duration(seconds=600.0):
#                 navigator.cancelTask()

#             # Preempt task with new goal if it takes too long
#             if now - nav_start > Duration(seconds=100.0):
#                 goal_pose4 = PoseStamped()
#                 goal_pose4.header.frame_id = 'map'
#                 goal_pose4.header.stamp = now.to_msg()
#                 goal_pose4.pose.position.x = 0.0
#                 goal_pose4.pose.position.y = 0.0
#                 goal_pose4.pose.orientation.w = 0.0
#                 goal_pose4.pose.orientation.z = 1.0
#                 nav_start = now
#                 navigator.goToPose(goal_pose4)
#     aruco_node.destroy_node()
    
    # TODO: Correct the pose in amcl
    '''
    corrected_aruco_pose = PoseStamped()
    corrected_aruco_pose.header.frame_id = 'map'
    corrected_aruco_pose.header.stamp = now.to_msg()
    corrected_aruco_pose.pose.position.x = 0.0
    corrected_aruco_pose.pose.position.y = 0.0
    corrected_aruco_pose.pose.orientation.w = 0.0
    corrected_aruco_pose.pose.orientation.z = 1.0
    
    navigator.setInitialPose(corrected_aruco_pose)
    navigator.waitUntilNav2Active()
    '''

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


def call_goto_aruco_server(count=1): # distance that robot should stop in front of the ARUco marker
    action_client = GoToArucoActionClient()
    # future = action_client.send_goal(distance_to_pose)
    # rclpy.spin_until_future_complete(action_client, future)
    save_current_image(action_client, topic='/camera/camera/color/image_raw', count=count)
    future = action_client.send_goal(distance_to_pose = 0.6,
                                     linear_tolerance = 0.3,
                                     orientation_tolerance = 0.01)
    while rclpy.ok() and not action_client.success:
        rclpy.spin_once(action_client)

    save_current_image(action_client, topic='/camera/camera/color/image_raw', count=count)
    future = action_client.send_goal(distance_to_pose = 0.53,
                                     linear_tolerance = 0.1,
                                     orientation_tolerance = 0.01)
    while rclpy.ok() and not action_client.success:
        rclpy.spin_once(action_client)
    action_client.get_logger().info('Action completed.')


def main():
    rclpy.init()
    logger = get_logger('localization_main')
    navigator = BasicNavigator()
    localization_node = LocalizationNode()
    while rclpy.ok() and not localization_node.pose_publisher.get_pose():
        rclpy.spin_once(localization_node, timeout_sec=1.0)

    initial_pose = localization_node.pose_publisher.get_pose()
    if initial_pose:
        logger.info('Got initial pose from TF')
        navigator.setInitialPose(initial_pose)
    else:
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.orientation.z = 1.0
        initial_pose.pose.orientation.w = 0.0
        navigator.setInitialPose(initial_pose)

    # Activate navigation, wait until Nav2 is active
    navigator.waitUntilNav2Active()

    # Set the waypoints to follow with the new configuration
    goal_poses = []
    goal_pause = []

    goal_poseOrigin = PoseStamped()
    goal_poseOrigin.header.frame_id = 'map'
    goal_poseOrigin.header.stamp = navigator.get_clock().now().to_msg()
    goal_poseOrigin.pose.position.x = 0.0
    goal_poseOrigin.pose.position.y = 0.0
    goal_poseOrigin.pose.orientation.w = 1.0
    goal_poseOrigin.pose.orientation.z = 0.0
    goal_pause.append(0.0)

    goal_poseLoad = PoseStamped()
    goal_poseLoad.header.frame_id = 'map'
    goal_poseLoad.header.stamp = navigator.get_clock().now().to_msg()
    goal_poseLoad.pose.position.x = 8.534028813982529
    goal_poseLoad.pose.position.y = 0.6136315443930057
    goal_poseLoad.pose.orientation.w = 1.0
    goal_poseLoad.pose.orientation.z = 0.00
    
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 32.530828682#7.1
    goal_pose1.pose.position.y = 1.983207871#0.1
    goal_pose1.pose.orientation.w = 1.0#0.0
    goal_pose1.pose.orientation.z = 0.0#0.0

    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 33.392909337#33.0
    goal_pose2.pose.position.y = -1.763458102#1.9
    goal_pose2.pose.orientation.w = 0.685#0.0
    goal_pose2.pose.orientation.z = -0.72


    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = 43.19715403#33.0
    goal_pose3.pose.position.y = -1.070896089#-2.0
    goal_pose3.pose.orientation.w = 0.98#0.0
    goal_pose3.pose.orientation.z = 0.15733300468185107#0.0


    goal_pose4 = PoseStamped()
    goal_pose4.header.frame_id = 'map'
    goal_pose4.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose4.pose.position.x = 43.333044346#38.4
    goal_pose4.pose.position.y = 0.064533098#-1.0
    goal_pose4.pose.orientation.w = 0.07643808939943315
    goal_pose4.pose.orientation.z = 0.9970743294704584#0.0

    goal_pose5 = PoseStamped()
    goal_pose5.header.frame_id = 'map'
    goal_pose5.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose5.pose.position.x = 35.146165746#42.0
    goal_pose5.pose.position.y = 2.598639188#0.3
    goal_pose5.pose.orientation.w = 0.7046463482793143#0.0
    goal_pose5.pose.orientation.z = 0.7046463482793143#1.0

    # goal_pose6 = PoseStamped()
    # goal_pose6.header.frame_id = 'map'
    # goal_pose6.header.stamp = navigator.get_clock().now().to_msg()
    # goal_pose6.pose.position.x = 9.417494168508158#40.71
    # goal_pose6.pose.position.y = 1.619783461785517#1.05
    # goal_pose6.pose.orientation.w = 0.01
    # goal_pose6.pose.orientation.z = 0.9999421087535106#1.0
    
    goal_poseDock = PoseStamped()
    goal_poseDock.header.frame_id = 'map'
    goal_poseDock.header.stamp = navigator.get_clock().now().to_msg()
    goal_poseDock.pose.position.x = 8.668#34.9
    goal_poseDock.pose.position.y = 0.7#-1.1
    goal_poseDock.pose.orientation.w =  0.7289284917726481 #0.616#0.0
    goal_poseDock.pose.orientation.z =  0.684589843542871   #0.787

    # goal_pose8 = PoseStamped()
    # goal_pose8.header.frame_id = 'map'
    # goal_pose8.header.stamp = navigator.get_clock().now().to_msg()
    # goal_pose8.pose.position.x = 34.6
    # goal_pose8.pose.position.y = 2.27
    # goal_pose8.pose.orientation.w = 0.0
    # goal_pose8.pose.orientation.z = 1.0
        
    # goal_pose9 = PoseStamped()
    # goal_pose9.header.frame_id = 'map'
    # goal_pose9.header.stamp = navigator.get_clock().now().to_msg()
    # goal_pose9.pose.position.x = 8.4
    # goal_pose9.pose.position.y = 0.28
    # goal_pose9.pose.orientation.z = -0.6374
    # goal_pose9.pose.orientation.w= 0.7705

    # Append the poses as per your request
    # goal_poses.append(goal_poseOrigin)
    # goal_poses.append(goal_poseLoad)
    goal_poses.append(goal_pose1)
    goal_poses.append(goal_pose2)
    goal_poses.append(goal_pose3)
    goal_poses.append(goal_pose4)
    goal_poses.append(goal_pose5)
    #goal_poses.append(goal_pose6)
    # goal_poses.append(goal_poseDock)
    
    # goal_poses.append(goal_pose8)
    # goal_poses.append(goal_pose9)
    # navigator.goToPose(goal_poseDock)
    # wait_for_task(navigator)
    # navigator.goToPose(goal_poseLoad)
    # wait_for_task(navigator)
    for _ in range(2):
        navigator.goToPose(goal_poseOrigin)
        wait_for_task(navigator)

        # navigator.goToPose(goal_poseLoad)
        # wait_for_task(navigator)
        # pause_navigation(t=5.0)
        snapshot_client = navigator.create_client(Trigger, '/trigger_snapshot')
        for count, goal_pose in enumerate(goal_poses, start=1):
            navigator.goToPose(goal_pose)
            wait_for_task(navigator)
            save_current_image(navigator, topic='/camera/camera/color/image_raw', count=count)
            trigger_snapshot_service(navigator, snapshot_client)
            
        
        navigator.goToPose(goal_poseDock)
        wait_for_task(navigator)
        #go_to_aruco(navigator)
        pause_navigation(t=2.0)
        call_goto_aruco_server(count+1)
        #save_current_image(navigator, topic='/camera/camera/color/image_raw', count=count+1)
        trigger_snapshot_service(navigator, snapshot_client)

    
    # ===Template for following a path===
    # # Start following the waypoints
    # path = navigator.getPathThroughPoses(start = goal_poseOrigin, goals = goal_poses)
    # print("Following path ...")
    # task_follow_path = navigator.followPath(path)   #goThroughPoses followWaypoints

    # wait_for_task(navigator, task_follow_path, goal_poseOrigin)
    # ===Template for following a path===

    localization_node.destroy_node()
    
    

    # 你的導航邏輯
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')
    # aruco_navigator.destroy_node()
    navigator.lifecycleShutdown()

    rclpy.shutdown()
    exit(0)


if __name__ == '__main__':
    main()
