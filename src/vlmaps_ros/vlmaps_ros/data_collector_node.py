import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import message_filters
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from scipy.spatial.transform import Rotation as R
from pathlib import Path


class VLMapsDataCollector(Node):
    def __init__(self):
        super().__init__('vlmaps_data_collector')
        self.bridge = CvBridge()
        
        # --- 1. 設定存檔資料夾 ---
        self.base_dir = os.path.expanduser('~/vlmaps_ws') # 以使用者主目錄為基底
        self.save_dir = os.path.join(self.base_dir, 'dataset') # 你可以改成你想要的資料夾
        self.rgb_dir = os.path.join(self.save_dir, 'rgb')
        self.depth_dir = os.path.join(self.save_dir, 'depth')
        self.pose_dir = os.path.join(self.save_dir, 'pose')
        os.makedirs(self.rgb_dir, exist_ok=True)
        os.makedirs(self.depth_dir, exist_ok=True)
        os.makedirs(self.pose_dir, exist_ok=True)
        
        self.frame_count = 0 # 用來替檔案命名 (00000, 00001...)

        # --- 2. 設定你的 Topic 和 Frame 名稱 ---
        # 這些要替換成你實機或模擬器上的真實名稱！
        self.rgb_topic = '/camera/camera/color/image_raw'
        self.depth_topic = '/camera/camera/aligned_depth_to_color/image_raw'
        self.map_frame = 'map'          # RTAB-Map 的全域座標系
        self.camera_frame = 'camera_link' # 攝影機的座標系

        # --- 3. 初始化 TF 監聽器 (用來查座標) ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- 4. 設定影像同步訂閱 (Time Synchronizer) ---
        self.sub_rgb = message_filters.Subscriber(self, Image, self.rgb_topic)
        self.sub_depth = message_filters.Subscriber(self, Image, self.depth_topic)

        self.last_saved_pose = None
        self.min_dist_threshold = 0.2  # 移動超過 20 公分才存檔
        self.min_angle_threshold = 0.1 # 旋轉超過約 5.7 度 (0.1 弧度) 才存檔
        
        # 允許 0.1 秒的誤差，把時間相近的 RGB 和 Depth 綁在一起送給 callback
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_rgb, self.sub_depth], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.sync_callback)
        
        self.get_logger().info("✅ VLMaps 資料收集節點已啟動！正在等待影像...")

    def sync_callback(self, rgb_msg, depth_msg):
        timestamp = rgb_msg.header.stamp
        
        try:
            trans = self.tf_buffer.lookup_transform(
                self.map_frame, 
                self.camera_frame, 
                rclpy.time.Time(), # 改成 Time()，代表「給我最新的，不要管時間差」
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"找不到 TF 座標轉換，跳過這張影像: {e}")
            return
        
        # --- 1. 先把 4x4 Pose 矩陣和目前位置算出來 ---
        t = trans.transform.translation
        q = trans.transform.rotation
        
        rot_matrix = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        pose_matrix = np.eye(4)
        pose_matrix[:3, :3] = rot_matrix
        pose_matrix[:3, 3] = [t.x, t.y, t.z]
        
        current_pos = np.array([t.x, t.y, t.z])

        # --- 2. 進行 Keyframe 距離判斷 ---
        if self.last_saved_pose is not None:
            last_pos = self.last_saved_pose[:3, 3]
            dist = np.linalg.norm(current_pos - last_pos)
            
            # 如果移動距離太短，直接跳出不存檔
            if dist < self.min_dist_threshold:
                return
        
        # --- 3. 通過檢查！更新記憶體並準備存檔 ---
        self.last_saved_pose = pose_matrix

        # 將 ROS 影像轉換為 OpenCV 格式
        cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        # 產生檔名
        filename = f"{self.frame_count:05d}"
        
        # 執行實體存檔
        cv2.imwrite(os.path.join(self.rgb_dir, f"{filename}.png"), cv_rgb)
        depth_array_meters = cv_depth.astype(np.float32) / 1000.0
        cv2.imwrite(os.path.join(self.depth_dir, f"{filename}.png"), (depth_array_meters * 1000).astype(np.uint16))
        np.save(os.path.join(self.depth_dir, f"{filename}.npy"), depth_array_meters)
        np.savetxt(os.path.join(self.pose_dir, f"{filename}.txt"), pose_matrix)

        self.get_logger().info(f"💾 偵測到足夠位移 (Keyframe)，成功儲存 Frame: {filename}")
        self.frame_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = VLMapsDataCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()