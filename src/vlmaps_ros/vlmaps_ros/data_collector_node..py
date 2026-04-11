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
        # 當收到一組對齊的彩色與深度影像時，就會觸發這個函數
        
        # 1. 取得當下的時間戳記
        timestamp = rgb_msg.header.stamp
        
        try:
            # 2. 查詢此時此刻，攝影機在 map 座標系中的位置 (Pose)
            trans = self.tf_buffer.lookup_transform(
                self.map_frame, 
                self.camera_frame, 
                timestamp, 
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"找不到 TF 座標轉換，跳過這張影像: {e}")
            return
        
        # 取得目前的 x, y, z 和四元數
        t = trans.transform.translation
        current_pos = np.array([t.x, t.y, t.z])

        # --keyframe 選擇邏輯--
        if self.last_saved_pose is not None:
            last_pos = self.last_saved_pose[:3, 3]
            # 計算歐式距離 (Euclidean distance)
            dist = np.linalg.norm(current_pos - last_pos)
            
            # (進階) 這裡也可以加入旋轉角度的計算
            
            # 如果移動距離太短，就直接 return 跳過，不存檔！
            if dist < self.min_dist_threshold:
                return
        
        # --- 如果通過檢查，就往下執行存檔，並更新 last_saved_pose ---
        self.last_saved_pose = pose_matrix
        self.get_logger().info(f"💾 偵測到足夠位移 (Keyframe)，成功儲存 Frame: {filename}")

        # 3. 將 ROS 影像轉換為 OpenCV 格式
        cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        # 深度圖通常是 16-bit 或是 32-bit float，這裡假設是常見的 16-bit 毫米單位
        cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        # 4. 把座標轉換成 4x4 的數學矩陣 (VLMaps 喜歡這種格式)
        t = trans.transform.translation
        q = trans.transform.rotation
        
        # 將四元數 (Quaternion) 轉成 3x3 旋轉矩陣
        rot_matrix = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        
        # 組合出 4x4 的齊次變換矩陣 (Homogeneous Transformation Matrix)
        pose_matrix = np.eye(4)
        pose_matrix[:3, :3] = rot_matrix
        pose_matrix[:3, 3] = [t.x, t.y, t.z]

        # 5. 存檔！
        filename = f"{self.frame_count:05d}" # 產出 00000, 00001 的字串
        
        cv2.imwrite(os.path.join(self.rgb_dir, f"{filename}.png"), cv_rgb)
        cv2.imwrite(os.path.join(self.depth_dir, f"{filename}.png"), cv_depth)
        np.savetxt(os.path.join(self.pose_dir, f"{filename}.txt"), pose_matrix)

        self.get_logger().info(f"💾 成功儲存 Frame: {filename}")
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