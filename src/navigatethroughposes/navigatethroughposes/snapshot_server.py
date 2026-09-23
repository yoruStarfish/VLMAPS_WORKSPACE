import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from cv_bridge import CvBridge
import cv2
import numpy as np

class SnapshotServer(Node):
    def __init__(self):
        super().__init__('snapshot_server')

        # 工具: ROS Image <-> OpenCV 轉換器
        self.cv_bridge = CvBridge()

        # 儲存歷史照片的列表 (List of cv2 images)
        self.image_history = []

        # 1. 建立 Publisher
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.panel_pub = self.create_publisher(Image, '/goal_snapshot', latched_qos)

        # 2. 建立 Subscriber (持續接收相機畫面)
        self.latest_msg = None
        self.create_subscription(
            Image, 
            '/camera/camera/color/image_raw', 
            self.image_callback, 
            10
        )

        # 3. 建立 Service Server
        self.srv = self.create_service(Trigger, '/trigger_snapshot', self.trigger_callback)
        
        # 4. (選用) 建立一個 Service 來清空相簿
        self.clear_srv = self.create_service(Trigger, '/clear_snapshots', self.clear_callback)

        self.get_logger().info("Snapshot Album Server Ready!")

    def image_callback(self, msg):
        self.latest_msg = msg

    def trigger_callback(self, request, response):
        if self.latest_msg is None:
            response.success = False
            response.message = "No camera image received yet!"
            return response

        try:
            # 1. 將 ROS Image 轉為 OpenCV 格式
            current_cv_image = self.cv_bridge.imgmsg_to_cv2(self.latest_msg, desired_encoding='bgr8')
            
            # 2. (選用) 縮小一點圖片，不然拼太多張會太大導致 RViz 變慢
            # 這裡將高度固定為 240 pixel，寬度等比例縮放
            height, width = current_cv_image.shape[:2]
            new_height = 240
            scale_factor = new_height / height
            new_width = int(width * scale_factor)
            resized_img = cv2.resize(current_cv_image, (new_width, new_height))

            # 3. 加入歷史列表
            self.image_history.append(resized_img)

            # 4. 拼接圖片 (這裡是水平拼接 Horizontal Stack)
            # 如果你想要垂直拼接，改用 np.vstack
            # 我們在圖片之間加一條 5 pixel 的黑線做分隔
            separator = np.zeros((new_height, 5, 3), dtype=np.uint8) # 黑線
            
            final_image = self.image_history[0]
            for img in self.image_history[1:]:
                final_image = np.hstack((final_image, separator, img)) # 以圖片的形式發布

            # 5. 轉回 ROS Image 並發布
            out_msg = self.cv_bridge.cv2_to_imgmsg(final_image, encoding='bgr8')
            self.panel_pub.publish(out_msg)

            response.success = True
            response.message = f"Snapshot added! Total images: {len(self.image_history)}"
            self.get_logger().info(f"Published album with {len(self.image_history)} images.")

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")
            response.success = False
            response.message = str(e)
        
        return response

    def clear_callback(self, request, response):
        self.image_history = []
        # 發布一張黑畫面或是空的
        empty_img = np.zeros((100, 100, 3), dtype=np.uint8)
        out_msg = self.cv_bridge.cv2_to_imgmsg(empty_img, encoding='bgr8')
        self.panel_pub.publish(out_msg)
        
        response.success = True
        response.message = "Album cleared."
        self.get_logger().info("Snapshot album cleared.")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = SnapshotServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()