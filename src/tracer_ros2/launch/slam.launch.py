import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # 找到 rtabmap_launch 套件的路徑
    rtabmap_pkg_dir = get_package_share_directory('rtabmap_launch')

    # 準備呼叫 rtabmap.launch.py 並帶入所有學長姊調校好的參數
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(rtabmap_pkg_dir, 'launch', 'rtabmap.launch.py')
        ]),
        launch_arguments={
            # ================= 基礎設定與 Topic 綁定 =================
            'odom_frame_id': 'odom',
            'odom_topic': '/odom',
            'approx_sync': 'true',
            'topic_queue_size': '30',
            'sync_queue_size': '20',
            'queue_size': '10',
            
            # 🚨 注意：這裡保留了你原本的 /camera/camera/... 
            # 如果等一下跑起來沒收到資料，請改成 /camera/color/image_raw
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            
            'scan_topic': '/velodyne_scan',
            'scan_cloud_topic': '/velodyne_points',
            
            # 開關設定
            'subscribe_scan': 'true',
            'subscribe_depth': 'true',
            'subscribe_rgb': 'true',
            'visual_odometry': 'false',
            'rtabmap_viz': 'false',
            'rviz': 'true',
            'rtabmap_args': '-d', # 每次啟動刪除舊地圖，重新建圖
            
            # ================= 核心演算法調校 (Jessica's work) =================
            'Rtabmap/DetectionRate': '0', # Rate (Hz) at which new nodes are added to ma
            'Rtabmap/LoopThr': '0.1',
            
            # 視覺特徵設定 (ORB, SURF 等)
            'Vis/MinInliers': '15', # Minimum visual inliers to accept loop closure
            'Vis/FeatureType': '2',
            'Kp/DetectorStrategy': '2', # 0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE
            'Kp/MaxFeatures': '500', # Maximum number of features to detect (for ORB, this is the number of keypoints) 0:代表無限大
            'SURF/HessianThreshold': '100', # SURF 特徵檢測的敏感度，數值越低越容易檢測到特徵點，但可能會增加誤匹配。一般建議從 100 開始調整。 Ued to extract more or less SURF features
            
            # ICP (雷達點雲配準) 設定
            'Icp/VoxelSize': '0.05',
            'Icp/MaxCorrespondenceDistance': '0.1',
            
            # 2D 網格地圖 (Grid Map) 過濾設定
            'Grid/MaxGroundHeight': '-0.3',
            'Grid/MaxObstacleHeight': '1.0',
            'Grid/NormalsSegmentation': 'true',
            'Grid/Sensor': '0',
            
            # 記憶體與圖優化設定
            'Mem/NotLinkedNodesKept': 'false',
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',
            'Reg/Strategy': '2',
            'Reg/Force3DoF': 'false',
            
            # RGB-D 迴圈閉合與最佳化
            'RGBD/NeighborLinkRefining': 'true',
            'RGBD/ProximityBySpace': 'true',
            'RGBD/AngularUpdate': '0.5',
            'RGBD/LinearUpdate': '0.5',
            'RGBD/OptimizeFromGraphEnd': 'true',
            'RGBD/ProximityPathMaxNeighbors': '10',
            'RGBD/LoopClosureReextractFeatures': 'true',
        }.items()
    )

    return LaunchDescription([
        rtabmap_launch
    ])