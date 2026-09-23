import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    rtabmap_pkg_dir = get_package_share_directory('rtabmap_launch')

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                rtabmap_pkg_dir,
                'launch',
                'rtabmap.launch.py'
            )
        ]),
        launch_arguments={

            # ================= 基礎設定 =================
            'odom_frame_id': 'odom',
            # 確認這裡的 topic 名稱與你底盤發布的里程計名稱一致
            'odom_topic': '/odom',
            'approx_sync': 'true',
            'topic_queue_size': '30',
            'sync_queue_size': '20',
            'queue_size': '10',
            'imu_topic': '/camera/camera/accel/imu_info',

            # ================= RGB-D 相機 =================
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'subscribe_rgb': 'true',
            'subscribe_depth': 'true',
            'subscribe_scan': 'false',
            'subscribe_imu': 'true',

            # ================= Odometry (關鍵切換) =================
            # 改為 true，開啟視覺里程計 (Visual Odometry) 進行特徵追蹤防飄移
            'visual_odometry': 'true',

            # ================= UI =================
            'rtabmap_viz': 'false',
            'rviz': 'true',
            'rtabmap_args': '-d',

            # ================= Visual Features =================
            'Rtabmap/DetectionRate': '5', 
            'Rtabmap/LoopThr': '0.1',
            'Kp/MaxFeatures': '1500', 
            
            # ================= Grid Map (保留降<噪設定) =================
	        # [修正] 明確告訴系統我們是用深度圖建地圖，不是 LiDAR
            'Grid/FromDepth': 'true',
            'Grid/MaxGroundHeight': '-0.3', 
            'Grid/MaxObstacleHeight': '1.0', 
            'Grid/MinObstacleHeight': '0.3', 
	        'Grid/RangeMin': '0.25',  # [新增] 忽略 0.25 公尺內的極近距離垃圾雜訊
            'Grid/RangeMax': '2.0',
            'Grid/NormalsSegmentation': 'true',
            'Grid/Sensor': '0', 
            'Grid/NoiseFilteringMinNeighbors': '15', # 提高到 15 (周圍要有25個點才承認是障礙物)
            'Grid/NoiseFilteringRadius': '0.15', 
            'Grid/DepthDecimation': '4', # [新增] 將深度圖長寬各縮小 4 倍再投影，能大幅消滅邊緣雜訊並降低 CPU 負擔
            'Grid/RayTracing': 'true',   # [新增] 走過確認沒東西的地方，強制把地圖洗成白色 (Free Space)
            'Grid/3D': 'false',          # [新增] 強制只建 2D 圖，不保留 3D 體素，減少 Z 軸飄移影響

            # ================= Memory =================
            'Mem/NotLinkedNodesKept': 'false',
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',

            'Reg/Strategy': '2', 
            'Reg/Force3DoF': 'true',

            # ================= RGB-D 進階 =================
            'RGBD/NeighborLinkRefining': 'true',
            'RGBD/ProximityBySpace': 'true',
            'RGBD/OptimizeFromGraphEnd': 'true',

        }.items()
    )

    return LaunchDescription([
        rtabmap_launch
    ])