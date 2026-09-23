from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    parameter=[{
    'subscribe_rgbd': True,            # ✅ 開啟 RGBD 模式（必要！）
    'subscribe_scan_cloud': True,      # ✅ 同時使用 Velodyne 點雲
    'subscribe_depth': False,          # ✅ 關閉單獨 depth 模式
    'subscribe_rgb': False,            # ✅ 關閉單獨 RGB 模式
    'subscribe_scan': False,

    'approx_sync': True,
    'wait_imu_to_init': True,
    'frame_id': 'base_link',
    'odom_frame_id': 'odom',

    # Odometry 與 SLAM 參數
    # 'Reg/Strategy': "2",
    'Reg/Force3DoF': "true",
    'Mem/NotLinkedNodesKept': "false",
    'Mem/InitWMWithAllNodes': "false",
    'Mem/IncrementalMemory': "true",
    'Icp/VoxelSize': "0.05",
    'Icp/MaxCorrespondenceDistance': "0.1",
    'RGBD/NeighborLinkRefining': "true",
    'RGBD/ProximityBySpace': "true",
    'RGBD/AngularUpdate': "0.01",
    'RGBD/LinearUpdate': "0.01",
    'RGBD/OptimizeFromGraphEnd': "false",
    'RGBD/ProximityPathMaxNeighbors': "10",

    # Grid地圖參數
    'Grid/Sensor': "0",
    'Grid/MaxGroundHeight': "-0.7",
    'Grid/MaxObstacleHeight': "0.5",
    'Grid/NormalsSegmentation': "false",

    # 視覺設定
    'Vis/MinInliers': "15",
    'Vis/FeatureType': "2",
    'Kp/DetectorStrategy': "2",
    'Kp/MaxFeatures': "400",
    'SURF/HessianThreshold': "100",

    # 其他
    # 'queue_size': 10,
    'sync_queue_size': 20,
    'topic_queue_size': 5,
    'Rtabmap/DetectionRate': "10",
    'visual_odometry': False,
    'rtabmap_viz': "true",
    'rviz': "true"
    }]

    remappings = [
    ('imu', '/imu/data'),
    ('rgbd_image', '/camera/camera/color/image_raw'),
    ('camera_info', '/camera/camera/color/camera_info'),
    ('depth_image', '/camera/camera/aligned_depth_to_color/image_raw'),
    ('scan_cloud', '/velodyne_points'),     # ✅ 必須用 scan_cloud_topic
    ('grid_map', '/map')
    ]

    return LaunchDescription([
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameter,
            remappings=remappings
        ),
        Node(
            package='rtabmap_slam', executable='rtabmap', 
            output='screen',
            parameters=parameter,
            remappings=remappings,
            arguments=['-d']
        ),
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', 
            output='screen',
            parameters=parameter,
            remappings=remappings
        )
    ])