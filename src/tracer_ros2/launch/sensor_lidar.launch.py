import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # 1. 啟動 Tracer 底盤
    tracer_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('tracer_base'), 'launch', 'tracer_base.launch.py')
        ])
    )

    # 2. 啟動雷神 C16 V4.0 官方驅動（需安裝 C16_V4.0 分支）。
    # 預設 YAML: device_ip=192.168.1.200，UDP 2368/2369。
    # 官方 launch namespace=cx，點雲為 /cx/lslidar_point_cloud。
    lslidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('lslidar_driver'), 'launch', 'lslidar_cx_launch.py')
        ])
    )

    # 3. frame 必須與官方 YAML 的 frame_id=laser_link 一致。
    # 0.71 m 與零旋轉暫沿用既有設定，需依 C16 原點及安裝方向實測確認。
    tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='baselink_to_lidar',
        # arguments 對應: x, y, z, yaw, pitch, roll, parent_frame, child_frame
        # 0.063 + 0.03387 = 0.09687 m，約 0.097 m
        arguments=['0.0', '0.0', '0.097', '1.57079632679', '0.0', '0.0', 'base_link', 'laser_link']
    )

    # 保留原本供導航使用的 /scan，由 C16 點雲投影產生。
    scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.01,
            'min_height': 0.1,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.45,
            'range_max': 100.0,
            'use_inf': True,
        }],
        remappings=[
            ('cloud_in', '/cx/lslidar_point_cloud'),
            ('scan', '/scan'),
        ],
    )

    # 將所有準備好的動作打包並回傳，ROS 2 就會一口氣把它們全部啟動
    return LaunchDescription([
        tracer_base_launch,
        lslidar_launch,
        tf_lidar,
        scan,
    ])
