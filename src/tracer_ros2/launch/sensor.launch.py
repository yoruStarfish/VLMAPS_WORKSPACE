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

    # 2. 啟動 Velodyne 雷達 (從 tracer_base package 呼叫)
    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('tracer_base'), 'launch', 'velodyne-all-nodes-VLP16-launch.py')
        ])
    )

    # 3. 啟動 RealSense 相機，並帶入詳細參數
        # 啟動了 Intel RealSense 深度相機（如 D435i 或 D455），並設定了詳細參數：
        #   initial_reset (true): 啟動時強制重啟相機。這對於防止相機連線不穩或 USB 錯誤非常有用
        #   align_depth.enable (true): 開啟深度圖與彩色圖的對齊。這對於生成彩色的點雲 (RGB-D Pointcloud) 是必須的
        #   rgb_camera.color_profile (1920,1080,30): 彩色鏡頭解析度設為 1080p，幀率 30 FPS
        #   depth_module.depth_profile (1280,720,30): 深度鏡頭解析度設為 720p，幀率 30 FPS
        #   enable_gyro & enable_accel (true): 開啟陀螺儀和加速規（即開啟 IMU 功能）
        #   unite_imu_method (2): 設定如何合併陀螺儀和加速規的數據。數值 2 通常代表使用線性插值 (Linear interpolation) 來將兩者合併為一個 imu topic。
        #   pointcloud.enable (true): 關鍵設定。這會讓相機節點計算並發布 3D 點雲資料（通常 topic 是 /camera/depth/color/points），這對於 3D 建圖 (如 RTAB-Map) 至關重要
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ]),
        launch_arguments={
            'initial_reset': 'true',
            'align_depth.enable': 'true',
            'rgb_camera.color_profile': '1920,1080,30',
            'depth_module.depth_profile': '1280,720,30',
            'queue_size': '10',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '2',
            'pointcloud.enable': 'true',
        }.items()
    )

    # 4. 發布靜態座標轉換 (TF) - base_link 到 camera_link
        # 相機安裝在機器人中心 (base_link) 的：
        #     前方 0.25 公尺 (X = 0.25)
        #     高度 0.69 公尺 (Z = 0.69)
        #     沒有旋轉 (Yaw/Pitch/Roll 都是 0)
    tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='baselink_to_camera',
        # arguments 對應: x, y, z, yaw, pitch, roll, parent_frame, child_frame
        arguments=['0.25', '0.0', '0.69', '0.0', '0.0', '0.0', 'base_link', 'camera_link']
    )

    # 5. 發布靜態座標轉換 (TF) - base_link 到 velodyne
        # 雷達安裝在機器人中心 (base_link) 的：
        #     正中央 (X = 0.0, Y = 0.0)
        #     高度 0.76 公尺 (Z = 0.76)
        #     沒有旋轉 (Yaw/Pitch/Roll 都是 0)
    tf_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='baselink_to_velodyne',
        # arguments 對應: x, y, z, yaw, pitch, roll, parent_frame, child_frame
        arguments=['0.0', '0.0', '0.76', '6.28', '0.0', '0.0', 'base_link', 'velodyne']
    )

    # 將所有準備好的動作打包並回傳，ROS 2 就會一口氣把它們全部啟動
    return LaunchDescription([
        tracer_base_launch,
        velodyne_launch,
        realsense_launch,
        tf_camera,
        tf_velodyne
    ])