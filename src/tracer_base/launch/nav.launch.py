import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    parameters = [{
        'frame_id': 'camera_link',
        'subscribe_depth': True,
        'subscribe_odom_info': True,
        'approx_sync': True,
        'wait_imu_to_init': True,
    }]

    remappings = [
        ('imu', '/imu/data'),
        ('rgb/image', '/camera/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/camera/color/camera_info'),
        ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw')
    ]

    return LaunchDescription([
        # Make sure IR emitter is enabled
        SetParameter(name='depth_module.emitter_enabled', value=1),

        # Include RTAB-Map launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('rtabmap_launch'), 'launch'),
                '/rtabmap.launch.py']),
            launch_arguments={
                'odom_frame_id': 'odom',
                'approx_sync': 'true',
                'rgb_topic': '/camera/camera/color/image_raw',
                'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
                'camera_info_topic': '/camera/camera/color/camera_info',
                'subscribe_scan_cloud': 'true',
                'visual_odometry': 'false',
                'odom_topic': '/odom',
                'rviz': 'true',
                'rtabmap_viz': 'false',
                'localization': 'true',
                'database_path': '/home/robotic/ros2_ws/Map/9Frtabmap.db'
            }.items(),
        ),

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            namespace='rtabmap',
            remappings=[('/rtabmap/grid_prob_map', '/map')],
        ),
        
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            output='screen',
            parameters=parameters,
            remappings=remappings,
        ),

        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            output='screen',
            parameters=parameters,
            remappings=remappings,
        ),

        # Compute quaternion of the IMU using Madgwick filter
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            output='screen',
            parameters=[{'use_mag': False, 'world_frame': 'enu', 'publish_tf': False}],
            remappings=[
                ('imu/accel', '/camera/camera/accel/sample'),
                ('imu/gyro', '/camera/camera/gyro/sample'),
                ('imu/data', '/imu/data')
            ],
        ),

        # The IMU frame is missing in TF tree, add it:
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'camera_gyro_optical_frame', 'camera_imu_optical_frame'],
        ),
    ])
