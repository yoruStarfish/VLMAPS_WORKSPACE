from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    tracer_base_dir = get_package_share_directory('tracer_base')
    rplidar_dir = get_package_share_directory('rplidar_ros')
    realsense_dir = get_package_share_directory('realsense2_camera')

    return LaunchDescription([
        # Tracer base
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tracer_base_dir, 'launch', 'tracer_base.launch.py')
            )
        ),

        # RPLIDAR A3
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rplidar_dir, 'launch', 'rplidar_a3_launch.py')
            )
        ),

        # RealSense Camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(realsense_dir, 'launch', 'rs_launch.py')
            ),
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
        ),

        # Static transform: base_link → camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='baselink_to_camera',
            arguments=['0.25', '0', '0.69', '0', '0', '0', 'base_link', 'camera_link']
        ),

        # Static transform: base_link → laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='baselink_to_laser',
            arguments=['0.0', '0', '0.71', '3.14', '0', '0', 'base_link', 'laser']
        ),

        # # Laser filter node
        # Node(
        #     package='laser_filters',
        #     executable='scan_to_scan_filter_chain',
        #     name='scan_filter_node',
        #     output='screen',
        #     parameters=[
        #         PathJoinSubstitution([
        #             get_package_share_directory("tracer_base"),
        #             "config", "scan_filter.yaml",
        #         ])]
        # ),
    ])
