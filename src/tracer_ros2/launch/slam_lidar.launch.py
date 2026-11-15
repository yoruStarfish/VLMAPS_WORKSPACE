
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    rtabmap_pkg_dir = get_package_share_directory('rtabmap_launch')

    parameters = {
        # RTAB-Map
        'Rtabmap/DetectionRate': '0',

        # ICP registration
        'Reg/Strategy': '1',
        'Reg/Force3DoF': 'true',
        'Icp/VoxelSize': '0.05',
        'Icp/MaxCorrespondenceDistance': '0.1',

        # Occupancy grid
        'Grid/Sensor': '0',
        'Grid/3D': 'false',
        'Grid/RayTracing': 'true',
        'Grid/CellSize': '0.05',
        'Grid/NormalsSegmentation': 'true',
        'Grid/MaxObstacleHeight': '1.0',

        # Memory management
        'Mem/NotLinkedNodesKept': 'false',
        'Mem/IncrementalMemory': 'true',
        'Mem/InitWMWithAllNodes': 'false',

        # Graph optimization
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/AngularUpdate': '0.5',
        'RGBD/LinearUpdate': '0.5',
        'RGBD/OptimizeFromGraphEnd': 'true',
        'RGBD/ProximityPathMaxNeighbors': '10',
    }

    # Delete the previous database and start a new mapping session.
    rtabmap_args = '-d ' + ' '.join(
        f'--{key} {value}' for key, value in parameters.items()
    )

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                rtabmap_pkg_dir,
                'launch',
                'rtabmap.launch.py'
            )
        ),
        launch_arguments={
            # Reference frames
            'frame_id': 'base_link',
            'map_frame_id': 'map',

            # Use Tracer odom -> base_link TF.
            'odom_frame_id': 'odom',
            'odom_topic': '/odom',

            # Disable RTAB-Map odometry nodes.
            'visual_odometry': 'false',
            'icp_odometry': 'false',

            # Synchronization
            'approx_sync': 'true',
            'topic_queue_size': '30',
            'sync_queue_size': '20',
            'queue_size': '10',

            # Disable camera inputs.
            'depth': 'false',
            'subscribe_rgb': 'false',
            'subscribe_rgbd': 'false',
            'stereo': 'false',
            'rgbd_sync': 'false',
            'compressed': 'false',
            'wait_imu_to_init': 'false',

            # Use the C16 3D point cloud.
            'subscribe_scan': 'false',
            'subscribe_scan_cloud': 'true',
            'scan_cloud_topic': '/cx/lslidar_point_cloud',

            # Visualization
            'rtabmap_viz': 'false',
            'rviz': 'false',

            # Core RTAB-Map parameters
            'rtabmap_args': rtabmap_args,
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            os.path.join(
                rtabmap_pkg_dir,
                'launch',
                'config',
                'rgbd.rviz'
            )
        ],
    )

    return LaunchDescription([rtabmap_launch, rviz])
