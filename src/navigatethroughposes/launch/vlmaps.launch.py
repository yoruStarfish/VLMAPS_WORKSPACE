import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_dir = get_package_share_directory('navigatethroughposes')

    conda_python_path = "/opt/miniforge3/envs/vlmaps310/bin/python"

    vlmap_script = os.path.join(pkg_dir, '..', '..', 'lib', 'navigatethroughposes', 'navi_using_api_vlmap')

    return LaunchDescription([
        Node(
            package='navigatethroughposes',   # 任意已存在的 package
            executable=conda_python_path,
            arguments=[vlmap_script],
            name='navi_vlmaps_node',
            output='screen',
        ),
    ])