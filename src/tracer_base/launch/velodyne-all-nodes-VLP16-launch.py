# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Launch the velodyne driver, pointcloud, and laserscan nodes with default configuration."""

import os
import yaml

import ament_index_python.packages
import launch
import launch_ros.actions


def generate_launch_description():
    driver_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_driver')
    driver_params_file = os.path.join(driver_share_dir, 'config', 'VLP16-velodyne_driver_node-params.yaml')
    velodyne_driver_node = launch_ros.actions.Node(package='velodyne_driver',
                                                   executable='velodyne_driver_node',
                                                   output='both',
                                                   parameters=[driver_params_file])

    convert_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_pointcloud')
    convert_params_file = os.path.join(convert_share_dir, 'config', 'VLP16-velodyne_transform_node-params.yaml')
    with open(convert_params_file, 'r') as f:
        convert_params = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']

    convert_params['calibration'] = os.path.join(convert_share_dir, 'params', 'VLP16db.yaml')
    velodyne_transform_node = launch_ros.actions.Node(package='velodyne_pointcloud',
                                                      executable='velodyne_transform_node',
                                                      output='both',
                                                      parameters=[convert_params])

    laserscan_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_laserscan')
    laserscan_params_file = os.path.join(laserscan_share_dir, 'config', 'default-velodyne_laserscan_node-params.yaml')
    velodyne_laserscan_node = launch_ros.actions.Node(package='velodyne_laserscan',
                                                      executable='velodyne_laserscan_node',
                                                      output='both',
                                                      parameters=[laserscan_params_file],
                                                      remappings=[
                                                          ('/scan', '/velodyne_scan'),
                                                            ]
                                                        )
    pointcloud_to_laserscan_node = launch_ros.actions.Node(package='pointcloud_to_laserscan',
                                                           executable='pointcloud_to_laserscan_node',
                                                        #    parameters=[
                                                        #        {"min_height": -0.759},
                                                        #        {"angle_increment": 0.00349066}
                                                        #      ],
                                                           parameters=[{
                                                                'target_frame': 'base_link',    # 關鍵：轉到車體座標，TF 會自動處理 0.76m 的高度差
                                                                'transform_tolerance': 0.01,
                                                                
                                                                # 過濾參數直接寫「離地高度」
                                                                'min_height': 0.1,              # 地面以上 10cm 才算障礙物 (成功濾掉地板)
                                                                #'max_height': 1.0,              # 地面以上 1m 內都算 (太高像天花板就不算)
                                                                
                                                                'angle_min': -3.14159,
                                                                'angle_max': 3.14159,
                                                                'angle_increment': 0.0087,      # 降低一點解析度減輕負擔
                                                                'scan_time': 0.1,
                                                                'range_min': 0.45,              # 設為機器人半徑 (0.45)，過濾打到自己車身的點
                                                                'range_max': 100.0,
                                                                'use_inf': True
                                                            }],
                                                           remappings=[
                                                            ('cloud_in', '/velodyne_points'),
                                                            ('scan', '/scan'),
                                                              ])


    return launch.LaunchDescription([velodyne_driver_node,
                                     velodyne_transform_node,
                                     velodyne_laserscan_node,
                                     pointcloud_to_laserscan_node,
                                     launch.actions.RegisterEventHandler(
                                         event_handler=launch.event_handlers.OnProcessExit(
                                             target_action=velodyne_driver_node,
                                             on_exit=[launch.actions.EmitEvent(
                                                 event=launch.events.Shutdown())],
                                            )), 
    ])