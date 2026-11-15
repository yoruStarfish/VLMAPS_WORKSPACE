# VLMap Project

## Data Collection
Use RTAB-Map (SLAM), currently, both LiDAR and RGB-D camera are being used.
- Once all the data has bee collected, 
	- first, run the `vlmapsSrc/filter_rgb_depth_poses.py` to extract keyframe data.
	- second, run the `vlmapsSrc/convert_depth_to_npy.py` to convert depth from '.png' to '.npy'.


In the future, I want to use only RGB-D SLAM in this project.

## Construct VLMap and Obstacle Map
- Run the python code in `vlmapsSrc/vlmaps/map/vlmap_tracer.py` to construct vlmap and obstacle map.
- Before running, check whether the config file `vlmapsSrc/config/map_config/vlmaps_tracer.yaml` is fit on your robot.
- Before running, please activate conda virtual environment `vlmaps310`.


# Activate Virtual Environment

## 1. terminal
ros2 launch tracer_ros2 sensor.launch.py

conda activate vlmaps
source ~/vlmaps_ws/install/setup.bash

we need colcon build in vlmaps_ws!

## 2. data_collector_node
- collect data, include: rgb, depth, pose(robot or camera)
```bash=
	ros2 launch slam.launch.py # can process it in virtural environment or not
	ros2 run vlmaps_ros data_collector
```
- then, turn off the data controller

## 3. construct vlmaps
- another terminal
```bash
ros2 run vlmaps_ros map_builder
ros2 service call /build_vlmap std_srvs/srv/Trigger
```

# LLM 串進來
```bash
ros2 run navigatethroughposes llm_cmd
```

# VLMAPS INTORODUCTION

## generate_nav_maps
1. nav_obstacle_map.png
	這是一張黑白分明的 2D 俯視圖（白色是可以走的地板，黑色是牆壁、桌子腳等障礙物）。你可以直接用 ROS 2 的 map_server 把它發布出去，Nav2 裡面的 Global Costmap（全域代價地圖） 就會吃這張圖來規劃閃避障礙物的路線。
2. target_mask_chair.png
	這張圖裡面，只有被判定為「椅子」的地方會是白色的，其他都是黑色的。
	接下來你的 ROS 2 導航節點只需要讀這張圖，用 OpenCV 算出這塊白色區域的**「幾何中心點 (Centroid)」**，那個點的 (X, Y) 座標，就是你要送給 Nav2 的目標座標（Goal Pose）！

# ROS2 bag
If you want to recode the ros2 topic data, use the following command:
```bash
ros2 bag record -o right_turn_debug_ver23 \
  /cmd_vel_nav \
  /cmd_vel_smoothed \
  /cmd_vel_filtered \
  /cmd_vel \
  /odom \
  /tf \
  /tf_static \
  /scan \
  /plan \
  /global_costmap/costmap \
  /global_costmap/costmap_updates \
  /global_costmap/published_footprint \
  /local_costmap/costmap \
  /local_costmap/costmap_updates \
  /local_costmap/published_footprint \
  /rosout
```