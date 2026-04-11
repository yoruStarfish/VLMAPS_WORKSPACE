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

## 3. contruct vlmaps
- another terminal
```bash
ros2 run vlmaps_ros map_builder
ros2 service call /build_vlmap std_srvs/srv/Trigger
```

# VLMAPS INTORODUCTION

## generate_nav_maps
1. nav_obstacle_map.png
	這是一張黑白分明的 2D 俯視圖（白色是可以走的地板，黑色是牆壁、桌子腳等障礙物）。你可以直接用 ROS 2 的 map_server 把它發布出去，Nav2 裡面的 Global Costmap（全域代價地圖） 就會吃這張圖來規劃閃避障礙物的路線。
2. target_mask_chair.png
	這張圖裡面，只有被判定為「椅子」的地方會是白色的，其他都是黑色的。
	接下來你的 ROS 2 導航節點只需要讀這張圖，用 OpenCV 算出這塊白色區域的**「幾何中心點 (Centroid)」**，那個點的 (X, Y) 座標，就是你要送給 Nav2 的目標座標（Goal Pose）！