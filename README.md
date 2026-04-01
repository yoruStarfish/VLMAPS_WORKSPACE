# Activate Virtual Environment
## 1. terminal
ros2 launch tracer_ros2 sensor.launch.py

conda activate vlmaps
source ~/vlmaps_ws/install/setup.bash

we need colcon build in vlmaps_ws!

## data_collector_node
- collect data, include: rgb, depth, pose(robot or camera)
```bash=
	ros2 launch slam.launch.py # can process it in virtural environment or not
	ros2 run vlmaps_ros data_collector
```
- then, turn off the data controller

## contruct vlmaps
- another terminal
```bash
ros2 run vlmaps_ros map_builder
ros2 service call /build_vlmap std_srvs/srv/Trigger
```

