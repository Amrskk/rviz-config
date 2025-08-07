# Lane Extractor Package

This ROS2 package subscribes to `/sensor/lidar_front/points`, processes the point cloud to extract road lane edges using KMeans clustering, and visualizes them in RViz as Marker LINE_STRIP messages.

## How to Run

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 run lane_extractor_pkg lane_extractor
or ros2 run lane_extractor_pkg lane_extractor --ros-args -p input_topic:=/sensor/lidar_rear/points
