# lane_extractor_pkg

> **Multi-LiDAR lane detection for ROS 2 (Jazzy)**  
> Extracts and visualizes road lane lines from 3D LiDAR point clouds using KMeans clustering, Open3D filtering, and Rviz visualization.

---

This ROS 2 node (`lane_extractor`) subscribes to **three LiDAR sensors**:
- `/sensor/lidar_front/points`
- `/sensor/lidar_left/points`
- `/sensor/lidar_right/points`

It:
- Merges and downsamples incoming point clouds (via Open3D)
- Filters points within a road-level ROI
- Applies KMeans clustering to detect lane boundaries
- Publishes left/right lanes as `Marker` messages for RViz2

---

## Dependencies

Make sure the following packages are installed:

### ROS 2 (Jazzy)
```bash
sudo apt install ros-jazzy-message-filters
```

---

## Run the package
```bash
ros2 run lane_extractor_pkg lane_extractor
```
