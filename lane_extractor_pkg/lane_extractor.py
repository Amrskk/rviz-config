import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from message_filters import Subscriber, ApproximateTimeSynchronizer

import numpy as np
import open3d as o3d
from sklearn.cluster import KMeans
from sensor_msgs_py import point_cloud2
from builtin_interfaces.msg import Duration

# --- ПАРАМЕТРЫ НАСТРОЙКИ --- #

# Пределы ROI (области интереса)
ROI_X = (0.0, 25.0)   # Спереди (от 0 до 25 метров)
ROI_Y = (-6.0, 6.0)   # По бокам
ROI_Z = (-0.2, 0.2)   # Почти ровная дорога

# Параметры voxel downsampling
VOXEL_SIZE = 0.2  # Чем больше — тем меньше точек (0.1–0.3 оптимально)

# Параметры кластеризации
NUM_CLUSTERS = 6          # KMeans: число кластеров
MIN_CLUSTER_POINTS = 15   # Минимум точек в кластере
MIN_CLUSTER_LENGTH = 2.0  # Минимальная длина кластера по X

# Пороговая разница от медианы по Y для определения левой/правой линии
LANE_OFFSET_Y = 0.5

# Сглаживание: окно скользящего среднего
SMOOTHING_WINDOW = 5

# Визуальные параметры
LINE_WIDTH = 0.1
FRAME_ID = "base_link"

# Цвета RGBA (r, g, b, a)
COLOR_LEFT = (1.0, 0.0, 0.0, 1.0)   # Красный
COLOR_RIGHT = (0.0, 1.0, 0.0, 1.0)  # Зелёный


# --- УТИЛИТЫ --- #

def ros_pointcloud2_to_xyz(msg):
    return np.array([
        [p[0], p[1], p[2]]
        for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    ])


def voxel_downsample(cloud, voxel_size=0.2):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(cloud)
    pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    return np.asarray(pcd.points)


def moving_average_smooth(points, window=5):
    if len(points) < window:
        return points
    smoothed = []
    for i in range(len(points) - window + 1):
        avg = np.mean(points[i:i + window], axis=0)
        smoothed.append(avg)
    return smoothed


# --- ОСНОВНОЙ КЛАСС --- #

class LaneExtractor(Node):
    def __init__(self):
        super().__init__('lane_extractor')
        self.sub_front = Subscriber(self, PointCloud2, '/sensor/lidar_front/points')
        self.sub_left = Subscriber(self, PointCloud2, '/sensor/lidar_left/points')
        self.sub_right = Subscriber(self, PointCloud2, '/sensor/lidar_right/points')
        
        self.ts = ApproximateTimeSynchronizer(
            [self.sub_front, self.sub_left, self.sub_right],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.callback)

        self.pub_left = self.create_publisher(Marker, '/lane_left', 10)
        self.pub_right = self.create_publisher(Marker, '/lane_right', 10)

    def callback(self, msg_front, msg_left, msg_right):
        clouds = [
            ros_pointcloud2_to_xyz(msg)
            for msg in [msg_front, msg_left, msg_right]
        ]
        cloud = np.vstack(clouds)
        if cloud.shape[0] == 0:
            self.get_logger().warn("[debug] Received empty PointCloud2 message")
            return
        
        self.get_logger().info(f"[debug] Combined cloud size: {cloud.shape[0]}")
        cloud = cloud[~np.isnan(cloud).any(axis=1)]

        # Фильтрация по ROI
        mask = (
            (cloud[:, 0] > ROI_X[0]) & (cloud[:, 0] < ROI_X[1]) &
            (cloud[:, 1] > ROI_Y[0]) & (cloud[:, 1] < ROI_Y[1]) &
            (cloud[:, 2] > ROI_Z[0]) & (cloud[:, 2] < ROI_Z[1])
        )
        ground = cloud[mask]

        if len(ground) < 10:
            return
        self.get_logger().info(f"[debug] ROI-filtered points: {ground.shape[0]}")


        # Снижение количества точек
        ground = voxel_downsample(ground, voxel_size=VOXEL_SIZE)
        self.get_logger().info(f"[debug] Downsampled points: {ground.shape[0]}")

        try:
            kmeans = KMeans(n_clusters=NUM_CLUSTERS, n_init='auto').fit(ground[:, :2])
        except Exception as e:
            self.get_logger().warn(f"KMeans failed: {e}")
            return

        labels = kmeans.labels_
        max_label = labels.max()

        left_candidates = []
        right_candidates = []
        left_line = []
        right_line = []

        median_y = np.median(ground[:, 1])  # делим облако на левую/правую половины


        # Обход кластеров
        for label in range(max_label + 1):
            points = ground[labels == label]
            if len(points) < MIN_CLUSTER_POINTS or np.ptp(points[:, 0]) < MIN_CLUSTER_LENGTH:
                continue
            mean_y = np.mean(points[:, 1])
            if mean_y < median_y - LANE_OFFSET_Y:
                left_candidates.append((mean_y, np.mean(points, axis=0), label))
            elif mean_y > median_y + LANE_OFFSET_Y:
                right_candidates.append((mean_y, np.mean(points, axis=0), label))
        
        self.get_logger().info(f"[debug] Left candidates: {len(left_candidates)}")
        self.get_logger().info(f"[debug] Right candidates: {len(right_candidates)}")


        # Объединение подходящих кластеров
        if left_candidates:
            left_labels = [c[2] for c in sorted(left_candidates, key=lambda x: x[0])]
            left_line = np.vstack([ground[labels == lbl] for lbl in left_labels])
        

        if right_candidates:
            right_labels = [c[2] for c in sorted(right_candidates, key=lambda x: -x[0])]
            right_line = np.vstack([ground[labels == lbl] for lbl in right_labels])
        else:
            self.get_logger().info("[lane] No right lane candidates found")

        # Визуализация
        def publish_polyline(points, publisher, ns, color):
            smoothed = moving_average_smooth(points, window=SMOOTHING_WINDOW)
            if len(smoothed) < 2:
                return

            marker = Marker()
            marker.header.frame_id = FRAME_ID
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = ns
            marker.id = 1 if ns == "right_lane" else 0
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = LINE_WIDTH
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
            marker.points = [Point(x=p[0], y=p[1], z=p[2]) for p in smoothed]
            marker.lifetime = Duration(sec=0, nanosec=500_000_000)  # 0.5 seconds

            publisher.publish(marker)

        if len(left_line) > 1:
            publish_polyline(left_line, self.pub_left, "left_lane", COLOR_LEFT)

        if len(right_line) > 1:
            publish_polyline(right_line, self.pub_right, "right_lane", COLOR_RIGHT)


def main(args=None):
    rclpy.init(args=args)
    node = LaneExtractor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()