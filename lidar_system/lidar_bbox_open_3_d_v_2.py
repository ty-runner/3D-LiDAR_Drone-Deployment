#!/usr/bin/env python3
"""
Robust LiDAR 3D bounding box node (v2)
- ROI filtering (range + height)
- Voxel downsampling
- Statistical outlier removal
- Optional plane removal (RANSAC)
- DBSCAN clustering (per-frame by default)
- Oriented bounding boxes (OBB)
- Publishes MarkerArray + PoseArray of centroids

Designed for servo-swept 2D LiDAR -> 3D clouds.
"""

import rclpy
from rclpy.node import Node

import numpy as np
import open3d as o3d

from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseArray
from builtin_interfaces.msg import Duration

import sensor_msgs_py.point_cloud2 as pc2
from tf_transformations import quaternion_from_matrix


class LidarBBoxOpen3DV2(Node):
    def __init__(self):
        super().__init__('lidar_bbox_open3d_v2')

        # ---------------- Parameters ----------------
        self.declare_parameter('pointcloud_topic', '/lidar_points')
        self.declare_parameter('frame_id_override', '')

        self.declare_parameter('range_min', 0.3)
        self.declare_parameter('range_max', 30.0)
        self.declare_parameter('z_min', -5.0)
        self.declare_parameter('z_max', 5.0)

        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('cluster_eps', 0.20)
        self.declare_parameter('min_cluster_points', 25)

        self.declare_parameter('outlier_nb_neighbors', 20)
        self.declare_parameter('outlier_std_ratio', 2.0)

        self.declare_parameter('enable_plane_removal', True)
        self.declare_parameter('plane_distance_threshold', 0.08)
        self.declare_parameter('plane_ransac_n', 3)
        self.declare_parameter('plane_num_iterations', 100)

        # ---------------- Load params ----------------
        self.pc_topic = self.get_parameter('pointcloud_topic').value
        self.frame_override = self.get_parameter('frame_id_override').value

        self.r_min = self.get_parameter('range_min').value
        self.r_max = self.get_parameter('range_max').value
        self.z_min = self.get_parameter('z_min').value
        self.z_max = self.get_parameter('z_max').value

        self.voxel_size = self.get_parameter('voxel_size').value
        self.cluster_eps = self.get_parameter('cluster_eps').value
        self.min_pts = self.get_parameter('min_cluster_points').value

        self.nb_neighbors = self.get_parameter('outlier_nb_neighbors').value
        self.std_ratio = self.get_parameter('outlier_std_ratio').value

        self.enable_plane = self.get_parameter('enable_plane_removal').value
        self.plane_dist = self.get_parameter('plane_distance_threshold').value
        self.plane_ransac_n = self.get_parameter('plane_ransac_n').value
        self.plane_iters = self.get_parameter('plane_num_iterations').value

        # ---------------- ROS IO ----------------
        self.sub = self.create_subscription(
            PointCloud2,
            self.pc_topic,
            self.pc_callback,
            10
        )

        self.marker_pub = self.create_publisher(MarkerArray, 'lidar_bboxes_obb', 10)
        self.centroid_pub = self.create_publisher(PoseArray, 'lidar_detections_centroids', 10)

        self.get_logger().info('lidar_bbox_open3d_v2 node started')

    # ------------------------------------------------
    def pc_callback(self, msg: PointCloud2):
        pts = np.array([
            [p[0], p[1], p[2]]
            for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        ], dtype=np.float32)

        if pts.shape[0] < self.min_pts:
            return

        # -------- ROI filtering --------
        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
        r = np.sqrt(x*x + y*y)
        mask = (
            (r >= self.r_min) & (r <= self.r_max) &
            (z >= self.z_min) & (z <= self.z_max)
        )
        pts = pts[mask]
        if pts.shape[0] < self.min_pts:
            return

        # -------- Open3D processing --------
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(pts.astype(np.float64))

        # Downsample
        pc = pc.voxel_down_sample(self.voxel_size)
        if len(pc.points) < self.min_pts:
            return

        # Outlier removal
        pc, _ = pc.remove_statistical_outlier(
            nb_neighbors=self.nb_neighbors,
            std_ratio=self.std_ratio
        )
        if len(pc.points) < self.min_pts:
            return

        # Optional plane removal (peels off dominant plane)
        if self.enable_plane:
            try:
                plane_model, inliers = pc.segment_plane(
                    distance_threshold=self.plane_dist,
                    ransac_n=self.plane_ransac_n,
                    num_iterations=self.plane_iters
                )
                pc = pc.select_by_index(inliers, invert=True)
            except Exception:
                pass

        pts_clean = np.asarray(pc.points)
        if pts_clean.shape[0] < self.min_pts:
            return

        # -------- Clustering --------
        labels = np.array(
            pc.cluster_dbscan(
                eps=self.cluster_eps,
                min_points=self.min_pts,
                print_progress=False
            )
        )

        if labels.max() < 0:
            self.marker_pub.publish(MarkerArray())
            return

        # -------- Publish results --------
        marker_array = MarkerArray()
        pose_array = PoseArray()

        frame_id = self.frame_override if self.frame_override else msg.header.frame_id
        pose_array.header.frame_id = frame_id
        pose_array.header.stamp = msg.header.stamp

        now = self.get_clock().now().to_msg()

        for cid in range(labels.max() + 1):
            cluster_pts = pts_clean[labels == cid]
            if cluster_pts.shape[0] < self.min_pts:
                continue

            pc_part = o3d.geometry.PointCloud()
            pc_part.points = o3d.utility.Vector3dVector(cluster_pts.astype(np.float64))

            obb = pc_part.get_oriented_bounding_box()
            center = obb.center
            extent = obb.extent
            R = obb.R

            # Rotation matrix -> quaternion
            T = np.eye(4)
            T[:3, :3] = R
            qx, qy, qz, qw = quaternion_from_matrix(T)

            # ---- Marker ----
            mk = Marker()
            mk.header.frame_id = frame_id
            mk.header.stamp = now
            mk.ns = 'lidar_bbox_obb'
            mk.id = int(cid)
            mk.type = Marker.CUBE
            mk.action = Marker.ADD
            mk.lifetime = Duration(sec=0)

            mk.pose.position.x = float(center[0])
            mk.pose.position.y = float(center[1])
            mk.pose.position.z = float(center[2])
            mk.pose.orientation.x = float(qx)
            mk.pose.orientation.y = float(qy)
            mk.pose.orientation.z = float(qz)
            mk.pose.orientation.w = float(qw)

            mk.scale.x = float(extent[0])
            mk.scale.y = float(extent[1])
            mk.scale.z = float(extent[2])

            mk.color.r = 0.1
            mk.color.g = 0.8
            mk.color.b = 0.2
            mk.color.a = 0.6

            marker_array.markers.append(mk)

            # ---- Centroid pose ----
            pose = Pose()
            pose.position.x = float(center[0])
            pose.position.y = float(center[1])
            pose.position.z = float(center[2])
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

        self.marker_pub.publish(marker_array)
        self.centroid_pub.publish(pose_array)


# ----------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = LidarBBoxOpen3DV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
