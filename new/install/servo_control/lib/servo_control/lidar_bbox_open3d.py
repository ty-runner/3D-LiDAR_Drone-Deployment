#!/usr/bin/env python3
# lidar_bbox_open3d.py – ROS 2 Humble
#
# • subscribes  /lidar_points  (PointCloud2, in base_link)
# • publishes   /lidar_bboxes  (MarkerArray -- view in RViz “MarkerArray” display)

import rclpy, numpy as np, open3d as o3d
from rclpy.node             import Node
from sensor_msgs.msg        import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose, PoseArray
from collections            import deque

class BBoxO3D(Node):
    def __init__(self):
        super().__init__("bbox_open3d")

        self.sub  = self.create_subscription(PointCloud2,
                                             "/lidar_points",
                                             self.on_cloud,   10)
        self.marker_pub = self.create_publisher(MarkerArray, 'lidar_bboxes', 10)
        self.centroid_pub = self.create_publisher(PoseArray, 'lidar_detections_centroids', 10)

        # --- rolling buffer (N most-recent frames) ------------
        self.buffer      = deque(maxlen=12)#30   # 5 × @10 Hz ≈ 0.5 s
        self.frame_id    = "base_link"
        self.cluster_eps = 0.24 #.15             # metres
        self.min_pts     = 18 #

    # -------- util: fast xyz extraction -----------------------
    '''@staticmethod
    def pc2_to_xyz(msg: PointCloud2) -> np.ndarray:
        step  = msg.point_step                # bytes per point (16)
        xyz_floats = np.frombuffer(msg.data,  dtype=np.float32)
        xyz_floats = xyz_floats.reshape(-1, step//4)[:, :3]
        return xyz_floats                     # (N,3) float32
    '''
    @staticmethod
    def pc2_to_xyz(msg: PointCloud2) -> np.ndarray:
        # read_points will look at msg.fields and msg.point_step for you
        points = pc2.read_points(msg,
                                 field_names=("x", "y", "z"),
                                 skip_nans=True)
        pts = np.array(list(points), dtype=np.float32)
       # print(pts)
        if pts.size == 0:
            return np.empty((0, 3), dtype=np.float32)
        return pts.reshape(-1, 3)

    # -------- main callback -----------------------------------
    def on_cloud(self, msg: PointCloud2) -> None:
        xyz = self.pc2_to_xyz(msg)
        #if xyz.shape[0] < 30:
         #   return                            # ignore nearly-empty slices

        self.buffer.append(xyz)               # add newest slice
        pts = np.vstack(self.buffer)          # stacked cloud (≈0.5 s)

        pc          = o3d.geometry.PointCloud()
        pc.points   = o3d.utility.Vector3dVector(pts.astype(np.float64))

        #pc = pc.voxel_down_samples(0.06) #added this line
                

        labels      = np.array(
            pc.cluster_dbscan(eps=self.cluster_eps,
                              min_points=self.min_pts,
                              print_progress=False)
        )
        n_clusters  = labels.max() + 1
        marray      = MarkerArray()
        pose_array = PoseArray()
        frame_id = msg.header.frame_id
        pose_array.header.frame_id = frame_id
        pose_array.header.stamp = msg.header.stamp
        stamp       = self.get_clock().now().to_msg()

        for cid in range(n_clusters):
            part = pts[labels == cid]
            if part.size == 0:
                continue

            min_xyz, max_xyz = part.min(axis=0), part.max(axis=0)
            center           = (min_xyz + max_xyz) / 2.0
            size             =  (max_xyz - min_xyz)

            mk = Marker()
            mk.header.frame_id = self.frame_id
            mk.header.stamp    = stamp
            mk.ns, mk.id       = "bbox_o3d", int(cid)
            mk.type            = Marker.CUBE
            mk.action          = Marker.ADD
            
            mk.lifetime.sec = 1
            mk.lifetime.nanosec = int(2e8)
            mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = \
                float(center[0]), float(center[1]), float(center[2])
            mk.pose.orientation.w = 1.0

            mk.scale.x, mk.scale.y, mk.scale.z = \
               float(size[0]),  float(size[1]),  float(size[2])
            mag = mk.scale.x**2 + mk.scale.y**2 + mk.scale.z**2
            pos_mag = mk.pose.position.x**2 + mk.pose.position.y**2 + mk.pose.position.z**2
            #print(f"Magnitude = {mag}")
            #print(f"Size = {size}")
            # ---- simple “human-height” heuristic -------------
            #if 1.0 < size[1] < 2.2 and mag < 3:           # ~1.4–2.2 m tall
            if 0.2 < size[2] < 3.0 and pos_mag > 1.0:
                mk.color.r, mk.color.g, mk.color.b, mk.color.a = 1.0, 0.0, 0.0,0.6   # red
            #mk.color.a = 0.35

            pose = Pose()
            pose.position.x = float(center[0])
            pose.position.y = float(center[1])
            pose.position.z = float(center[2])
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

            marray.markers.append(mk)
        #how do we add orientation??
        self.marker_pub.publish(marray)
        self.centroid_pub.publish(pose_array)

# --------------------------- main ----------------------------------------
def main():
    rclpy.init()
    node = BBoxO3D()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
