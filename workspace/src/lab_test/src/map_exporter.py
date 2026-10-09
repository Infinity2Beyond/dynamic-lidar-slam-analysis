#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import open3d as o3d
from rclpy.qos import qos_profile_sensor_data
from pyquaternion import Quaternion

class MapExporter(Node):
    def __init__(self):
        super().__init__('map_exporter')
        self.poses_list = []
        self.global_pcd = o3d.geometry.PointCloud()
        
        self.odom_sub = message_filters.Subscriber(self, Odometry, '/kiss/odometry')
        self.lidar_sub = message_filters.Subscriber(self, PointCloud2, '/kitti/velodyne_points', qos_profile=qos_profile_sensor_data)
        
        self.ts = message_filters.ApproximateTimeSynchronizer([self.odom_sub, self.lidar_sub], queue_size=10, slop=0.05)
        self.ts.registerCallback(self.sync_callback)
        
        self.get_logger().info("Map Exporter waiting for data")

    def sync_callback(self, odom_msg, lidar_msg):
        pos = odom_msg.pose.pose.position
        ori = odom_msg.pose.pose.orientation
        
        q = Quaternion(ori.w, ori.x, ori.y, ori.z)
        transform = np.eye(4)
        transform[:3, :3] = q.rotation_matrix
        transform[0, 3] = pos.x
        transform[1, 3] = pos.y
        transform[2, 3] = pos.z
        
        row = transform[:3, :].flatten().tolist()
        self.poses_list.append(row)

        cloud_gen = pc2.read_points(lidar_msg, field_names=("x", "y", "z"), skip_nans=True)
        points = np.array(list(cloud_gen))
        if points.shape[0] == 0: return
        
        if points.dtype.names is not None:
            points = np.column_stack((points['x'], points['y'], points['z']))
        points = np.ascontiguousarray(points, dtype=np.float64)
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.transform(transform)
        
        pcd = pcd.voxel_down_sample(voxel_size=0.2)
        
        self.global_pcd += pcd

    def save_data(self):
        save_dir = "/workspace/results/latest"
        os.makedirs(save_dir, exist_ok=True)
        
        o3d.io.write_point_cloud(os.path.join(save_dir, "kiss_icp_map.pcd"), self.global_pcd)
        
        with open(os.path.join(save_dir, "kiss_icp_poses.txt"), "w") as f:
            for row in self.poses_list:
                f.write(" ".join(map(str, row)) + "\n")

def main(args=None):
    rclpy.init(args=args)
    node = MapExporter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_data() # Kích hoạt lưu file khi bấm Ctrl+C
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()