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

from dynamic_object_removal import clean_map_by_fusion

class MapCleaner(Node):
    def __init__(self):
        super().__init__('map_cleaner_node')
        self.scans = [] 
        self.global_pcd = o3d.geometry.PointCloud()
        
        self.odom_sub = message_filters.Subscriber(self, Odometry, '/kiss/odometry')
        self.lidar_sub = message_filters.Subscriber(self, PointCloud2, '/kitti/velodyne_points', qos_profile=qos_profile_sensor_data)
        
        self.ts = message_filters.ApproximateTimeSynchronizer([self.odom_sub, self.lidar_sub], queue_size=10, slop=0.05)
        self.ts.registerCallback(self.sync_callback)
        print("Waiting for mcap data")

    def sync_callback(self, odom_msg, lidar_msg):
        pos = odom_msg.pose.pose.position
        ori = odom_msg.pose.pose.orientation
        q = Quaternion(ori.w, ori.x, ori.y, ori.z)
        transform = np.eye(4)
        transform[:3, :3] = q.rotation_matrix
        transform[0, 3] = pos.x
        transform[1, 3] = pos.y
        transform[2, 3] = pos.z
        
        sensor_origin = transform[:3, 3]

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
        
        points_in_map_frame = np.asarray(pcd.points)
        self.scans.append((points_in_map_frame, sensor_origin))
        
        self.global_pcd += pcd
        print(f"Received frame. Current map: {len(self.global_pcd.points)} points.", end='\r')

    def save_data(self):
        print("\n\nStopped receiving data. Starting map cleaning")
        if len(self.scans) == 0:
            print("No data received!")
            return
            
        map_points = np.asarray(self.global_pcd.points)
        
        keep_mask = clean_map_by_fusion(
            map_points=map_points, 
            scans=self.scans, 
            workers=6
        )
        
        save_dir = "/workspace/results/latest"
        os.makedirs(save_dir, exist_ok=True)
        
        kept_pcd = o3d.geometry.PointCloud()
        kept_pcd.points = o3d.utility.Vector3dVector(map_points[keep_mask])
        kept_pcd.paint_uniform_color([0, 1, 0])
        o3d.io.write_point_cloud(os.path.join(save_dir, "static_map.pcd"), kept_pcd)
        
        removed_pcd = o3d.geometry.PointCloud()
        removed_pcd.points = o3d.utility.Vector3dVector(map_points[~keep_mask])
        removed_pcd.paint_uniform_color([1, 0, 0])
        o3d.io.write_point_cloud(os.path.join(save_dir, "dynamic_map.pcd"), removed_pcd)

        print(f"Completed! Original number of points: {len(map_points)}")
        print(f"Kept (Green): {np.sum(keep_mask)} points")
        print(f"Removed (Red): {np.sum(~keep_mask)} points")

def main(args=None):
    rclpy.init(args=args)
    node = MapCleaner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_data()
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass

if __name__ == '__main__':
    main()