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

from dufomap import dufomap

class DufomapNode(Node):
    def __init__(self):
        super().__init__('dufomap_cleaner_node')
        self.scans_local = []
        self.poses = []
        
        self.odom_sub = message_filters.Subscriber(self, Odometry, '/kiss/odometry')
        self.lidar_sub = message_filters.Subscriber(self, PointCloud2, '/kitti/velodyne_points', qos_profile=qos_profile_sensor_data)
        
        self.ts = message_filters.ApproximateTimeSynchronizer([self.odom_sub, self.lidar_sub], queue_size=10, slop=0.05)
        self.ts.registerCallback(self.sync_callback)
        print("Waiting for mcap data to run DUFOMap")

    def sync_callback(self, odom_msg, lidar_msg):
        pos = odom_msg.pose.pose.position
        ori = odom_msg.pose.pose.orientation
        q = Quaternion(ori.w, ori.x, ori.y, ori.z)
        transform = np.eye(4)
        transform[:3, :3] = q.rotation_matrix
        transform[0, 3] = pos.x
        transform[1, 3] = pos.y
        transform[2, 3] = pos.z
        
        cloud_gen = pc2.read_points(lidar_msg, field_names=("x", "y", "z"), skip_nans=True)
        points = np.array(list(cloud_gen))
        if points.shape[0] == 0: return
        
        if points.dtype.names is not None:
            points = np.column_stack((points['x'], points['y'], points['z']))

        points = np.ascontiguousarray(points, dtype=np.float64)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd = pcd.voxel_down_sample(voxel_size=0.2)
        
        local_points = np.ascontiguousarray(pcd.points, dtype=np.float64)
        
        self.scans_local.append(local_points)
        self.poses.append(transform)
        print(f"Received frame. Buffer size: {len(self.scans_local)} frames.", end='\r')

    def save_data(self):
        print("\n\nData stream stopped. Initializing DUFOMap")
        if len(self.scans_local) == 0:
            print("No data received!")
            return
            
        mydufo = dufomap(0.1, 0.2, 1)
        
        print("Ray-casting map updates")
        for pts, pose in zip(self.scans_local, self.poses):
            mydufo.run(pts, pose, cloud_transform=True)
            
        print("Dynamic Object Segmentation")
        static_points_list = []
        dynamic_points_list = []
        
        for pts, pose in zip(self.scans_local, self.poses):
            labels = mydufo.segment(pts, pose, cloud_transform=True)
            
            static_mask = (labels == 0)
            dynamic_mask = (labels == 1)
            
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pts)
            pcd.transform(pose)
            global_pts = np.asarray(pcd.points)
            
            static_points_list.append(global_pts[static_mask])
            dynamic_points_list.append(global_pts[dynamic_mask])
            
        all_static = np.vstack(static_points_list) if static_points_list else np.empty((0,3))
        all_dynamic = np.vstack(dynamic_points_list) if dynamic_points_list else np.empty((0,3))
        
        save_dir = "/workspace/results/latest"
        os.makedirs(save_dir, exist_ok=True)
        os.chmod(save_dir, 0o777)
        
        kept_pcd = o3d.geometry.PointCloud()
        kept_pcd.points = o3d.utility.Vector3dVector(all_static)
        kept_pcd.paint_uniform_color([0, 1, 0])
        static_file = os.path.join(save_dir, "static_map_dufo.pcd")
        o3d.io.write_point_cloud(static_file, kept_pcd)
        os.chmod(static_file, 0o666)
        
        removed_pcd = o3d.geometry.PointCloud()
        removed_pcd.points = o3d.utility.Vector3dVector(all_dynamic)
        removed_pcd.paint_uniform_color([1, 0, 0])
        dynamic_file = os.path.join(save_dir, "dynamic_map_dufo.pcd")
        o3d.io.write_point_cloud(dynamic_file, removed_pcd)
        os.chmod(dynamic_file, 0o666)

        print("DUFOMap Processing Completed!")
        print(f"Kept (Static - Green): {len(all_static)} points")
        print(f"Removed (Dynamic - Red): {len(all_dynamic)} points")

def main(args=None):
    rclpy.init(args=args)
    node = DufomapNode()
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