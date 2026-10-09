import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rtabmap_launch_pkg = get_package_share_directory('rtabmap_launch')
    rtabmap_launch_path = os.path.join(rtabmap_launch_pkg, 'launch', 'rtabmap.launch.py')

    rtabmap_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch_path),
        launch_arguments={
            'use_sim_time': 'true',
            'depth': 'false',
            'visual_odometry': 'false',
            'subscribe_odom_info': 'false',
            'subscribe_scan_cloud': 'true',
            'frame_id': 'velodyne',
            'scan_cloud_topic': '/kitti/velodyne_points',
            'icp_odometry': 'false',
            'odom_topic': '/kiss/odometry',
            'approx_sync': 'true',
            'rtabmap_viz': 'true',
            'args': '-d --Grid/CellSize 0.2 --Grid/RayTracing true --Grid/3D true'
        }.items()
    )

    return LaunchDescription([
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        rtabmap_node
    ])