import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_kiss_icp = get_package_share_directory('kiss_icp')

    topic = LaunchConfiguration('topic', default='/kitti/velodyne_points')
    base_frame = LaunchConfiguration('base_frame', default='velodyne')
    publish_odom_tf = LaunchConfiguration('publish_odom_tf', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    kiss_icp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_kiss_icp, 'launch', 'odometry.launch.py')
        ),
        launch_arguments={
            'topic': topic,
            'base_frame': base_frame,
            'publish_odom_tf': publish_odom_tf,
            'visualize': 'true',
            # 'use_sim_time': use_sim_time,
        }.items()
    )

    return LaunchDescription([
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        DeclareLaunchArgument('topic', default_value='/kitti/velodyne_points'),
        DeclareLaunchArgument('base_frame', default_value='velodyne'),
        DeclareLaunchArgument('publish_odom_tf', default_value='true'),
        # DeclareLaunchArgument('use_sim_time', default_value='true'),
        kiss_icp_launch
    ])