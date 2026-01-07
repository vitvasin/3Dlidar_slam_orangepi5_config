import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap

def generate_launch_description():
    # 1. Paths
    pkg_my_config = get_package_share_directory('my_slam_config')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    map_file = '/home/smr/maps/latest_map' 

    # 2. Config Files
    # SLAM Config
    slam_config_file = os.path.join(pkg_my_config, 'config', 'my_slam_localization.yaml')
    # Nav2 Params
    nav2_params_file = os.path.join(pkg_my_config, 'params', 'my_nav2_params.yaml')

    # 3. Launch Files
    # SLAM Toolbox Launch
    slam_launch_file = os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
    # Nav2 Navigation Launch (No AMCL/MapServer)
    nav2_launch_file = os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')

    return LaunchDescription([
        # --- Section 1: Static TF (Lidar) ---
        # Adjust z=0.46 based on previous files
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0.15', '--y', '0', '--z', '0.76', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'center_footprint', '--child-frame-id', 'velodyne'],
            output='screen'
        ),

        # --- Group to apply remapping to both SLAM and Nav2 ---
        GroupAction(
            actions=[
                # Remap /scan to /scan_filtered for all nodes in this group
                # SetRemap(src='/scan', dst='/scan_filtered'),

                # --- Section 2: SLAM Toolbox ---
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(slam_launch_file),
                    launch_arguments={'slam_params_file': slam_config_file}.items()
                ),

                # --- Section 3: Navigation2 ---
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav2_launch_file),
                    launch_arguments={
                        # 'map': map_file,
                        'params_file': nav2_params_file,
                        'use_sim_time': 'False',
                        'autostart': 'True',
                    }.items(),
                ),
            ]
        )
    ])
