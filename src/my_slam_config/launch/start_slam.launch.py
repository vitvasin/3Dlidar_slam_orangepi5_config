import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. หาตำแหน่งไฟล์ Config ของเรา
    pkg_my_config = get_package_share_directory('my_slam_config')
    slam_config_file = os.path.join(pkg_my_config, 'config', 'my_slam_params.yaml') 
    print(slam_config_file)

    # 2. หาตำแหน่งไฟล์ Launch หลักของ SLAM Toolbox
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    slam_launch_file = os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')

    return LaunchDescription([
        # --- ส่วนที่ 1: TF เชื่อม Lidar เข้ากับหุ่น (base_link -> velodyne) ---
        # แก้เลข 0.2 เป็นความสูงจริงของ Lidar คุณ
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0.15', '--y', '0', '--z', '0.76', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'center_footprint', '--child-frame-id', 'velodyne'],
            output='screen'
        ),

        # --- ส่วนที่ 2: SLAM Toolbox (พร้อมโหลด Config ของเรา) ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_file),
            launch_arguments={'slam_params_file': slam_config_file}.items()
        )
    ])
