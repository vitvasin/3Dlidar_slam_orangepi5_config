import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([ 
        # ✅ เหลือแค่อันนี้: บอกว่า Lidar ติดอยู่ตรงไหนของหุ่น (base_link -> velodyne)
        # ปรับค่า x, y, z ตามจริง (เช่น สูงจากพื้นรถ 0.2 เมตร)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0.2', '0', '0', '0', 'base_link', 'velodyne']
        )
    ])
