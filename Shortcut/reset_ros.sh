#ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py
sudo pkill -9 -f ros
sudo pkill -9 -f cyclonedds
ros2 daemon start
