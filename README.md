# ROS 2 Workspace

This workspace contains configuration and launch files for ROS 2 navigation and SLAM tasks.

## Packages

### `my_slam_config`

This package holds custom configurations for SLAM (Simultaneous Localization and Mapping) and Navigation 2.

#### Launch Files (`src/my_slam_config/launch/`)

- **`start_slam.launch.py`**: Launches the SLAM nodes (likely SLAM Toolbox) to generate a map from sensor data.
- **`start_nav.launch.py`**: Launches the Navigation 2 stack for autonomous navigation.
- **`start_nav_slam.launch.py`**: Launches both SLAM and Navigation for integrated mapping and navigation tasks.
- **`lidar_2d_convert.launch.py`**: Utility launch file, likely for converting pointcloud data to 2D laser scans.

#### Parameters (`src/my_slam_config/params/`)

- **`my_nav2_params.yaml`**: Configuration parameters for the Navigation 2 stack (costmaps, controllers, planners, etc.).

### Shortcut Scripts (`Shortcut/`)

These shell scripts provide convenient shortcuts for common tasks:

- **`lidar3d_bringup.sh`**: Launches the 3D LiDAR driver (`velodyne-all-nodes-VLP16-launch.py`).
- **`mapping.sh`**: Helper script to launch SLAM for mapping (`start_slam.launch.py`).
- **`navigation.sh`**: Helper script to launch Navigation 2 (`start_nav.launch.py`).
- **`slam_nav.sh`**: Helper script to launch combined SLAM and Navigation (`start_nav_slam.launch.py`).
- **`save_map.sh`**: Saves the current map using Nav2's `map_saver_cli`. Auto-increments map numbering (`map_001`, `map_002`, etc.) and updates a `latest_map` symlink.
- **`save_slamtb_map.sh`**: Saves the SLAM Toolbox serialized map (posegraph and data). Also auto-increments map numbering and updates symlinks.
- **`reset_ros.sh`**: Kills all ROS 2 and CycloneDDS processes and restarts the ROS 2 daemon. Useful for a clean reset.

## Setup & Usage

1. **Build the workspace:**
   ```bash
   colcon build
   ```

2. **Source the setup script:**
   ```bash
   source install/setup.bash
   ```

3. **Run a launch file:**
   ```bash
   ros2 launch my_slam_config <launch_file_name>
   ```
