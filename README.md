# Unitree Go2 RPLiDAR SLAM

A ROS 2 package that provides SLAM (Simultaneous Localization and Mapping) capabilities for the Unitree Go2 robot using an RPLiDAR sensor and the SLAM Toolbox.

## Overview

This package integrates:
- **RPLiDAR** for laser scanning
- **SLAM Toolbox** for mapping and localization
- **Transform broadcasting** for robot pose integration
- **RViz visualization** for real-time monitoring

## Prerequisites

- ROS 2 Humble
- Python 3.10+
- RPLiDAR connected via USB (typically `/dev/ttyUSB0`)

## Dependencies

The following ROS 2 packages are required:
- `rclpy`
- `tf2_ros`
- `sensor_msgs`
- `geometry_msgs`
- `rplidar_ros`
- `slam_toolbox`
- `launch`
- `launch_ros`
- `ament_index_python`

## Installation

1. Clone this repository into your ROS 2 workspace:
```bash
cd ~/ros2_ws/src
git clone https://github.com/OpenmindAGI/unitree_go2_rplidar_slam.git unitree_go2_rplidar_slam
```

2. Install dependencies:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:
```bash
colcon build --packages-select unitree_go2_rplidar_slam
```

4. Source the workspace:
```bash
source install/setup.bash
```

5. Create udev rules for rplidar:
```bash
sudo chmod 777 /dev/ttyUSB0
```

## Usage

### Launch SLAM System

To start the complete SLAM system:

```bash
ros2 launch unitree_go2_rplidar_slam slam_launch.py
```

### Launch with Custom Parameters

You can customize the RPLiDAR configuration:

```bash
ros2 launch unitree_go2_rplidar_slam slam_launch.py \
    serial_port:=/dev/ttyUSB0 \
    serial_baudrate:=115200 \
    frame_id:=laser \
    scan_mode:=Sensitivity
```

### Visualize in RViz

Launch RViz with the provided configuration:

```bash
rviz2 -d config/rviz.rviz
```

## Package Structure

```
unitree_go2_rplidar_slam/
├── config/
│   ├── slam.yaml          # SLAM Toolbox configuration
│   └── rviz.rviz          # RViz visualization setup
├── launch/
│   └── slam_launch.py     # Main launch file
├── src/
│   ├── __init__.py
│   └── pose_to_tf.py      # Pose to transform broadcaster
├── test/                  # Unit tests
├── package.xml            # Package dependencies
├── setup.py               # Python package setup
└── README.md
```

## Configuration

### SLAM Parameters

Key SLAM parameters can be adjusted in [`config/slam.yaml`](config/slam.yaml):

- **Frame Configuration:**
  - `base_frame`: `base_link`
  - `odom_frame`: `odom`
  - `map_frame`: `map`

- **Laser Parameters:**
  - `minimum_laser_range`: `0.1m`
  - `maximum_laser_range`: `12.0m`

- **Performance Tuning:**
  - `scan_buffer_size`: `20`
  - `tf_buffer_duration`: `30.0s`

### RPLiDAR Configuration

Default RPLiDAR settings:
- **Serial Port:** `/dev/ttyUSB0`
- **Baudrate:** `115200`
- **Frame ID:** `laser`
- **Scan Mode:** `Sensitivity`

## Nodes

### 1. RPLiDAR Node
- **Package:** `rplidar_ros`
- **Executable:** `rplidar_node`
- **Function:** Publishes laser scan data from RPLiDAR

### 2. SLAM Toolbox Node
- **Package:** `slam_toolbox`
- **Executable:** `sync_slam_toolbox_node`
- **Function:** Performs SLAM using laser scan data

### 3. Pose to TF Broadcaster
- **Package:** `unitree_go2_rplidar_slam`
- **Executable:** `pose_to_tf`
- **Function:** Converts robot pose messages to TF transforms

### 4. Static Transform Publisher
- **Package:** `tf2_ros`
- **Function:** Publishes static transform from `base_link` to `laser`

## Topics

- `/scan` - Laser scan data from RPLiDAR
- `/map` - Occupancy grid map
- `/utlidar/robot_pose` - Robot pose from external source
- `/tf` and `/tf_static` - Transform data

## Frames

- `map` - Global reference frame
- `odom` - Odometry frame
- `base_link` - Robot base frame
- `laser` - LiDAR sensor frame

## Troubleshooting

### RPLiDAR Connection Issues

1. Check USB connection:
```bash
ls -la /dev/ttyUSB*
```

2. Verify permissions:
```bash
sudo chmod 777 /dev/ttyUSB0
```

3. Add user to dialout group:
```bash
sudo usermod -a -G dialout $USER
```

### SLAM Performance Issues

- Reduce `scan_buffer_size` if experiencing dropped messages
- Adjust `correlation_search_space_dimension` for better loop closure
- Modify `loop_search_maximum_distance` based on environment size

### Transform Issues

- Ensure all required transforms are being published
- Check TF tree with: `ros2 run tf2_tools view_frames`
- Verify timing with: `ros2 topic echo /tf`