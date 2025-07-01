# Unitree Go2 RPLiDAR SLAM

A ROS 2 package that provides SLAM (Simultaneous Localization and Mapping) capabilities for the Unitree Go2 robot using an RPLiDAR sensor and the SLAM Toolbox.

## Overview

This package integrates:
- **RPLiDAR** for laser scanning
- **SLAM Toolbox** for mapping and localization
- **Transform broadcasting** for robot pose integration
- **RViz visualization** for real-time monitoring

## Features

- **Real-time SLAM**: Simultaneous localization and mapping using SLAM Toolbox
- **RPLiDAR Integration**: Support for RPLiDAR A1/A2/A3 series sensors
- **Robot Control**: Direct integration with Unitree Go2 movement commands
- **Visualization**: Pre-configured RViz setup for monitoring
- **Transform Management**: Automatic handling of coordinate frame transforms
- **Configurable Parameters**: Easy customization of SLAM and sensor parameters

## Prerequisites

- ROS 2 Humble
- Python 3.10+
- RPLiDAR connected via USB (typically `/dev/ttyUSB0`)

## Dependencies

The following ROS 2 packages are required:
- `rclpy` - Python client library for ROS 2
- `tf2_ros` - Transform library
- `sensor_msgs` - Sensor message definitions
- `geometry_msgs` - Geometry message definitions
- `rplidar_ros` - RPLiDAR driver
- `slam_toolbox` - SLAM implementation
- `launch` - Launch system
- `launch_ros` - ROS-specific launch functionality
- `ament_index_python` - Package resource indexing
- `unitree_api` - Custom Unitree Go2 API messages (included in this repository)

## Installation

1. Clone this repository into your ROS 2 workspace:
```bash
cd ~/ros2_ws/src
git clone https://github.com/OpenmindAGI/unitree_go2_ros2_sdk.git
```

2. Install dependencies:
```bash
cd ~/ros2_ws
rosdep install --from-paths . --ignore-src -r -y
```

3. Build the packages:
```bash
colcon build --packages-select unitree_api go2_sdk
```

4. Source the workspace:
```bash
source install/setup.bash
```

5. Set up RPLiDAR permissions:
```bash
# Option 1: Temporary permission (needs to be run each time)
sudo chmod 777 /dev/ttyUSB0

# Option 2: Add user to dialout group (permanent, requires logout/login)
sudo usermod -a -G dialout $USER

# Option 3: Create udev rule (permanent)
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", GROUP="dialout", MODE="0666"' | sudo tee /etc/udev/rules.d/99-rplidar.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Usage

### Launch SLAM System

To start the complete SLAM system:

```bash
ros2 launch go2_sdk slam_launch.py
```

### Launch with Custom Parameters

You can customize RPLiDAR and other parameters:

```bash
ros2 launch go2_sdk slam_launch.py \
    serial_port:=/dev/ttyUSB0 \
    serial_baudrate:=115200 \
    frame_id:=laser \
    scan_mode:=Sensitivity
```

### Control the Robot

Once SLAM is running, you can control the robot using:

```bash
# Using keyboard teleop (install first: sudo apt install ros-humble-teleop-twist-keyboard)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Or publish velocity commands directly
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

### Visualize in RViz

Launch RViz with the provided configuration:

```bash
rviz2 -d config/rviz.rviz
```

## Package Structure

```
unitree_go2_rplidar_slam/
├── go2_sdk/                    # Main ROS 2 package
│   ├── config/
│   │   ├── slam.yaml          # SLAM Toolbox configuration
│   │   └── rviz.rviz          # RViz visualization setup
│   ├── launch/
│   │   └── slam_launch.py     # Main launch file
│   ├── go2_sdk/               # Python package source
│   │   ├── __init__.py
│   │   ├── pose_to_tf.py      # Pose to transform broadcaster
│   │   ├── go2_command.py     # Command velocity to Go2 converter
│   │   └── go2_command_script.py
│   ├── package.xml            # Package dependencies
│   ├── setup.py               # Python package setup
│   └── resource/
├── unitree_api/               # Unitree API messages package
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── msg/                   # Custom message definitions
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
- **Package:** `go2_sdk`
- **Executable:** `pose_to_tf`
- **Function:** Converts robot pose messages to TF transforms

### 4. Command Velocity to Go2 Converter
- **Package:** `go2_sdk`
- **Executable:** `cmd_vel_to_go2`
- **Function:** Converts standard ROS cmd_vel messages to Unitree Go2 sport commands

### 5. Static Transform Publisher
- **Package:** `tf2_ros`
- **Function:** Publishes static transform from `base_link` to `laser`

## Topics

- `/scan` - Laser scan data from RPLiDAR
- `/map` - Occupancy grid map from SLAM Toolbox
- `/cmd_vel` - Velocity commands for robot movement
- `/utlidar/robot_pose` - Robot pose from Unitree Go2
- `/lf_sport_req` - Sport command requests to Go2 robot
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

### Robot Control Issues

- Check if the robot is receiving commands: `ros2 topic echo /lf_sport_req`
- Verify cmd_vel messages are being published: `ros2 topic echo /cmd_vel`
- Ensure the robot is in the correct mode for receiving movement commands

### Timestamp Issues

Somehow, the timestamp of the ros topics from Unitree Go2 is 12 seconds behind the current timestamp. Please disable your computer `automatic date & Time` and sync the timestamp using

```
sudo date -s "@unix"
```

You can find the timestamp of Unitree Go2 via

```
ros2 topic echo /utlidar/robot_pose --field header.stamp
```
