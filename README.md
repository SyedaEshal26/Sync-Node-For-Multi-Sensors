# ROS 2 Sensor Synchronization Node

A ROS 2 C++ node for synchronizing data from multiple sensors, including camera, LiDAR, IMU, depth, odometry, and GNSS. The node provides synchronized sensor data for reliable multi-sensor perception and fusion.

## Features

* Multi-sensor data synchronization
* Camera and LiDAR synchronization
* IMU, depth, odometry, and GNSS data handling
* ROS 2 C++ implementation
* Designed for perception and sensor fusion

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select sync_node_pkg
source install/setup.bash
```

## Run

```bash
ros2 run sync_node_pkg sync_node
```

## Requirements

* ROS 2
* C++
* `rclcpp`
* `sensor_msgs`
* `nav_msgs`
* `message_filters`

## Purpose

The node acts as a synchronization layer between heterogeneous sensors, providing consistent sensor data for downstream perception, localization, and fusion tasks.
