# Velodyne Simulator
URDF description and Gazebo Fortress plugins to simulate Velodyne laser scanners for ROS 2 Humble.

![rviz screenshot](img/rviz.png)

# Requirements
- ROS 2 Humble
- Gazebo Fortress (gz-sim7)
- ros_gz packages (ros_gz_sim, ros_gz_bridge)

# Installation
```bash
# Install dependencies
sudo apt install ros-humble-ros-gz ros-humble-xacro ros-humble-robot-state-publisher

# Clone and build
cd ~/ros2_ws/src
git clone <repository_url> velodyne_simulator
cd ~/ros2_ws
colcon build --packages-select velodyne_simulator velodyne_description velodyne_gazebo_plugins
source install/setup.bash
```

# Features
* URDF with colored meshes
* Gazebo Fortress GPU Lidar sensor integration
* Publishes PointCloud2 via ros_gz_bridge
* Simulated Gaussian noise
* GPU acceleration (default)
* Supported models:
    * [VLP-16](velodyne_description/urdf/VLP-16.urdf.xacro)
    * [HDL-32E](velodyne_description/urdf/HDL-32E.urdf.xacro)
    * [LiDAR-X](velodyne_description/urdf/LiDAR-X.urdf.xacro)
    * Pull requests for other models are welcome

# Parameters
* ```*origin``` URDF transform from parent link.
* ```parent``` URDF parent link name. Default ```base_link```
* ```name``` URDF model name. Also used as tf frame_id for PointCloud2 output. Default ```velodyne```
* ```topic``` ROS 2 topic name for PointCloud2 output. Default ```/velodyne_points```
* ```gz_topic``` Gazebo topic name for lidar data. Default ```lidar```
* ```hz``` Update rate in hz. Default ```10```
* ```lasers``` Number of vertical spinning lasers. Default ```VLP-16: 16, HDL-32E: 32```
* ```samples``` Number of horizontal rotating samples. Default ```VLP-16: 1875, HDL-32E: 2187```
* ```min_range``` Minimum range value in meters. Default ```0.9```
* ```max_range``` Maximum range value in meters. Default ```130.0```
* ```noise``` Gaussian noise value in meters. Default ```0.008```
* ```min_angle``` Minimum horizontal angle in radians. Default ```-3.14```
* ```max_angle``` Maximum horizontal angle in radians. Default ```3.14```
* ```gpu``` Use gpu_lidar sensor (always true for Fortress). Default ```true```

# Example Usage

## Launch Gazebo Fortress with Velodyne Robot
```bash
ros2 launch velodyne_description example.launch.py
```

## Launch with custom world
```bash
ros2 launch velodyne_description example.launch.py world:=/path/to/custom.sdf
```

## Launch without RViz
```bash
ros2 launch velodyne_description example.launch.py rviz:=false
```

# Architecture (ROS 2 + Gazebo Fortress)

```
┌─────────────────────────────────────────────────────────────┐
│                    Gazebo Fortress                          │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  Built-in GPU Lidar Sensor                          │   │
│  │  - Publishes to Gazebo topic (e.g., /lidar)         │   │
│  └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    ros_gz_bridge                            │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  Bridges gz.msgs.PointCloudPacked                   │   │
│  │  to sensor_msgs/msg/PointCloud2                     │   │
│  └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    ROS 2 Humble                             │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  Topics: /velodyne_points, /velodyne_points2        │   │
│  │  Visualize in RViz2                                 │   │
│  └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

# Migration from ROS 1 / Gazebo Classic
This package has been migrated from ROS 1 (catkin) to ROS 2 Humble and from 
Gazebo Classic to Gazebo Fortress. Key changes:

| ROS 1 / Gazebo Classic | ROS 2 / Gazebo Fortress |
|------------------------|-------------------------|
| `catkin` build system | `ament_cmake` build system |
| `.launch` XML files | `.launch.py` Python files |
| `gazebo_ros` package | `ros_gz_sim` package |
| Custom Ray/GpuRay plugin | Built-in gpu_lidar sensor |
| Direct ROS publishing | ros_gz_bridge for message bridging |
| `sensor type="gpu_ray"` | `sensor type="gpu_lidar"` |

# Known Issues
* Gazebo Fortress requires GPU for lidar simulation (cpu-only lidar not supported)
* Large point clouds may require tuning QoS settings for reliable transmission

# License
BSD-3-Clause

