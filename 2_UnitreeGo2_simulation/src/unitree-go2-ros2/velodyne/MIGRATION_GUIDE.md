# Migration Guide: Gazebo Classic → Gazebo Fortress (ROS 2 Humble)

This document details the migration of the Velodyne Simulator from ROS 1 / Gazebo Classic to ROS 2 Humble / Gazebo Fortress.

## Overview

| Aspect | Before | After |
|--------|--------|-------|
| ROS Version | ROS 1 (Noetic) | ROS 2 Humble |
| Gazebo Version | Gazebo Classic (11) | Gazebo Fortress (Ignition Gazebo 6) |
| Build System | catkin | ament_cmake |
| Package Format | format="2" | format="3" |
| Launch Files | XML (.launch) | Python (.launch.py) |
| Package Naming | N/A | ignition-gazebo6 (not gz-sim7) |

**Important Note**: ROS 2 Humble uses the older "Ignition" naming convention (`ignition-gazebo6`, `ignition-plugin1`, etc.) rather than the newer "gz" naming (`gz-sim7`). All code and dependencies must use `ignition::` namespaces and `IGNITION_` macros.

---

## Files Modified

### 1. velodyne_description/package.xml

**Changes:**
- Updated package format from 2 to 3
- Replaced `catkin` with `ament_cmake`
- Added ROS 2 dependencies: `robot_state_publisher`, `ros_gz_sim`, `ros_gz_bridge`, `rviz2`
- Added test dependencies for ament_lint
- Added `<export><build_type>ament_cmake</build_type></export>`

```xml
<!-- Before -->
<buildtool_depend>catkin</buildtool_depend>
<exec_depend>urdf</exec_depend>
<exec_depend>xacro</exec_depend>

<!-- After -->
<buildtool_depend>ament_cmake</buildtool_depend>
<depend>urdf</depend>
<depend>xacro</depend>
<depend>robot_state_publisher</depend>
<exec_depend>ros_gz_sim</exec_depend>
<exec_depend>ros_gz_bridge</exec_depend>
```

### 2. velodyne_description/CMakeLists.txt

**Changes:**
- Updated cmake_minimum_required to 3.8
- Replaced catkin macros with ament_cmake equivalents
- Updated install destinations to use `share/${PROJECT_NAME}`

```cmake
# Before
find_package(catkin REQUIRED)
catkin_package()
install(DIRECTORY ... DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# After
find_package(ament_cmake REQUIRED)
install(DIRECTORY ... DESTINATION share/${PROJECT_NAME})
ament_package()
```

### 3. velodyne_description/urdf/VLP-16.urdf.xacro

**Changes:**
- Added `gz_topic` parameter for Gazebo topic name
- Changed default `gpu` to `true` (Fortress requires GPU lidar)
- Changed sensor type from `gpu_ray`/`ray` to `gpu_lidar`
- Replaced `<ray>` element with `<lidar>` element
- Removed custom plugin reference (uses built-in sensor)
- Added `<always_on>`, `<topic>`, `<gz_frame_id>` elements

```xml
<!-- Before -->
<sensor type="gpu_ray" name="${name}-VLP16">
  <ray>
    <scan>...</scan>
    <range>...</range>
  </ray>
  <plugin name="gazebo_ros_laser_controller" filename="libgazebo_ros_velodyne_gpu_laser.so">
    <topicName>${topic}</topicName>
  </plugin>
</sensor>

<!-- After -->
<sensor type="gpu_lidar" name="${name}-VLP16">
  <always_on>true</always_on>
  <topic>${gz_topic}</topic>
  <gz_frame_id>${name}</gz_frame_id>
  <lidar>
    <scan>...</scan>
    <range>...</range>
  </lidar>
</sensor>
```

### 4. velodyne_description/urdf/HDL-32E.urdf.xacro

Same changes as VLP-16.urdf.xacro.

### 5. velodyne_description/urdf/LiDAR-X.urdf.xacro

Same changes as VLP-16.urdf.xacro.

### 6. velodyne_description/urdf/LiDAR-X-rotating.urdf.xacro

**Additional Changes:**
- Replaced ROS 1 transmission interface with ros2_control
- Added gz_ros2_control plugin for joint control

```xml
<!-- Before -->
<transmission name="simple_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="${name}_base_scan_joint">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
  </joint>
</transmission>

<!-- After -->
<ros2_control name="${name}_control" type="system">
  <hardware>
    <plugin>gz_ros2_control/GazeboSimSystem</plugin>
  </hardware>
  <joint name="${name}_base_scan_joint">
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

### 7. velodyne_description/urdf/example.urdf.xacro

**Changes:**
- Added `gz_topic` parameters to sensor macros
- Added Sensors system plugin for Gazebo Fortress

```xml
<!-- Added -->
<gazebo>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
    <render_engine>ogre2</render_engine>
  </plugin>
</gazebo>
```

### 8. velodyne_description/launch/example.launch.py

**Critical Fix for Mesh Loading:**
Added `SetEnvironmentVariable` action to set `IGN_GAZEBO_RESOURCE_PATH` so Gazebo can resolve mesh URIs like `model://velodyne_description/meshes/VLP16_base_1.dae`.

```python
from launch.actions import SetEnvironmentVariable

# Set Gazebo resource path to find meshes
set_gazebo_resource_path = SetEnvironmentVariable(
    name='IGN_GAZEBO_RESOURCE_PATH',
    value=os.path.join(pkg_velodyne_description, '..')
)

# Add to LaunchDescription first
return LaunchDescription([
    set_gazebo_resource_path,  # Must be first to take effect
    # ... other actions
])
```

**Without this fix**, you'll see errors like:
```
[Err] [SystemPaths.cc:378] Unable to find file with URI [model://velodyne_description/meshes/HDL32E_base.dae]
```

### 9. velodyne_gazebo_plugins/package.xml

**Changes:**
- Updated to format 3
- Replaced ROS 1 dependencies with ROS 2 equivalents
- **Critical**: Use Ignition naming (not gz naming) for ROS 2 Humble

```xml
<!-- Before -->
<depend>roscpp</depend>
<depend>tf</depend>
<depend>gazebo_ros</depend>

<!-- After (CORRECT for ROS 2 Humble) -->
<depend>rclcpp</depend>
<depend>tf2</depend>
<depend>tf2_ros</depend>
<depend>ignition-gazebo6</depend>
<depend>ignition-plugin1</depend>
<depend>ignition-sensors6</depend>
<depend>ignition-transport11</depend>
<depend>ignition-msgs8</depend>

<!-- INCORRECT - Don't use these with ROS 2 Humble -->
<!-- <depend>gz-sim7</depend> -->
<!-- <depend>gz-plugin2</depend> -->
```

### 10. velodyne_gazebo_plugins/CMakeLists.txt

**Changes:**
- Migrated from catkin to ament_cmake
- **Critical**: Use Ignition package names (not gz) for ROS 2 Humble
- Created new VelodyneLidarSystem target with ignition namespaces

```cmake
# Before
find_package(catkin REQUIRED COMPONENTS roscpp gazebo_ros ...)
find_package(gazebo REQUIRED)
add_library(gazebo_ros_velodyne_laser ...)

# After (CORRECT for ROS 2 Humble)
find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
add_library(VelodyneLidarSystem SHARED ...)
target_link_libraries(VelodyneLidarSystem
  ignition-gazebo6::core
  ignition-plugin1::register
  ignition-transport11::core
  ignition-msgs8::core
)

# INCORRECT - Don't use these with ROS 2 Humble
# find_package(gz-sim7 REQUIRED)
# find_package(gz-plugin2 REQUIRED)
```

### 11. velodyne_gazebo_plugins/include/.../VelodyneLidarSystem.hpp

**Changes:**
- New header file for Ignition system plugin
- **Critical**: Uses `ignition::` namespace (not `gz::`)

```cpp
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/EventManager.hh>

namespace velodyne_gazebo_plugins
{
class VelodyneLidarSystem :
  public ignition::gazebo::System,
  public ignition::gazebo::ISystemConfigure,
  public ignition::gazebo::ISystemPreUpdate,
  public ignition::gazebo::ISystemPostUpdate
{
  // ...
};
}
```

### 12. velodyne_gazebo_plugins/src/VelodyneLidarSystem.cpp

**Changes:**
- Complete rewrite as Ignition system plugin
- **Critical**: Uses `ignition::` namespace throughout
- Uses `IGNITION_ADD_PLUGIN` macro (not `GZ_ADD_PLUGIN`)

```cpp
#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/pointcloud_packed.pb.h>

// Implementation uses ignition:: namespace
ignition::transport::Node gz_node;
void OnLidarMsg(const ignition::msgs::PointCloudPacked & _msg);

// Register with IGNITION macro
IGNITION_ADD_PLUGIN(
  velodyne_gazebo_plugins::VelodyneLidarSystem,
  ignition::gazebo::System,
  velodyne_gazebo_plugins::VelodyneLidarSystem::ISystemConfigure,
  velodyne_gazebo_plugins::VelodyneLidarSystem::ISystemPreUpdate,
  velodyne_gazebo_plugins::VelodyneLidarSystem::ISystemPostUpdate)
```

### 13. velodyne_simulator/package.xml

**Changes:**
- Updated to format 3
- Replaced catkin with ament_cmake
- Removed metapackage export (not needed in ROS 2)

### 14. velodyne_simulator/CMakeLists.txt

**Changes:**
- Replaced `catkin_metapackage()` with simple ament_cmake package

### 15. README.md

**Changes:**
- Updated installation instructions for ROS 2
- Updated launch commands to use `ros2 launch`
- Added architecture diagram showing ros_gz_bridge flow
- Added migration notes section

---

## Files Created

### 1. velodyne_description/launch/example.launch.py

New Python-based ROS 2 launch file that:
- Launches Gazebo Fortress with `ros_gz_sim`
- Spawns robot using `ros_gz_sim create`
- Starts `robot_state_publisher`
- Configures `ros_gz_bridge` for lidar topics and clock
- Optionally launches RViz2

### 2. velodyne_description/world/example.sdf

New SDF 1.8 world file for Gazebo Fortress with:
- Physics configuration
- Required system plugins (Physics, UserCommands, SceneBroadcaster, Sensors)
- Sun light source
- Ground plane
- Test objects (boxes, cylinder) for lidar scanning

### 3. velodyne_description/config/bridge.yaml

ros_gz_bridge configuration file mapping:
- `/lidar` (Gazebo) → `/velodyne_points` (ROS 2)
- `/lidar2` (Gazebo) → `/velodyne_points2` (ROS 2)
- `/clock` (Gazebo) → `/clock` (ROS 2)

### 4. velodyne_description/rviz/example_ros2.rviz

Updated RViz2 configuration with:
- `rviz_default_plugins` class names (instead of `rviz`)
- ROS 2 topic QoS settings
- Updated RobotModel to use `/robot_description` topic

### 5. velodyne_gazebo_plugins/include/velodyne_gazebo_plugins/VelodyneLidarSystem.hpp

Header file for optional Gazebo Fortress system plugin:
- Inherits from `gz::sim::System`
- Implements `ISystemConfigure`, `ISystemPreUpdate`, `ISystemPostUpdate`

### 6. velodyne_gazebo_plugins/src/VelodyneLidarSystem.cpp

Implementation of optional Gazebo Fortress system plugin:
- Subscribes to Gazebo lidar topic
- Publishes ROS 2 PointCloud2 with Velodyne field layout (x, y, z, intensity, ring)
- Supports Gaussian noise injection
- Supports range and intensity filtering

**Note:** This plugin is optional. The built-in `gpu_lidar` sensor with `ros_gz_bridge` handles most use cases.

### 7. velodyne_gazebo_plugins/config/velodyne_controller_ros2.yaml

ros2_control configuration for rotating lidar joint control.

---

## Key API Changes

| Gazebo Classic | Gazebo Fortress (Ignition Gazebo 6) |
|----------------|-------------------------------------|
| `gazebo::sensors::RaySensor` | `ignition::sensors::GpuLidarSensor` |
| `gazebo::transport::Node` | `ignition::transport::Node` |
| `gazebo::msgs::*` | `ignition::msgs::*` |
| `ros::NodeHandle` | `rclcpp::Node` |
| `ros::Publisher` | `rclcpp::Publisher<T>` |
| `boost::thread` | `std::thread` |
| `boost::mutex` | `std::mutex` |
| `GAZEBO_REGISTER_SENSOR_PLUGIN` | `IGNITION_ADD_PLUGIN` (not `GZ_ADD_PLUGIN`) |
| `sensor type="gpu_ray"` | `sensor type="gpu_lidar"` |
| `<ray>` element | `<lidar>` element |
| N/A | `gz::` (newer naming, not used in Humble) |

---

## Build and Run

```bash
# Install dependencies (Note: Uses ignition-gazebo6, not gz-fortress)
sudo apt install ros-humble-ros-gz ros-humble-xacro ros-humble-robot-state-publisher ros-humble-rviz2
sudo apt install libignition-gazebo6-dev ignition-gazebo6

# Verify Ignition Gazebo is installed
dpkg -l | grep ignition-gazebo6

# Build
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select velodyne_simulator velodyne_description velodyne_gazebo_plugins
source install/setup.bash

# Launch
ros2 launch velodyne_description example.launch.py

# Launch without RViz
ros2 launch velodyne_description example.launch.py rviz:=false

# Verify topics
ros2 topic list
ros2 topic hz /velodyne_points
ros2 topic echo /velodyne_points --once
```

---

## Troubleshooting

### Mesh files not found

**Symptom:**
```
[Err] [SystemPaths.cc:378] Unable to find file with URI [model://velodyne_description/meshes/VLP16_base_1.dae]
[Err] [SystemPaths.cc:473] Could not resolve file [model://velodyne_description/meshes/VLP16_base_1.dae]
```

**Solution:**
The launch file now automatically sets `IGN_GAZEBO_RESOURCE_PATH`. If running Gazebo manually, export it:
```bash
export IGN_GAZEBO_RESOURCE_PATH=$(ros2 pkg prefix velodyne_description)/share:$IGN_GAZEBO_RESOURCE_PATH
ign gazebo path/to/world.sdf
```

### Build error: "Could not find gz-sim7"

**Symptom:**
```
CMake Error: Could not find a package configuration file provided by "gz-sim7"
```

**Solution:**
ROS 2 Humble uses Ignition naming, not gz naming. Update your package.xml and CMakeLists.txt:
```xml
<!-- WRONG -->
<depend>gz-sim7</depend>

<!-- CORRECT -->
<depend>ignition-gazebo6</depend>
```

```cmake
# WRONG
find_package(gz-sim7 REQUIRED)

# CORRECT
find_package(ignition-gazebo6 REQUIRED)
```

Install the correct packages:
```bash
sudo apt install libignition-gazebo6-dev ignition-gazebo6
```

### Gazebo doesn't start

Ensure you have Ignition Gazebo 6 (Fortress) installed:
```bash
sudo apt install ignition-fortress
# OR
sudo apt install ignition-gazebo6
```

Check installation:
```bash
ign gazebo --version
# Should show: Ignition Gazebo, version 6.x.x
```

### No point cloud data
1. Check Gazebo topics: `ign topic -l` (should see `/lidar`, `/lidar2`)
2. Check bridge is running: `ros2 node list | grep bridge`
3. Check ROS topics: `ros2 topic list` (should see `/velodyne_points`)
4. Verify sensor is enabled in URDF: `<always_on>true</always_on>`
5. Check bridge mappings in launch file

### GPU lidar not working
Gazebo Fortress requires a GPU with OpenGL support. Check:
```bash
glxinfo | grep "OpenGL renderer"
```

If using a virtual machine or headless system, you may need software rendering:
```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

### Segmentation fault in Ogre2 renderer

**Symptom:**
```
[ign gazebo-1] Segmentation fault (Address not mapped to object [0x18])
Stack trace shows crash in Ogre2LightVisual::CreateVisual()
```

**Solution:**
This is a known issue with Ogre2 renderer when creating light visuals on some systems. The world file has been updated to use scene ambient lighting instead of directional light visuals:

```xml
<!-- Instead of <light> elements, use scene ambient lighting -->
<scene>
  <ambient>0.8 0.8 0.8 1.0</ambient>
  <background>0.7 0.7 0.7 1</background>
  <shadows>false</shadows>
</scene>
```

This provides sufficient lighting for visualization without triggering the Ogre2 crash.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Gazebo Fortress                          │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  gpu_lidar Sensor (built-in)                        │   │
│  │  Publishes: gz.msgs.PointCloudPacked                │   │
│  │  Topic: /lidar, /lidar2                             │   │
│  └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    ros_gz_bridge                            │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  /lidar → /velodyne_points                          │   │
│  │  /lidar2 → /velodyne_points2                        │   │
│  │  /clock → /clock                                    │   │
│  └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    ROS 2 Humble                             │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  sensor_msgs/msg/PointCloud2                        │   │
│  │  Topics: /velodyne_points, /velodyne_points2        │   │
│  └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```
