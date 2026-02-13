# Example 2: Unitree Go2 Simulation (Ignition Gazebo)

[Usage and Installation Video](https://youtu.be/6EQpNAicpiI)

This example demonstrates how to control a **Unitree Go2 Quadruped Robot** inside a physics simulation using **Ignition Gazebo (Fortress)** and the **ROS-MCP Server**.

Using natural language and the `ROS-MCP server`, you can control the robot's gait, velocity, and pose, as well as inspect its sensors (Lidar, IMU).

## 📋 Tested On

This example has been tested and verified on:

*   **OS:** Ubuntu 22.04 LTS
*   **ROS Distro:** ROS 2 Humble
*   **Simulator:** Ignition Gazebo Fortress

## 🛠️ Prerequisites

Before running this example, ensure you have the necessary ROS 2 and Gazebo packages installed on your system:

```bash
sudo apt update
sudo apt install ros-humble-ros-gz          # Bridge between ROS 2 and Ignition
sudo apt install ros-humble-rosapi          # Required for introspection (listing topics)
sudo apt install ros-humble-rosbridge-server # WebSocket connection for MCP
```

## 📦 Installation & Setup

### 1.0 Clone the repository:

Navigate to the `src` directory of this workspace (or your own ROS 2 workspace) and clone the necessary packages.

```bash
mkdir src 
cd src
git clone https://github.com/rahgirrafi/unitree-go2-ros2.git
cd unitree-go2-ros2

# Initialize and update the velodyne submodule
git submodule update --init --recursive
```

> **Note:** The velodyne lidar package is included as a git submodule. If you've already cloned the repository without the `--recursive` flag, run `git submodule update --init --recursive` to fetch the submodule.

### 1.1 Install dependencies using rosdep:

```bash
# Install rosdep if not already installed
sudo apt install -y python3-rosdep

# Initialize rosdep (only needed once)
sudo rosdep init  # Skip if already initialized
rosdep update

# Install all dependencies
cd 2_UnitreeGo2_simulation
rosdep install --from-paths src --ignore-src -r -y
```

### 1.2 Copy the Custom Launch File

We have provided a custom launch file `gazebo_rosbridge.launch.py` that starts the simulation along with the ROS-MCP server connection (rosbridge). You need to copy this file into the package's launch directory.

**From the root of `2_UnitreeGo2_simulation`:**
```bash
cp gazebo_rosbridge.launch.py src/unitree-go2-ros2/robots/configs/go2_config/launch/
```

### 1.3 Build your workspace:
```bash
cd 2_UnitreeGo2_simulation
colcon build --packages-select go2_config
. 2_UnitreeGo2_simulation/install/setup.bash
```

## 🚀 How to Run

### 1. Launch the Simulation (Basic World)

This command starts Ignition Gazebo with the Unitree Go2 robot, the CHAMP controller, and the rosbridge server on port 9090.

```bash
ros2 launch go2_config gazebo_rosbridge.launch.py
```

### 2. Launch the Simulation (Outdoor World)

To run the robot in a more complex "construction site" environment:

```bash
ros2 launch go2_config gazebo_rosbridge.launch.py world:=$(ros2 pkg prefix champ_gazebo)/share/champ_gazebo/worlds/outdoor.world
```

### 3. Start the MCP Client

Once the simulation is running, you can connect your AI agent.

*   **Option A: Robot MCP Client (CLI)**
    ```bash
    # In the robot-mcp-client directory
    uv run clients/baseclient.py
    ```

*   **Option B: Desktop Clients**
    Configure **Claude Desktop** or **Cursor** to use the MCP server. Ensure they are configured to connect to the ROS-MCP server (typically running securely or locally).

## 🤖 Sample Prompts

Once connected, try these commands:

> "Walk forward at 0.5 m/s"

> "Turn left"

> "Stop and stand still"

> "What topics are available?"

> "Read the IMU data from /imu/data"
