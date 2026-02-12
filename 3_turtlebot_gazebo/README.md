# TurtleBot3 Simulation in Gazebo

This demo provides a simulation environment for the TurtleBot3 robot using Gazebo and ROS 2. It is designed to be used with the **ROS-MCP Server** to enable LLM-controlled robotics interactions.

## 1. Prerequisites

Ensure you have the following installed:
- **ROS 2** (Humble or newer recommended)
- **Gazebo** (Gazebo Classic 11)
- **ROS-MCP Server** (installed in your workspace)
- **Python 3.10+**

## 2. Installation

Navigate to the root of your workspace (e.g., `~/ros2_ws`) and build the package:

```bash
# Clone the repository if you haven't already
# git clone <repo_url> src/demos-ros-mcp-server

# Build the specific package
colcon build --packages-select turtlebot3_gazebo turtlebot3_simulations turtlebot3_msgs DynamixelSDK hls_lfcd_lds_driver

# Source the setup script
source install/setup.bash
```

## 3. Running the Simulation

You can launch various worlds and robot models.

### Select Robot Model

Set the `TURTLEBOT3_MODEL` environment variable to one of the following: `burger`, `waffle`, `waffle_pi`.

```bash
export TURTLEBOT3_MODEL=burger
```

### Launch Commands

Here are the commands to launch different simulation worlds:

**1. Default World (TurtleBot3 World)**
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**2. Empty World**
```bash
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

**3. House Environment**
```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

**4. Machine Learning Worlds (DQN Stages)**
```bash
ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage1.launch.py
ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage2.launch.py
ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage3.launch.py
ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage4.launch.py
```

**5. AutoRace Track**
```bash
ros2 launch turtlebot3_gazebo turtlebot3_autorace_2020.launch.py
```

## 4. ROS-MCP Server Integration

The **ROS-MCP Server** allows you to connect an LLM (Large Language Model) to this simulation to inspect topics, call services, and control the robot.

### Running the Server

In a new terminal, run the ROS-MCP server:

```bash
# Source your workspace
source install/setup.bash

# Run the server
ros2 run ros_mcp_server server
```

### Capabilities

Once the server is running, the connected MCP client (e.g., Claude Desktop, Cursor) can perform the following:

-   **Inspect System State**:
    -   List all active nodes: `ros2 node list` via agent.
    -   List all topics: `ros2 topic list` via agent.
    -   Read sensor data: `get_topic_message("/scan")` or `get_topic_message("/odom")`.

-   **Control the Robot**:
    -   Publish velocity commands to move the robot:
        ```python
        # Agent can publish to /cmd_vel
        publish_message("/cmd_vel", "geometry_msgs/msg/Twist", "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}")
        ```

-   **Navigation (if nav2 is running)**:
    -   The agent can trigger navigation actions if the specifics are set up (this demo focuses on simulation bringup).

## credits

This simulation package is based on the [ROBOTIS TurtleBot3 Simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations).

-   **Authors**: ROBOTIS
-   **License**: Apache 2.0 (See `LICENSE` file in `src/turtlebot3_simulations`)
