# ROS MCP Server Demos
This repository is a collection of demos and tutorials on connecting AI LLMs to Robots using https://github.com/robotmcp/ros-mcp-server

# In this Repository
Each sub-folder in this repository can be treated as a standalone project with its own readme and pyproject.toml file. Below is a list of demos along with a brief description of each.


### 1: TugBot Simulation in Gazebo

This example demonstrates how to control a **Tugbot mobile robot** inside a warehouse environment using **Ignition Gazebo (Fortress)**.

Using natural language and the `ROS-MCP server`, you can dictatate movement commands (navigation), inspect sensor data (Lidar), and check the robot's position.

[View Example 1](./1_Gazebo_Tugbot/README.md)

[Usage and Installation Video](https://youtu.be/rnea0zybCBo)

--- 

### 2. Unitree Go2 Simulation using CHAMP inside Gazebo

This example demonstrates how to control a **Unitree Go2 Quadruped Robot** inside a physics simulation using **Ignition Gazebo (Fortress)**.

It leverages the **CHAMP** quadruple controller to manage complex gait and posture. Using natural language, you can control the robot's walking velocity, execute turns, and manage its stance, as well as inspect sensors like Lidar and IMU.

[View Example 2](./2_UnitreeGo2_simulation/README.md)


[Usage and Installation Video](https://youtu.be/6EQpNAicpiI)

---
