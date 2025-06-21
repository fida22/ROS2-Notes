# 4-Wheeled Differential Drive Robot Simulation in ROS 2


<br>

# Project Overview

This project simulates a 4-wheeled differential-drive robot in ROS 2 with integrated sensors and basic obstacle avoidance. 
The robot is equipped with a LIDAR and camera and automatically stops when approaching a wall.

<br>

# Installation and Usage

## Installation

1. Install ROS2 humble, Gazebo Classic
2. Clone this repository into workkspace’s src directory:
    
    ```bash
    cd ~/ros2_ws/src
    git clone <your_repo_url>
    ```
    
3. Build and source the Environment
    
    ```bash
    cd ~/ros2_ws
    colcon build
    source install/setup.bash
    
    ```
    

## Running the Simulation

- Run the launch file inside the workspace

```bash
ros2 launch four_wheel_robot simulation.launch.py
```

## For Rviz Model

- Run the following command in the workspace directory

```bash
ros2 launch urdf_tutorial display.launch.py model:=/home/fida/marstask_ros2_ws/src/four_wheel_robot/urdf/four_wheel_robot.urdf.xacro
```

## Navigation of robot

robot is navigated using teleop_twist_keyboard

![teleop_keyboard](https://github.com/fida22/ROS2-Notes/blob/2cb3e7d87dac1bfe6bf790571363443eeb0078c9/images/teleop_keyboard.png)

# File overview

| File | Description |
| --- | --- |
| /src/four_wheel_robot/urdf/**four_wheel_robot.urdf.xacro** | Contains the robot description including links joints and gazebo plugins |
| /src/four_wheel_robot/worlds/**final_wall.world** | contains world file |
| src/four_wheel_robot/four_wheel_robot/**obstacle_stop_node.py** | This node stops the robot when it reaches 0.5 m infront of an obstacle/wall |
| /src/four_wheel_robot/launch/**simulate.launch.py** | Launches Gazebo World, Robot state Publisher, Spawn Robot in gazebo,Teleop node, Obstacle stop Node |

## Obstacle Stop Node

### **Overview**

This ROS 2 node provides automatic obstacle avoidance for a robot by:

- Monitoring LIDAR sensor data for obstacles in the robot's path
- Stopping the robot when obstacles are too close
- Resuming movement when the path is clear
- Allowing manual override via teleop commands

### **Key Features**

- **Obstacle Detection**: Uses point cloud data to detect obstacles in a 1m wide area in front of the robot
- **Safety Stop**: Automatically stops the robot when obstacles are closer than **`min_safe_distance`** (0.5m)
- **Hysteresis Control**: Requires obstacles to move beyond **`resume_distance`** (0.7m) before resuming movement
- **Manual Override**: Accepts new movement commands even when obstacles are present
- **Efficient Processing**: Only checks relevant points in the point cloud (ignores Z-axis and distant points)

### **Node Structure**

**Publishers**

- **`/cmd_vel`** - Publishes movement commands (Twist messages)

**Subscribers**

- **`/gazebo_ros_laser_controller/out`** - Receives LIDAR point cloud data
- **`/cmd_vel_input`** - Receives teleoperation commands

**Parameters**

- **`min_safe_distance`** = 0.5m (stop threshold)
- **`resume_distance`** = 0.7m (resume threshold)

### **Behavior**

1. **Normal Operation**:
    - Robot moves according to teleop commands when path is clear
2. **Obstacle Detected**:
    - Stops immediately if obstacle < 0.5m ahead
    - Logs warning message with distance
3. **Path Clear**:
    - Resumes movement when obstacle > 0.7m
    - Logs info message when resuming
4. **Manual Override**:
    - New teleop commands force resume regardless of obstacles

## urdf.xacro file

- The **chassis** serves as the **base link** of the robot.
- Wheels are defined using a reusable `<xacro:macro>` for simplicity and modularity.
- A cylindrical LiDAR sensor is mounted at the front-top of the chassis.
- A camera is placed on top of the LiDAR, facing forward.
- The following Gazebo plugins are integrated:
    - `Differential Drive` plugin for robot movement
    - `LiDAR Sensor` plugin for obstacle detection
    - `Camera Sensor` plugin for visual data capture


      
![view_frames](https://github.com/fida22/ROS2-Notes/blob/2cb3e7d87dac1bfe6bf790571363443eeb0078c9/images/view_frames.png)


![robot_image](https://github.com/fida22/ROS2-Notes/blob/641edc90a4a44acd6255424d16279c278db71df8/images/four_wheel_robo.png)

