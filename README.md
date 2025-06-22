# 🤖 Person Following Robot

A ROS-based person detection and following system for TurtleBot3 robots using YOLO object detection and laser scanner-based obstacle avoidance.

## 📋 Overview

This project implements an autonomous person-following robot that can:
- Detect and track persons using YOLOv8 computer vision
- Follow detected persons while maintaining safe distance
- Avoid obstacles using laser scanner data
- Navigate around obstacles and return to following behavior
- Search for persons when none are detected

## 🔧 Hardware Requirements

- TurtleBot3 Waffle Pi robot
- Raspberry Pi camera module
- LiDAR sensor (for obstacle detection)
- Network connection between robot and control computer

## 💻 Software Dependencies

### System Requirements
- Ubuntu 20.04 (recommended)
- ROS Noetic
- Python 3.8+

### Required Packages
```bash
# ROS packages
sudo apt install ros-noetic-image-transport ros-noetic-cv-bridge ros-noetic-vision-opencv
sudo apt install python3-opencv libopencv-dev ros-noetic-image-proc

# Python packages
sudo apt-get install python-is-python3
sudo apt install python3-pip
pip install ultralytics
```

## 📥 Installation

1. Clone this repository to your catkin workspace:
```bash
cd ~/catkin_ws/src/
git clone https://github.com/C-H-E-N-Zhihao/Person_Follower_ROS
mv Person_Follower_ROS my_following_person_package
cd my_following_person_package/scripts
chmod +x *.py
```

2. Build the workspace:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

3. Set up environment variables:
```bash
echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
source ~/.bashrc
```

## 🚀 Usage

The system requires multiple terminals to run. Follow these steps in order:

### Terminal 1: Start ROS Core
```bash
roscore
```

### Terminal 2: Robot Connection and Hardware Launch
```bash
# Connect to the robot
ssh <robot_ip>

# Launch robot base
roslaunch turtlebot3_bringup turtlebot3_robot.launch

# In a new terminal on the robot (or execute crtl+Z), launch camera
roslaunch turtlebot3_bringup turtlebot3_rpicamera.launch
```

### Terminal 3: Remote Bringup
```bash
roslaunch turtlebot3_bringup turtlebot3_remote.launch
```

### Terminal 4: Run Person Following System
```bash
rosrun my_following_person_package main_person_follower.py
```

## 🧩 System Architecture

### Core Components

1. **PersonDetector** (`person_detector.py`)
   - Uses YOLOv8 for real-time person detection
   - Processes camera feed and provides person location data

2. **ObstacleDetector** (`obstacle_detector.py`)
   - Processes LiDAR data for obstacle detection
   - Monitors front sector (345°-15°) and left sector (75°-105°)
   - Configurable detection distances

3. **RobotController** (`robot_controller.py`)
   - State machine implementation with four states:
     - **SEARCHING**: Rotating to find a person
     - **FOLLOWING**: Moving toward detected person
     - **AVOIDING**: Executing obstacle avoidance maneuver
     - **WAITING**: Stationary when person is centered and obstacle ahead

4. **OdomHelper** (`odom_helper.py`)
   - Provides odometry-based rotation control
   - Enables precise angular movements for obstacle avoidance

5. **MoveKobuki** (`move_robot.py`)
   - Low-level movement interface
   - Publishes velocity commands to `/cmd_vel` topic

### State Machine Behavior

- **SEARCHING**: Robot rotates in place looking for a person
- **FOLLOWING**: Robot moves forward while adjusting direction to center the person
- **AVOIDING**: Robot executes a 90° rotation, moves forward, then rotates back
- **WAITING**: Robot stops when person is centered but obstacle is detected ahead

## 📡 Topics

### Subscribed Topics
- `/camera/image` (sensor_msgs/Image): Camera feed for person detection
- `/scan` (sensor_msgs/LaserScan): LiDAR data for obstacle detection
- `/odom` (nav_msgs/Odometry): Robot odometry for precise rotations

### Published Topics
- `/cmd_vel` (geometry_msgs/Twist): Robot velocity commands

## 📜 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🔥 Demo
[🎬 Watch demo video](https://github.com/user-attachments/assets/8469ec2c-ddee-4241-85b4-c9284b3eb90b)

## 👥 Authors
<div align="center">
  
<!-- Team Members -->
<p>
  <a href="https://www.linkedin.com/in/zhihao-chen-584aa52b5/">
    <img src="https://img.shields.io/badge/Zhihao_Chen-LinkedIn-%2300a0dc?style=for-the-badge&logo=linkedin&logoColor=white" alt="Zhihao Chen"/>
  </a>
  &nbsp;&nbsp;
  <a href="https://www.linkedin.com/in/zhiqian-zhou-196350300/">
    <img src="https://img.shields.io/badge/Zhiqian_Zhou-LinkedIn-%2300a0dc?style=for-the-badge&logo=linkedin&logoColor=white" alt="Zhiqian Zhou"/>
  </a>
</p>

</div>
