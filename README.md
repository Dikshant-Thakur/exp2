# Assignment 2 ROS Package

This repository contains the ROS 2 package developed for Assignment 2, which enables a mobile robot to autonomously navigate to the waypoint corresponding to the lowest ID marker in a predefined environment.


## Overview
The goal of this project is to use a robot equipped with a camera and laser scanner to locate markers and move to the waypoint associated with the marker having the lowest ID. 

## Prerequisites
The following tools and dependencies are required:
- ROS 2 Humble
- Gazebo (for simulation)
- Rviz (for visualization)
- SLAM Toolbox (for mapping and localization)
- Nav2 (for navigation)
- PlanSys2 (for task planning)

## Installation
You can also install PlanSys2, Nav2, and SLAM Toolbox using apt-get, but some parameters may need to be changed. Therefore, it is recommended that you use the provided packages.
## Usage
Run these commands in order:
1. Launch simulation (Gazebo + Rviz):
```bash
ros2 launch assignment_2 assignment_2.launch.py
```
2. Start SLAM Toolbox for mapping:
```bash
ros2 launch slam_toolbox online_sync_launch.py
```
3. Launch Nav2 for navigation:
```bash
ros2 launch nav2_bringup navigation_launch.py
```
4. Start PlanSys2 server and nodes:
``` bash
ros2 launch plansys2_patrol_navigation_example patrol_example_fakesim_launch.py
```
5. To initialize the problem, and implement a FSM.
``` bash
ros2 run plansys2_patrol_navigation_example patrolling_controller_node
```



