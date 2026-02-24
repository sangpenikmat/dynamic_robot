# Intelligent Obstacle Avoidance Robot (ROS 2 Jazzy)

**Goal:** Implementation of a proportional vector-based avoidance system for a differential drive robot to reach a goal coordinate while navigating through static obstacles.

[Image of LiDAR sector analysis for mobile robot navigation]

## Features
* **Goal Seeking:** Uses Odometry data to calculate the heading toward a specific (x, y) coordinate.
* **Intelligent Avoidance:** Uses 5-sector LiDAR analysis to "slide" around obstacles instead of just stopping.
* **Proportional Control:** Linear and angular speeds are scaled based on the proximity to obstacles.
* **Emergency Recovery:** Automatic spin recovery if an obstacle is detected within the critical safety margin.

## System Requirements
* **OS:** Ubuntu 24.04 (Noble Numbat) on WSL2
* **Middleware:** ROS 2 Jazzy Jalisco
* **Simulator:** Gazebo Harmonic (GZ Sim)
* **GPU:** NVIDIA RTX 2060 (Optimized for OGRE2 rendering)

## Launch & Execution

1. **Build the Workspace:**
   ```bash
   cd ~/dynamic_robot_ws
   colcon build --symlink-install
   source install/setup.bash