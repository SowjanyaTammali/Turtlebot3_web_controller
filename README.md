# TurtleBot3 Web Controller with Navigation Interface

## Description
A web-based controller interface for TurtleBot3 that provides real-time control and navigation capabilities through a browser. This project integrates with ROS2 (Robot Operating System 2) and provides a user-friendly interface for robot control, 2D pose estimation, and navigation goal setting.

## Features
- Real-time robot control through web interface
- 2D Pose Estimation functionality
- Navigation Goal setting
- Robot position and orientation tracking
- Speed control adjustments
- WebSocket-based communication with ROS2

## Technologies Used
- Node.js
- Express.js
- WebSocket (ws)
- ROS2 Humble
- TurtleBot3 Simulation
- HTML5 Canvas for visualization

## Prerequisites
- ROS2 Humble
- Node.js
- TurtleBot3 packages
- Nav2 packages

## Installation & Setup
1. Clone the repository
2. Install dependencies: `npm install`
3. Source ROS2: `source /opt/ros/humble/setup.bash`
4. Launch TurtleBot3 simulation: `ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False map:=$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/maps/turtlebot3_world.yaml`
5. Start the web server: `node website_socket_server.js`
6. Access the interface at: `http://localhost:8080`

## Current Status
The interface successfully communicates with ROS2, allowing for robot control and navigation. The 2D Pose Estimate and Navigation Goal functionalities are operational, with ongoing improvements to coordinate mapping between the web interface and RViz.
