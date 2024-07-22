# Turtlebot 3 Crash Detection and Emergency Stop

This repository contains the implementation of Service and Client nodes for controlling TB3 using the custom package `mrob_highlight_controller`. The package is designed to prevent the robot from crashing into obstacles using Laser Scanner data and detects crashes using IMU sensor data.

## Overview

The `mrob_highlight_controller` package includes:

- **Service Node**: For handling service request to stop the robot.
- **Client Nodes**: For sending service requests to the service node.

## Features

- **Emergency Stop**:
1.    This client node sends requests to stop the robot once the robot gets a bit close to the pillar.

2.    When the robot is within the threshold distance from the pillar the speed is set to "0" and
      immediately requests the server to stop the robot.
  
- **Crash Detection**:
1.    This client node sends requests to stop the robot once the robot crashes into the pillar.

2.    When the robot hits the pillar the acceleration drastically increases, and hence the client 
      immediately requests the server to stop the robot.

### Demo Video
Click on the below image, to open the simulation video:
[![Watch the video](https://github.com/EhtishamAshraf/Turtlebot3_crash_detection/blob/main/simulation_img.png)](https://youtu.be/F_WYwv8lBlQ)

## Installation
1. Clone the repository on your PC:

    ```bash
    git clone https://github.com/EhtishamAshraf/Turtlebot3_crash_detection.git
    ```

2. Navigate to the workspace and build it:

    ```bash
    catkin_make
    ```

3. Source the workspace:

    ```bash
    source devel/setup.bash
    ```

## Usage
1. If you want to call the service manually then run the following command:
    ```bash
    rosservice call /robot_start_stop 'true'
    ```
2. If you want to call the service to stop the robot as soon as it gets closer to the obstacle, then navigate to the launch file, and run the following command:
    ```bash
    roslaunch my_launch_file.launch
    ```
2. If you want to call the service to stop the robot after it has hit the obstacle, then navigate to the launch file, and run the following command:
    ```bash
    roslaunch my_launch_file.launch run_emergency_stop:='false'
    ```
#### Start/Stop Service

To start or stop the robot, use the `robot_start_stop` service:

- **Service file**: `srv/robot_start_stop.srv`


