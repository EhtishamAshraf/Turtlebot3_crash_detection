# mrob_highlight_controller

This repository contains the implementation of Service and Client nodes for controlling a robot using the `mrob_highlight_controller` package. The package includes functionalities for mapping unknown environments and navigating to specific points using ROS.

## Overview

The `mrob_highlight_controller` package includes:

- **Service Nodes**: For handling various service requests such as starting and stopping the robot.
- **Client Nodes**: For sending service requests to the service nodes.

## Features

- **Mapping**: Create a map of an unknown environment using the `gmapping` package.
- **Navigation**: Navigate to a specific point using the `AMCL` and `move_base` nodes.

## Installation

1. Clone the repository:

    ```bash
    git clone https://github.com/yourusername/mrob_highlight_controller.git
    ```

2. Navigate to the workspace:

    ```bash
    cd ~/Desktop/my_github/lab5/lab5_ws
    ```

3. Build the workspace:

    ```bash
    catkin_make
    ```

4. Source the workspace:

    ```bash
    source devel/setup.bash
    ```

## Usage

### Mapping

To create a map of an unknown environment:

1. Launch the mapping node:

    ```bash
    roslaunch mrob_highlight_controller mapping.launch
    ```

2. Control the robot using the teleop keyboard to explore the environment:

    ```bash
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py
    ```

3. Save the map:

    ```bash
    rosrun map_server map_saver -f ~/Desktop/my_github/lab5/lab5_ws/src/mrob_highlight_controller/maps/map
    ```

### Navigation

1. Launch the navigation stack:

    ```bash
    roslaunch mrob_highlight_controller navigation.launch
    ```

2. Set the initial pose using **2D Pose Estimate** in RViz.

3. Set a goal position for the robot using **2D Nav Goal** in RViz.

### Service Nodes

#### Start/Stop Service

To start or stop the robot, use the `robot_start_stop` service:

- **Service file**: `srv/robot_start_stop.srv`
- **Client node**: `robot_emergency_stop_client.cpp`

Example usage:

```bash
rosrun mrob_highlight_controller robot_emergency_stop_client

