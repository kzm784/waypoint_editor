[日本語](README.ja.md) | [English](README.md)

# Waypoint Editor

![demo](https://raw.github.com/wiki/kzm784/waypoint_editor/images/waypoint_editor_demo.gif)

## Table of Contents
- [Overview](#overview)
- [Development Environment](#development-environment)
- [Installation](#installation)
- [Usage](#usage)

## Overview
This package provides a tool for intuitively editing and saving waypoints used in robot navigation while referencing a 2D map.  
The edited waypoints can be saved in **CSV format**.

## Development Environment
- Ubuntu 22.04 (Jammy Jellyfish)
- ROS 2 Humble Hawksbill

## Installation
Run the following commands in your terminal:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/kzm784/waypoint_editer.git
cd ~/ros2_ws
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build
```

## Usage

### 1. Launching the Waypoint Editor  
Run the following commands to launch the tool:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch waypoint_editer waypoint_editer.launch.py
```

### 2. Loading a 2D Map  
- Use Nav2's `nav2_map_server` to load a 2D map in `.yaml` format.  
- Click the **Load 2D Map** button in the bottom-right panel of RViz2 and select the desired `.yaml` file.

### 3. Adding Waypoints  
- From the toolbar at the top of RViz2, select **Add Waypoint**.  
- Click and drag on the map to add a new waypoint with the desired position and orientation.  
- Once added, each waypoint can:
  - Be **moved or rotated** via drag operations
  - Be **right-clicked** to open a context menu for deletion or other actions

### 4. Saving Waypoints  
- Click **Save Waypoints** button in the bottom-right panel of RViz2.  
- Enter a file name to save the edited waypoints in **CSV format**.

### 5. Loading Waypoints  
- Click **Load Waypoints** button in the bottom-right panel of RViz2 and select the previously saved `.csv` file.  
- The waypoints can then be edited again in the same interface.
