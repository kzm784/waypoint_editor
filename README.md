[日本語](README.ja.md) | [English](README.md)

[![ROS 2 Humble build](https://github.com/kzm784/waypoint_editor/actions/workflows/humble_build.yml/badge.svg?branch=main&label=ROS%202%20Humble%20build)](https://github.com/kzm784/waypoint_editor/actions/workflows/humble_build.yml)
[![ROS 2 Jazzy build](https://github.com/kzm784/waypoint_editor/actions/workflows/jazzy_build.yml/badge.svg?branch=main&label=ROS%202%20Jazzy%20build)](https://github.com/kzm784/waypoint_editor/actions/workflows/jazzy_build.yml)

# Waypoint Editor

![demo](https://raw.github.com/wiki/kzm784/waypoint_editor/images/waypoint_editor_demo.gif)

## Table of Contents
- [Overview](#overview)
- [Development Environment](#development-environment)
- [Installation](#installation)
- [Usage](#usage)
- [License](#License)


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
git clone https://github.com/kzm784/waypoint_editor.git
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
ros2 launch waypoint_editor waypoint_editor.launch.py
```

### 2. Loading a 2D Map  
- Use Nav2's `nav2_map_server` to load a 2D map in `.yaml` format.  
- Click the "**Load Map**" button in the bottom-right panel of RViz2 and select the desired `.yaml` file.

![load_map_demo](https://raw.github.com/wiki/kzm784/waypoint_editor/images/loading_2d_map_demo.gif)


### 3. Adding Waypoints  
- From the toolbar at the top of RViz2, select "**Add Waypoint**".  
- Click and drag on the map to add a new waypoint with the desired position and orientation.  
- Once added, each waypoint can:
  - Be **moved or rotated** via drag operations
  - Be **right-clicked** to open a context menu for deletion or other actions

![adding_waypoints_demo](https://raw.github.com/wiki/kzm784/waypoint_editor/images/Adding_waypoints_demo.gif)


### 4. Saving Waypoints  
- Click "**Save WPs**" button in the bottom-right panel of RViz2.  
- Enter a file name to save the edited waypoints in **CSV format**.

![saving_waypoints_demo](https://raw.github.com/wiki/kzm784/waypoint_editor/images/saving_waypoints.gif)


### 5. Loading Waypoints  
- Click "**Load WPs**" button in the bottom-right panel of RViz2 and select the previously saved `.csv` file.  
- The waypoints can then be edited again in the same interface.

![loading_waypoints_demo](https://raw.github.com/wiki/kzm784/waypoint_editor/images/loading_waypoints.gif)


## License
[![License: Apache-2.0](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

This project is licensed under the Apache License, Version 2.0.  
See the [LICENSE](LICENSE) file for details.
