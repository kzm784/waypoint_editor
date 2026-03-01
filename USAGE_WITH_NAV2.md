# Using Waypoint Editor with Nav2

[中文版本](USAGE_WITH_NAV2.zh.md)

This document explains how to use waypoint_editor to edit waypoints and send them to Nav2 for navigation.

## Problem

Both waypoint_editor and Nav2 start their own map_server, causing conflicts. Additionally, the CSV format saved by waypoint_editor cannot be used directly by Nav2.

## Solution

Two bridge nodes are provided. Both read waypoint_editor's CSV files, convert the waypoints to Nav2-compatible format, and send them for execution:

| Bridge Node | Nav2 Action | Behavior | Use Case |
|-------------|-------------|----------|----------|
| `waypoint_to_nav2` | `follow_waypoints` | Stop at each waypoint before moving to the next | Inspection, point-by-point tasks |
| `waypoint_through_nav2` | `navigate_through_poses` | Plan a smooth path through all waypoints without stopping | Patrol, fast traversal |

## Usage

### 1. Build

```bash
cd ~/ros2_ws
colcon build --packages-select waypoint_editor
source install/setup.bash
```

### 2. Launch Nav2

Start Nav2 as usual (map_server, AMCL, navigation stack, etc.).

### 3. Add Waypoint Editor Plugin in RViz2

In the RViz2 interface:
- Panels -> Add New Panel -> select "WaypointEditorPanel"
- Tools -> Add New Tool -> select "Add Waypoint"

You can now add waypoints on the map.

### 4. Save Waypoints

Click "Save WPs" in the Waypoint Editor Panel to save waypoints as a CSV file (e.g. `/home/user/my_waypoints.csv`).

### 5. Launch Bridge Node and Start Navigation

Choose one of the two modes:

#### Mode A: Follow Waypoints (stop at each point)

The robot visits each waypoint in order, stopping at each one before proceeding to the next.

```bash
ros2 launch waypoint_editor waypoint_to_nav2.launch.py waypoint_file:=/path/to/your/waypoints.csv
```

```bash
ros2 service call /start_waypoint_following std_srvs/srv/Trigger
```

#### Mode B: Navigate Through Poses (smooth traversal)

The robot plans a continuous path through all waypoints without stopping at intermediate points.

```bash
ros2 launch waypoint_editor waypoint_through_nav2.launch.py waypoint_file:=/path/to/your/waypoints.csv
```

```bash
ros2 service call /start_waypoint_through std_srvs/srv/Trigger
```
