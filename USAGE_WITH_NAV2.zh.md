# 与Nav2一起使用Waypoint Editor

[English Version](USAGE_WITH_NAV2.md)

本文档说明如何使用waypoint_editor编辑航点，并将它们发送给Nav2进行导航。

## 问题说明

waypoint_editor和Nav2都会启动map_server，导致冲突。此外，waypoint_editor保存的CSV格式航点不能直接被Nav2使用。

## 解决方案

提供了两个桥接节点，它们都可以读取waypoint_editor保存的CSV文件，将航点转换为Nav2可以理解的格式，并发送给Nav2执行：

| 桥接节点 | Nav2 Action | 行为 | 适用场景 |
|---------|-------------|------|----------|
| `waypoint_to_nav2` | `follow_waypoints` | 逐点停靠 — 到达每个航点后停下再前往下一个 | 巡检、逐点任务 |
| `waypoint_through_nav2` | `navigate_through_poses` | 平滑穿越 — 规划连续路径经过所有航点，不停留 | 巡逻、快速穿越 |

## 使用步骤

### 1. 编译项目

```bash
cd ~/ros2_ws
colcon build --packages-select waypoint_editor
source install/setup.bash
```

### 2. 启动Nav2

按照你的正常流程启动Nav2（包括map_server、AMCL、navigation等）。

### 3. 在Nav2的RViz2中添加Waypoint Editor插件

在RViz2界面中：
- 点击 Panels -> Add New Panel -> 选择 "WaypointEditorPanel"
- 点击 Tools -> Add New Tool -> 选择 "Add Waypoint"

现在你可以在地图上添加航点了。

### 4. 保存航点

使用Waypoint Editor Panel中的"Save WPs"按钮保存航点为CSV文件（例如：`/home/user/my_waypoints.csv`）。

### 5. 启动桥接节点并开始导航

根据需要选择以下两种模式之一：

#### 模式 A：逐点停靠（Follow Waypoints）

机器人依次到达每个航点，在每个点停下后再前往下一个。

```bash
ros2 launch waypoint_editor waypoint_to_nav2.launch.py waypoint_file:=/path/to/your/waypoints.csv
```

```bash
ros2 service call /start_waypoint_following std_srvs/srv/Trigger
```

#### 模式 B：平滑穿越（Navigate Through Poses）

机器人规划一条连续路径穿越所有航点，中间不停留。

```bash
ros2 launch waypoint_editor waypoint_through_nav2.launch.py waypoint_file:=/path/to/your/waypoints.csv
```

```bash
ros2 service call /start_waypoint_through std_srvs/srv/Trigger
```
