# 与Nav2一起使用Waypoint Editor

本文档说明如何使用waypoint_editor编辑航点，并将它们发送给Nav2进行导航

## 问题说明

waypoint_editor和Nav2都会启动map_server，导致冲突。此外，waypoint_editor保存的CSV格式航点不能直接被Nav2使用。

## 解决方案

我创建了一个桥接节点`waypoint_to_nav2`，它可以：
1. 读取waypoint_editor保存的CSV文件
2. 将航点转换为Nav2可以理解的格式
3. 通过Nav2的`follow_waypoints` action发送航点

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
- 点击 Panels -> Add New Panel
- 选择 "WaypointEditorPanel"
- 点击 Tools -> Add New Tool
- 选择 "Add Waypoint"

现在你可以在地图上添加航点了。

### 4. 保存航点

使用Waypoint Editor Panel中的"Save WPs"按钮保存航点为CSV文件（例如：`/home/user/my_waypoints.csv`）。

### 5. 启动桥接节点

```bash
ros2 launch waypoint_editor waypoint_to_nav2.launch.py waypoint_file:=/path/to/your/waypoints.csv
```

### 6. 开始导航

调用服务触发航点跟随：

```bash
ros2 service call /start_waypoint_following std_srvs/srv/Trigger
```


