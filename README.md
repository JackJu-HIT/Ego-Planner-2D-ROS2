# ROS2 RViz Ego Planner Motion Simulation

这个仓库是在 Ego-Planner-2D-ROS2 基础上整理出的 ROS 2 + RViz 交互式运动仿真工程。当前版本不再只提供规划节点，而是通过 launch 同时启动：

- `ego_planner/motion_plan`：2D 局部规划与轨迹发布节点
- `rviz_car_sim/fake_sim_node`：简易小车运动仿真节点
- `rviz2`：自动加载仓库内的 RViz 配置

## 演示视频

[点击查看 Ego Planner RViz 运动仿真演示](./ego.mp4)

支持 HTML 视频标签的 Markdown 平台也可以直接播放：

<video src="./ego.mp4" controls width="100%"></video>

## 功能说明

`ego_planner` 接收 RViz 交互输入和仿真小车当前位置，生成全局路径可视化、局部轨迹和障碍物可视化。

`rviz_car_sim` 订阅规划出的局部轨迹，使用简易运动学模型驱动车辆运动，并发布：

- `/current_pose`：当前小车位姿，供规划节点使用
- `/car_marker`：小车 Marker
- `/global_obstacles`：全局障碍物点云
- `/local_obstacles`：局部障碍物点云
- `map -> base_link` TF

RViz 配置文件已放入 `rviz_car_sim` 包：

```bash
src/rviz_car_sim/config/rviz.rviz
```

## 环境要求

- ROS 2 Humble
- colcon
- rviz2

如果使用当前 Docker 环境，可以进入 `ros2_dev` 容器后操作。

## 编译

在工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

如果只改了启动和仿真包，可以只编译 `rviz_car_sim`：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select rviz_car_sim
source install/setup.bash
```

## 启动

推荐使用 launch 一次性启动规划、仿真和 RViz：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rviz_car_sim bringup.launch.py
```

根目录也保留了一个简化入口：

```bash
./start.sh
```

它内部会调用：

```bash
ros2 launch rviz_car_sim bringup.launch.py
```

## RViz 交互方式

启动后在 RViz 中使用工具栏交互：

1. 使用 `2D Pose Estimate` 设置或重置小车初始位姿。
2. 使用 `Publish Point` 依次点击至少两个路径点，作为全局路径输入。
3. 使用 `2D Nav Goal` 添加障碍物点。
4. 观察小车沿规划出的局部轨迹运动。

规划节点默认允许规划。如果需要手动触发，也可以发布：

```bash
ros2 topic pub /trigger_plan std_msgs/msg/Bool "{data: true}" --once
```

## 主要话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | RViz 设置/重置小车位姿 |
| `/clicked_point` | `geometry_msgs/msg/PointStamped` | RViz 点击生成全局路径点 |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | RViz 目标点，这里用于添加障碍物 |
| `/current_pose` | `geometry_msgs/msg/PoseStamped` | 仿真小车当前位置 |
| `/visual_global_path` | `nav_msgs/msg/Path` | 全局路径可视化 |
| `/visual_local_trajectory` | `nav_msgs/msg/Path` | 局部轨迹，仿真节点跟踪该轨迹 |
| `/visual_obstacles` | `sensor_msgs/msg/PointCloud2` | 全局障碍物可视化 |
| `/visual_local_obstacles` | `sensor_msgs/msg/PointCloud2` | 局部障碍物可视化 |
| `/car_marker` | `visualization_msgs/msg/Marker` | 仿真小车模型 |

## 文件结构

```text
.
├── rviz.rviz
├── start.sh
└── src
    ├── EgoPlanner-ROS2
    │   └── src/trajectory_publisher.cpp
    └── rviz_car_sim
        ├── config/rviz.rviz
        ├── launch/bringup.launch.py
        └── src/fake_sim_node.cpp
```

## 常见问题

### ros2 launch 找不到 bringup.launch.py

确认已经重新编译并 source 工作区：

```bash
colcon build --packages-select rviz_car_sim
source install/setup.bash
```

### RViz 没有显示小车或轨迹

检查是否已经通过 `2D Pose Estimate` 设置初始位姿，并通过 `Publish Point` 添加了至少两个路径点。

### 宿主机没有 ROS 环境

当前机器上 ROS 环境可能在 Docker 容器中。可以使用：

```bash
docker exec -it ros2_dev bash
cd /workspace/ros2_rviz_ws/ros2_rviz_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rviz_car_sim bringup.launch.py
```

## 参考

核心规划算法来源于 Ego-Planner 相关工作，本仓库重点整理了 ROS 2 RViz 交互、launch 启动和简易运动仿真流程。

