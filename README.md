# ROS 2 RViz Ego Planner Motion Simulation

基于 **Ego-Planner-2D-ROS2** 整理的 ROS 2 + RViz 交互式运动仿真工程。

本仓库不仅包含 Ego Planner 2D 局部规划节点，还集成了简易小车运动仿真、RViz 交互操作、轨迹跟踪以及一键启动功能，可用于快速演示和验证 Ego Planner 的二维轨迹规划效果。

通过 Launch 文件可以同时启动以下节点：

- `ego_planner/motion_plan`：二维局部轨迹规划与可视化节点
- `rviz_car_sim/fake_sim_node`：简易小车运动仿真节点
- `rviz2`：自动加载仓库内置的 RViz 配置

---

## 演示视频

点击下方链接查看 Ego Planner RViz 运动仿真效果：

[点击查看 Ego Planner RViz 运动仿真演示](./ego.mp4)

---

## 功能特性

本工程主要实现以下功能：

- 基于 RViz 设置小车初始位姿
- 通过 RViz 交互添加全局参考路径点
- 通过 RViz 交互添加障碍物
- 根据当前位姿、参考路径和障碍物生成局部轨迹
- 可视化全局路径、局部轨迹和障碍物
- 使用简易运动学模型模拟小车轨迹跟踪
- 实时发布小车位姿、Marker 和 TF
- 通过 Launch 文件一键启动规划、仿真和 RViz

---

## 系统说明

### Ego Planner 规划节点

`ego_planner` 接收以下信息：

- RViz 交互输入的全局路径点
- 仿真小车当前位姿
- RViz 交互输入的障碍物
- 规划触发信号

规划节点根据输入信息生成：

- 全局参考路径
- 局部优化轨迹
- 全局障碍物点云
- 局部障碍物点云

### 小车仿真节点

`rviz_car_sim` 订阅 Ego Planner 输出的局部轨迹，并使用简易运动学模型驱动小车沿轨迹运动。

仿真节点主要发布：

- `/current_pose`：小车当前位姿，供规划节点进行实时重规划
- `/car_marker`：RViz 中的小车模型
- `/global_obstacles`：全局障碍物点云
- `/local_obstacles`：局部障碍物点云
- `map -> base_link`：小车坐标系 TF

RViz 配置文件位于：

```text
src/rviz_car_sim/config/rviz.rviz
```

---

## 环境要求

推荐使用以下环境：

- Ubuntu 22.04
- ROS 2 Humble
- colcon
- RViz 2

如果使用 Docker 环境，可以进入 `ros2_dev` 容器后进行编译和运行。

---

## 编译工程

进入工作区根目录后执行：

```bash
source /opt/ros/humble/setup.bash

colcon build

source install/setup.bash
```

如果只修改了仿真节点、Launch 文件或 RViz 配置，可以仅编译 `rviz_car_sim`：

```bash
source /opt/ros/humble/setup.bash

colcon build --packages-select rviz_car_sim

source install/setup.bash
```

为了避免旧的构建缓存影响运行，也可以进行一次完整清理：

```bash
rm -rf build install log

source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

---

## 启动工程

推荐使用 Launch 文件一次性启动规划节点、仿真节点和 RViz：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch rviz_car_sim bringup.launch.py
```

仓库根目录同时提供了简化启动脚本：

```bash
./start.sh
```

该脚本内部调用：

```bash
ros2 launch rviz_car_sim bringup.launch.py
```

如果脚本没有执行权限，可以先运行：

```bash
chmod +x start.sh
```

---

## RViz 交互说明

工程启动后，可以通过 RViz 工具栏完成路径和障碍物设置。

### 1. 设置小车初始位姿

点击 RViz 工具栏中的：

```text
2D Pose Estimate
```

在地图中点击并拖动鼠标，设置或重置小车的初始位置和朝向。

### 2. 添加全局参考路径

点击 RViz 工具栏中的：

```text
Publish Point
```

在地图中依次点击路径点。

至少需要添加两个路径点，规划节点才可以生成有效的全局参考路径。

### 3. 添加障碍物

点击 RViz 工具栏中的：

```text
2D Nav Goal
```

在地图中点击目标位置。

在当前工程中，`2D Nav Goal` 被用于添加障碍物点，而不是设置导航终点。

### 4. 观察规划和运动效果

完成初始位姿、路径点和障碍物设置后，可以在 RViz 中观察：

- 全局参考路径
- Ego Planner 生成的局部轨迹
- 全局与局部障碍物
- 小车沿局部轨迹运动
- 局部轨迹实时更新过程

---

## 手动触发规划

规划节点默认允许进行轨迹规划。

如果需要手动触发规划，可以执行：

```bash
ros2 topic pub /trigger_plan std_msgs/msg/Bool "{data: true}" --once
```

---

## 主要话题

| 话题名称 | 消息类型 | 功能说明 |
| --- | --- | --- |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 设置或重置小车初始位姿 |
| `/clicked_point` | `geometry_msgs/msg/PointStamped` | 添加全局参考路径点 |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | 添加障碍物点 |
| `/trigger_plan` | `std_msgs/msg/Bool` | 手动触发轨迹规划 |
| `/current_pose` | `geometry_msgs/msg/PoseStamped` | 仿真小车当前位姿 |
| `/visual_global_path` | `nav_msgs/msg/Path` | 全局参考路径可视化 |
| `/visual_local_trajectory` | `nav_msgs/msg/Path` | 局部优化轨迹，供仿真节点跟踪 |
| `/visual_obstacles` | `sensor_msgs/msg/PointCloud2` | 全局障碍物可视化 |
| `/visual_local_obstacles` | `sensor_msgs/msg/PointCloud2` | 局部障碍物可视化 |
| `/car_marker` | `visualization_msgs/msg/Marker` | RViz 小车模型可视化 |

---

## 工程结构

```text
.
├── ego.mp4
├── rviz.rviz
├── start.sh
└── src
    ├── EgoPlanner-ROS2
    │   └── src
    │       └── trajectory_publisher.cpp
    └── rviz_car_sim
        ├── config
        │   └── rviz.rviz
        ├── launch
        │   └── bringup.launch.py
        └── src
            └── fake_sim_node.cpp
```

其中：

- `trajectory_publisher.cpp`：Ego Planner 轨迹规划与可视化节点
- `fake_sim_node.cpp`：小车运动仿真与轨迹跟踪节点
- `bringup.launch.py`：统一启动规划、仿真和 RViz
- `rviz.rviz`：RViz 显示配置
- `start.sh`：工程简化启动脚本
- `ego.mp4`：运动仿真演示视频

---

## 常见问题

### 1. `ros2 launch` 找不到 `bringup.launch.py`

确认已经重新编译 `rviz_car_sim`，并加载当前工作区环境：

```bash
source /opt/ros/humble/setup.bash

colcon build --packages-select rviz_car_sim

source install/setup.bash
```

可以通过以下命令检查 Launch 文件是否已经正确安装：

```bash
ros2 pkg prefix rviz_car_sim
```

---

### 2. RViz 中没有显示小车

首先使用：

```text
2D Pose Estimate
```

设置小车初始位姿。

同时检查 `/car_marker` 是否正常发布：

```bash
ros2 topic echo /car_marker
```

---

### 3. RViz 中没有显示规划轨迹

请确认：

1. 已经设置小车初始位姿。
2. 已通过 `Publish Point` 添加至少两个路径点。
3. 规划节点已经正常启动。
4. `/visual_local_trajectory` 话题正在发布。

可以执行：

```bash
ros2 topic hz /visual_local_trajectory
```

检查局部轨迹发布频率。

---

### 4. 添加路径点后没有触发规划

可以手动发布规划触发信号：

```bash
ros2 topic pub /trigger_plan std_msgs/msg/Bool "{data: true}" --once
```

同时检查当前位姿和路径输入是否正常：

```bash
ros2 topic echo /current_pose
ros2 topic echo /clicked_point
```

---

### 5. 宿主机没有安装 ROS 2

如果 ROS 2 环境位于 Docker 容器中，可以执行：

```bash
docker exec -it ros2_dev bash
```

进入容器后运行：

```bash
cd /workspace/ros2_rviz_ws/ros2_rviz_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch rviz_car_sim bringup.launch.py
```

---

## 轨迹平滑性调整

除了调整 Ego Planner 内部的平滑项、障碍物约束项和动力学约束参数之外，还可以从输入参考轨迹的点间距入手。

适当增大相邻参考路径点之间的距离，可以减少优化器受到的局部参考约束，为轨迹形状调整提供更大的优化空间，从而在一定程度上提升最终轨迹的平滑性。

需要注意的是，参考点间距不宜设置得过大，否则可能导致：

- 原始路径细节丢失
- 转弯位置表达不准确
- 局部障碍物信息利用不足
- 优化轨迹偏离原始参考路径

实际使用时，需要结合地图分辨率、机器人尺寸、运行速度和场景复杂度进行调整。

---

## 项目说明

本仓库的核心规划算法来源于 Ego Planner 相关开源工作。

在原有规划功能的基础上，本工程重点补充和整理了：

- ROS 2 节点适配
- RViz 交互输入
- 全局路径与局部轨迹可视化
- 简易小车运动学仿真
- 局部轨迹跟踪
- Launch 一键启动
- 完整的运动仿真演示流程

本项目主要用于算法学习、轨迹规划验证和 RViz 仿真演示，不建议未经充分测试直接应用于实际机器人产品。

---

## 更多内容

### 微信公众号

**机器人规划与控制研究所**

文章：

[【机器人轨迹优化算法】Ego Planner 2D Motion Simulation](https://mp.weixin.qq.com/s/Ff3IK3oWwibDAN7lcKIvvA)

### Bilibili

**机器人算法研究院**

视频：

[Ego Planner 2D Motion Simulation](https://www.bilibili.com/video/BV1e4Ng6AExQ/)

---

## License

请遵循原始 Ego Planner 项目及本仓库相关依赖的开源许可证。
