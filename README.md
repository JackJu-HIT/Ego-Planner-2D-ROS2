# Ego-Planner-2D-ROS2

[![ROS2](https://img.shields.io/badge/ROS2-Humble%2FIron%2FJazzy-blue.svg)](https://docs.ros.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)]()
[![C++](https://img.shields.io/badge/Language-C++17-orange.svg)]()

> **真正的地面移动机器人 2D 轨迹优化方案**  
> 基于浙大高飞团队优秀的开源工作 Ego-Planner 进行深度二次开发，从底层算法层面进行了真正的降维重构，使其完全适配 2D 导航场景。

🔗 **项目地址**：[https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2](https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2)

---

## 📅 更新日志（Changelog）

### 最新版本：ROS 2 + RViz 交互式运动仿真

基于 **Ego-Planner-2D-ROS2** 整理并扩展的 ROS 2 + RViz 交互式运动仿真工程。

> [!IMPORTANT]
> 最新功能位于 `develop` 分支，请先切换分支：

```bash
git switch develop
```

当前版本不仅包含 Ego Planner 2D 局部轨迹规划节点，还集成了以下功能：

- 简易小车运动仿真
- RViz 交互式路径设置
- RViz 交互式障碍物添加
- 局部轨迹跟踪
- 规划结果实时可视化
- Launch 一键启动

通过 Launch 文件可同时启动以下节点：

| 节点 | 功能说明 |
| --- | --- |
| `ego_planner/motion_plan` | 二维局部轨迹规划与可视化 |
| `rviz_car_sim/fake_sim_node` | 简易小车运动仿真与轨迹跟踪 |
| `rviz2` | 自动加载仓库内置的 RViz 配置 |

---

### 2025-12-10

#### ✨ 新增功能

- 新增 GridMap 膨胀障碍物的 RViz 2 可视化功能。
- 支持通过以下话题查看局部障碍物及其膨胀结果：

```text
/visual_local_obstacles
```

#### 🐛 问题修复

- 修复 GridMap 在特定边界条件下可能出现的异常问题。
- 提升局部地图在边界区域更新时的稳定性。

---

## 🚀 项目简介 (Introduction)

本项目针对地面移动机器人（AGV/AMR）的特性，对原有的 Ego-Planner 算法进行了核心重构。不同于简单的“压扁”三维轨迹，我们从**优化器底层数学模型**到**环境地图表达**进行了全面的二维化改造，实现了更高效、更轻量的平面轨迹规划。

### ✨ 核心特性 (Key Features)

#### 1. 真正的 2D 轨迹优化 (True 2D Optimization)
重构了轨迹优化器的核心数学模型，将优化变量从 3 维降至 2 维（仅优化 $x, y$）。**彻底移除了 Z 轴的冗余计算**，实现了数学意义上的平面轨迹规划，显著提升了求解效率。

#### 2. 环境层重构 (Environment Refactoring)
将原有的 3D 环境表达重构为 **2D GridMap（栅格地图）**。这一改进大幅降低了内存占用，并显著提高了环境查询与碰撞检测的计算速度。

#### 3. 寻路算法降维 (2D A* Search)
对 Path Searching 模块中的 A* 算法进行了降维适配，专注于平面路径搜索，生成的初始路径更贴合地面移动需求。

#### 4. 纯 C++ 核心与解耦 (Pure C++ Core)
剥离了原算法对 ROS1 的依赖，核心算法逻辑采用**纯 C++ 实现**，模块化程度高，具有极高的可移植性，易于集成到其他系统。

#### 5. 全面适配 ROS 2 (ROS 2 Support)
项目采用 ROS 2 作为外壳封装，完全适配 ROS 2 通信框架。支持 Rviz2 交互式仿真，提供流畅的可视化交互体验，开箱即用。

---

## 🛠️ 编译与运行 (How to use)

请确保您的环境已安装 ROS 2 及相关依赖。

### 1. 编译项目
```bash
colcon build
source install/setup.bash
```

### 2. 运行步骤
1.  **启动规划节点**：
    ```bash
    ros2 run ego_planner motion_plan
    ```
2.  **启动可视化界面**：
    ```bash
    rviz2
    # 建议加载项目提供的 rviz 配置文件（如有）
    ```
3.  **交互测试**：
    - 在 Rviz2 中使用 `2D Pose Estimate` 设置起点（或使用默认起点）。
    - 使用 `2D Nav Goal` 设置全局目标点。
    - 使用 `Publish Point` 工具添加模拟障碍物。
    - 观察话题 `visual_local_obstacles` 查看膨胀后的障碍物信息。

---

## 🔗 参考与致谢 (References)

本项目核心算法源自 **ZJU-FAST-Lab** 团队的优秀开源工作，特此感谢。

*   **Original Project**: [ZJU-FAST-Lab/ego-planner](https://github.com/ZJU-FAST-Lab/ego-planner)

---

## 📢 更多信息 (More Info)

如果您对轨迹优化与运动控制感兴趣，欢迎关注我的媒体渠道获取更多技术分享：

*   **微信公众号**：`机器人规划与控制研究所`
    *   [点击阅读相关技术文章](https://mp.weixin.qq.com/s/tjHMyyEMzsonYaVbrq4WTQ)
    *   *如需加入交流群，请通过公众号后台联系我。*
*   **Bilibili**：`机器人算法研究所`
    *   [观看演示视频](https://www.bilibili.com/video/BV11RUfB8ELb/)

---

## ⚠️ 免责声明 (Disclaimer)

1.  本项目属于本人 (**JackJu**) 在业余时间进行的个人学习与技术研究成果。
2.  项目代码**未应用**于任何公司、机构或商业团队的实际项目中。
3.  项目仅供**学术交流与学习使用**，不涉及任何商业利益冲突或侵权行为。
4.  如果您使用此 2D ego-planner 进行二次开发或发表学术论文，**请引用本 Git 仓库链接**：
    > https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2

---
*Last updated: 2025-12-30*
