# Ego-Planner-2D-ROS2
0.通过对优化器底层进行修改，使其优化的轨迹坐标维度为2维（x和y），并对grid map进行重写，使其真正适配地面2D移动机器人。
1.项目地址： https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2

本项目基于浙大高飞团队优秀的开源工作 Ego-Planner 进行深度二次开发。针对地面移动机器人的特性，我从底层算法层面进行了真正的降维重构，使其完全适配 2D 导航场景。

2.🚀 主要特性与改进 (Key Features)

（1）真正的 2D 轨迹优化 (True 2D Optimization)

重构了轨迹优化器的核心数学模型，将优化变量从 3 维降至 2 维（仅优化 x, y），彻底移除了 Z 轴冗余计算，实现了数学意义上的平面轨迹规划。

（2）环境层重构 (Environment Refactoring)

将原有的 3D 环境表达重构为 2D GridMap（栅格地图），大幅降低了内存占用并提高了环境查询与碰撞检测的计算速度。

（3）前端寻路降维 (2D A Search)*

对 Path Searching 模块的 A* 算法进行了降维适配，专注于平面路径搜索。

（4）纯 C++ 核心与解耦 (Pure C++ Core)

剥离了原算法对 ROS1 的依赖，核心算法逻辑采用纯 C++ 实现，具有极高的可移植性。

（5）项目采用 ROS2 作为外壳封装，支持 Rviz2 交互式仿真，开箱即用。

ROS2 仿真支持

（6）全面适配 ROS2 框架，提供流畅的 Rviz2 可视化交互体验。

3.⚠️ 免责声明 (Disclaimer)

本项目属于本人（JackJu）在业余时间进行的个人学习与技术研究成果。

项目代码未应用于任何公司、机构或商业团队的实际项目中。

项目仅供学术交流与学习使用，不涉及任何商业利益冲突或侵权行为。

感谢 ZJU-FAST-Lab 团队的原始开源贡献。


# how to use ?

ROS2 compile

1.运行节点motion_plan
2.运行rviz2
3.设置好全局轨迹与障碍物信息

# more info

微信公众号：机器人规划与控制研究所。https://mp.weixin.qq.com/s/tjHMyyEMzsonYaVbrq4WTQ
如若加轨迹优化与运动控制交流群请通过公众号后台联系我。

b站：机器人算法研究所 https://www.bilibili.com/video/BV11RUfB8ELb/?spm_id_from=333.1387.homepage.video_card.click&vd_source=3e9e0488974285dd9fea47318bfd814e

如果您使用此2D ego-planner 进行二次开发或学术论文代码等进行开源，请引用此git仓库链接：https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2


# 参考项目
https://github.com/ZJU-FAST-Lab/ego-planner
