/*
MIT License

Copyright (c) 2025 Chunyu Ju

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
@Function: Ego Planner + RViz Interactive Input (Global Path + Obstacles)
@Create by: juchunyu@qq.com
@Date: 2025-11-01 18:58:01
*/
#ifndef TRAJECTORY_OBSTACLES_PUBLISHER_H
#define TRAJECTORY_OBSTACLES_PUBLISHER_H

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <visualization_msgs/msg/marker_array.hpp>
#include <cmath>
#include <vector>
#include <memory>
#include <mutex>

// Ego Planner相关头文件
#include "planner_interface.h"

using namespace ego_planner;

class TrajectoryAndObstaclesPublisher : public rclcpp::Node
{
public:
    TrajectoryAndObstaclesPublisher();
    ~TrajectoryAndObstaclesPublisher() = default;

private:
    void init_ego_planner_base();
    void publish_and_plan();
    
    // 回调函数
    void rviz_global_path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void rviz_obstacles_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void rviz_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void pose_estimate_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void trigger_plan_callback(const std_msgs::msg::Bool::SharedPtr msg);
    
    // 发布函数
    void publish_global_path();
    void publish_planned_trajectory();
    void publish_obstacles();
    void publish_a_star_path();
    void publish_local_obstacles();

    // 辅助函数
    void generate_straight_path(const geometry_msgs::msg::PoseStamped& start, 
                               const geometry_msgs::msg::PoseStamped& goal);
    void add_obstacle_at_position(double x, double y);

    void discretize_trajectory(const std::vector<PathPoint>& original_trajectory,
                           std::vector<PathPoint>& discrete_trajectory,
                           double interval = 0.1);

    double distance(const PathPoint& p1, const PathPoint& p2);

    // ROS 2发布者/订阅者
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr a_star_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_traj_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obs_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obs_local_pub_;

    
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr rviz_global_path_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr rviz_obstacles_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr rviz_point_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_estimate_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_plan_sub_;
    
    rclcpp::TimerBase::SharedPtr timer_;

    // Ego Planner核心对象
    std::shared_ptr<PlannerInterface> ego_planner_;

    // 数据存储
    std::vector<PathPoint> global_plan_traj_;
    std::vector<ObstacleInfo> obstacles_;
    std::mutex data_mutex_;
    bool has_valid_global_path_;
    bool has_obstacles_;
    bool should_plan_;
    bool needs_replan_;  // 新增：是否需要重新规划的标志
    bool flag_ = false;
    std::vector<PathPoint> planned_traj;

    PathPoint  cur_pose_;

    vector<vector<Eigen::Vector2d>> a_star_pathes_;

    // 参数
    double max_vel_ = 2.0;
    double max_acc_ = 3.0;
    double max_jerk_ = 4.0;
    double map_resolution_ = 0.1;
    double map_x_size_ = 50.0;
    double map_y_size_ = 50.0;
    double map_z_size_ = 10.0;
    Eigen::Vector3d map_origin_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    double map_inflate_value_ = 1.0;

    double theta_ = 0.0;
};

#endif // TRAJECTORY_OBSTACLES_PUBLISHER_H