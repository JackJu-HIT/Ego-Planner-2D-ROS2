#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include <mutex>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp" // 用于 initialpose
#include "geometry_msgs/msg/point32.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "nav_msgs/msg/path.hpp"

#include "tf2/LinearMath/Quaternion.h"
// #include "tf2_utils/Transform.h" // 包含 getYaw
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rviz_car_sim/msg/current_pose.hpp"
#include <tf2/utils.h>

using namespace std::chrono_literals;

class FakeSimNode : public rclcpp::Node
{
public:
    FakeSimNode() : Node("fake_sim_node")
    {
        // 1. 发布者
        // path_pub_ = this->create_publisher<nav_msgs::msg::Path>("sim_debug_path", 10);
        car_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("car_marker", 10);
        global_obstacle_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud>("global_obstacles", 10);
        local_obstacle_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud>("local_obstacles", 10);
        cur_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("current_pose", 10);

        // TF 广播
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 2. 订阅者
        // 订阅局部规划出的轨迹
        local_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "visual_local_trajectory",
            10,
            std::bind(&FakeSimNode::local_path_callback, this, std::placeholders::_1)
        );

        // 订阅 RViz 的 2D Pose Estimate，用于重置仿真位置
        initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose",
            10,
            std::bind(&FakeSimNode::initial_pose_callback, this, std::placeholders::_1)
        );

        // 3. 定时器 (50Hz)
        timer_ = this->create_wall_timer(50ms, std::bind(&FakeSimNode::loop, this));

        RCLCPP_INFO(this->get_logger(), "简易仿真器启动！");
        RCLCPP_INFO(this->get_logger(), "等待 /initialpose 初始化位置，或 /visual_local_trajectory 开始运动");
    }

private:
    void loop()
    {
        // 1. 运动学更新
        update_physics();
        
        // 2. 纯追踪控制计算 (计算下一时刻的 v, w)
        run_control_logic();

        // 3. 发布 TF 和 可视化
        publish_tf_and_marker();

        // 4. 感知模拟
        process_perception();
    }

    // 处理 RViz 的重置信号
    void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(path_mutex_);
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        robot_yaw_ = tf2::impl::getYaw(q);

        current_v_ = 0.0;
        current_w_ = 0.0;
        received_path_.poses.clear(); // 重置后清空旧路径
        
        RCLCPP_INFO(this->get_logger(), "仿真器位置重置: x=%.2f, y=%.2f", robot_x_, robot_y_);
    }

    // 接收规划路径
    void local_path_callback(const nav_msgs::msg::Path::SharedPtr path_msg)
    {
        std::lock_guard<std::mutex> lock(path_mutex_);
        if (path_msg->poses.empty()) return;

        // ！！！关键修复：绝对不要在这里重置 robot_x_, robot_y_ ！！！
        // 仅仅更新参考路径
        received_path_ = *path_msg;
        
        // 可以在这里打印一下调试信息，但不影响物理状态
        // RCLCPP_INFO(this->get_logger(), "收到新路径，点数: %zu", received_path_.poses.size());
    }

    void update_physics()
    {
        double dt = 0.02; // 20ms

        // 简单的差速模型
        robot_x_ += current_v_ * std::cos(robot_yaw_) * dt;
        robot_y_ += current_v_ * std::sin(robot_yaw_) * dt;
        robot_yaw_ += current_w_ * dt;

        // 规范化角度
        if (robot_yaw_ > M_PI) robot_yaw_ -= 2 * M_PI;
        else if (robot_yaw_ < -M_PI) robot_yaw_ += 2 * M_PI;

        // // 发布当前位置给 Planner
        // auto msg = rviz_car_sim::msg::CurrentPose();
        // msg.x = robot_x_;
        // msg.y = robot_y_;
        // msg.theta = robot_yaw_;
        std::cout << "[update_physics] pose =" << robot_x_ << "," << robot_y_ << "," << robot_yaw_ << std::endl;
        // cur_pose_publisher_->publish(msg);
        
        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        msg.pose.position.x = robot_x_;
        msg.pose.position.y = robot_y_;
        msg.pose.position.z = 0.0; // 平面移动 z 为 0

        // --- 核心：Theta (Yaw) 转四元数 ---
        tf2::Quaternion q;
        // setRPY 代表 Set Roll(翻滚), Pitch(俯仰), Yaw(偏航)
        // 对于平面移动的小车，Roll=0, Pitch=0, 只有 Yaw 需要设置
        q.setRPY(0.0, 0.0, robot_yaw_); // robot_yaw_ 就是你的 theta

        // 将 tf2 的四元数赋值给消息格式
        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        msg.pose.orientation.w = q.w();
        // 发布
        cur_pose_publisher_->publish(msg);
    }

    void publish_tf_and_marker()
    {
        rclcpp::Time now = this->get_clock()->now();

        // 1. TF: map -> base_link
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now;
        t.header.frame_id = "map";
        t.child_frame_id = "base_link";
        t.transform.translation.x = robot_x_;
        t.transform.translation.y = robot_y_;
        t.transform.translation.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, robot_yaw_);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);

        // 2. Car Marker
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link"; // 相对于车身
        marker.header.stamp = now;
        marker.ns = "sim_car";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.4; marker.scale.y = 0.2; marker.scale.z = 0.1;
        marker.color.a = 0.8; marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0;
        
        // Marker在 base_link 下是原点
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.w = 1.0;

        car_marker_pub_->publish(marker);
    }

    void process_perception()
    {
        // 模拟生成障碍物 (base_link 系)
        sensor_msgs::msg::PointCloud local_cloud;
        local_cloud.header.frame_id = "base_link";
        local_cloud.header.stamp = this->get_clock()->now();

        // 示例：在车前方 2m 处生成一排虚拟障碍物
        // 实际应用中可以根据 robot_x_ 的位置动态改变障碍物
        /* 
        for (float y = -0.5; y <= 0.5; y += 0.1) {
            geometry_msgs::msg::Point32 p;
            p.x = 2.0; 
            p.y = y;
            p.z = 0.0;
            local_cloud.points.push_back(p);
        }
        */
        local_obstacle_pub_->publish(local_cloud);

        // 转换到全局坐标系
        sensor_msgs::msg::PointCloud global_cloud;
        global_cloud.header.frame_id = "map";
        global_cloud.header.stamp = local_cloud.header.stamp;

        double cos_t = std::cos(robot_yaw_);
        double sin_t = std::sin(robot_yaw_);

        for (const auto& p_local : local_cloud.points) {
            geometry_msgs::msg::Point32 p_global;
            p_global.x = robot_x_ + p_local.x * cos_t - p_local.y * sin_t;
            p_global.y = robot_y_ + p_local.x * sin_t + p_local.y * cos_t;
            p_global.z = p_local.z;
            global_cloud.points.push_back(p_global);
        }
        global_obstacle_pub_->publish(global_cloud);
    }

    void run_control_logic()
    {
        std::lock_guard<std::mutex> lock(path_mutex_);
        if (received_path_.poses.empty()) {
            current_v_ = 0.0;
            current_w_ = 0.0;
            return;
        }

        // 1. 寻找路径上距离当前位置最近的点 (Match Point)
        double min_dist = 1e9;
        size_t closest_idx = 0;
        for (size_t i = 0; i < received_path_.poses.size(); ++i) {
            double dx = received_path_.poses[i].pose.position.x - robot_x_;
            double dy = received_path_.poses[i].pose.position.y - robot_y_;
            double dist = std::sqrt(dx*dx + dy*dy);
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
            }
        }

        // 2. 寻找前瞻点 (Lookahead Point)，距离最近点 Ld 远处
        double lookahead_dist = 1.0; // 前瞻距离 0.3m
        size_t target_idx = closest_idx;
        for (size_t i = closest_idx; i < received_path_.poses.size(); ++i) {
            double dx = received_path_.poses[i].pose.position.x - received_path_.poses[closest_idx].pose.position.x;
            double dy = received_path_.poses[i].pose.position.y - received_path_.poses[closest_idx].pose.position.y;
            if (std::sqrt(dx*dx + dy*dy) > lookahead_dist) {
                target_idx = i;
                break;
            }
        }

        // 如果已经接近终点
        double dist_to_goal = std::sqrt(
            std::pow(received_path_.poses.back().pose.position.x - robot_x_, 2) +
            std::pow(received_path_.poses.back().pose.position.y - robot_y_, 2)
        );

        if (dist_to_goal < 0.1 && closest_idx == received_path_.poses.size() - 1) {
            current_v_ = 0.0;
            current_w_ = 0.0;
            return;
        }

        // 3. Pure Pursuit 计算
        auto target_pose = received_path_.poses[target_idx];
        double tx = target_pose.pose.position.x;
        double ty = target_pose.pose.position.y;
        
        double alpha = std::atan2(ty - robot_y_, tx - robot_x_) - robot_yaw_;
        // Normalize alpha
        while(alpha > M_PI) alpha -= 2*M_PI;
        while(alpha < -M_PI) alpha += 2*M_PI;

        current_v_ = 1.0;//0.5; // 固定线速度
        // Pure pursuit curvature: 2*sin(alpha)/Ld
        // W = v * curvature
        // 简单 P 控制：
        current_w_ = 1.0 * alpha; // 增益需调试

        // 限制
        if (current_v_ > 0.5) current_v_ = 0.5;
        if (current_w_ > 1.0) current_w_ = 1.0;
        if (current_w_ < -1.0) current_w_ = -1.0;
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr car_marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr global_obstacle_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr local_obstacle_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cur_pose_publisher_;
    
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_path_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 仿真状态
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_yaw_ = 0.0;
    double current_v_ = 0.0;
    double current_w_ = 0.0;

    nav_msgs::msg::Path received_path_;
    std::mutex path_mutex_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FakeSimNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}