#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <vector>
#include <string>

using namespace std::chrono_literals;

class TrajectoryPublisher : public rclcpp::Node
{
public:
  TrajectoryPublisher() : Node("trajectory_publisher")
  {
    // 创建一个Publisher，发布话题为 "trajectories"，消息类型为 MarkerArray
    publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("trajectories", 10);
    
    // 创建一个定时器，每500ms发布一次数据
    timer_ = this->create_wall_timer(
      500ms, std::bind(&TrajectoryPublisher::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Trajectory Publisher Node has started.");
  }

private:
  // 模拟生成轨迹数据
  // 在你的实际应用中，你会从传感器、规划器等获取这些数据
  std::vector<std::vector<geometry_msgs::msg::Point>> generate_trajectories()
  {
    std::vector<std::vector<geometry_msgs::msg::Point>> all_trajectories;
    
    // 轨迹1: 方形
    std::vector<geometry_msgs::msg::Point> traj1_points;
    for (int i = 0; i <= 10; ++i) traj1_points.push_back({(double)i, 0.0, 0.0});
    for (int i = 0; i <= 10; ++i) traj1_points.push_back({10.0, (double)i, 0.0});
    for (int i = 10; i >= 0; --i) traj1_points.push_back({(double)i, 10.0, 0.0});
    for (int i = 10; i >= 0; --i) traj1_points.push_back({0.0, (double)i, 0.0});
    all_trajectories.push_back(traj1_points);

    // 轨迹2: 圆形
    std::vector<geometry_msgs::msg::Point> traj2_points;
    for (int i = 0; i <= 100; ++i) {
      double theta = 2 * M_PI * i / 100.0;
      traj2_points.push_back({5.0 + 3.0 * cos(theta), 5.0 + 3.0 * sin(theta), 1.0});
    }
    all_trajectories.push_back(traj2_points);

    // 轨迹3: 三角形
    std::vector<geometry_msgs::msg::Point> traj3_points;
    traj3_points.push_back({2.0, 2.0, 2.0});
    traj3_points.push_back({8.0, 2.0, 2.0});
    traj3_points.push_back({5.0, 8.0, 2.0});
    traj3_points.push_back({2.0, 2.0, 2.0}); // 闭合
    all_trajectories.push_back(traj3_points);
    
    return all_trajectories;
  }

  void timer_callback()
  {
    auto marker_array_msg = visualization_msgs::msg::MarkerArray();
    
    // 1. 获取你的轨迹数据
    std::vector<std::vector<geometry_msgs::msg::Point>> my_trajectories = generate_trajectories();
    
    // 2. 为每条轨迹创建一个Marker
    for (size_t i = 0; i < my_trajectories.size(); ++i)
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map"; // 假设使用 "map" 坐标系
      marker.header.stamp = this->get_clock()->now();
      marker.ns = "trajectories"; // 所有轨迹共享同一个命名空间
      marker.id = i; // 每条轨迹一个唯一的ID
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD; // 每次都重新添加

      // 设置线宽
      marker.scale.x = 0.1;

      // 设置不同的颜色
      std::vector<std_msgs::msg::ColorRGBA> colors = {
        {1.0, 0.0, 0.0, 1.0},  // 红色
        {0.0, 1.0, 0.0, 1.0},  // 绿色
        {0.0, 0.0, 1.0, 1.0}   // 蓝色
      };
      marker.color = colors[i % colors.size()];

      // 设置轨迹点
      marker.points = my_trajectories[i];
      
      // 将单个Marker添加到MarkerArray中
      marker_array_msg.markers.push_back(marker);
    }
    
    // 3. 发布MarkerArray消息
    publisher_->publish(marker_array_msg);
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryPublisher>());
  rclcpp::shutdown();
  return 0;
}
