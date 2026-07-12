/*
 * @Function: Ego Planner + RViz Interactive Input (Global Path + Obstacles)
 * @Create by: juchunyu@qq.com
 * @Date: 2025-11-01 18:58:01
 */
#include "trajectory_obstacles_publisher.h"
#include <tf2/utils.h>
#include "geometry_msgs/msg/quaternion.hpp"  // 确保包含四元数消息类型

TrajectoryAndObstaclesPublisher::TrajectoryAndObstaclesPublisher() 
    : Node("ego_planner_interactive_node"),
      has_valid_global_path_(false),
      has_obstacles_(false),
      should_plan_(true), // 默认允许规划，方便调试
      needs_replan_(false),
      flag_(false) // 初始化 flag_
{
    // 1. 创建发布者
    global_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("visual_global_path", 10);
    a_star_path_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("trajectories", 10);
    local_traj_pub_ = this->create_publisher<nav_msgs::msg::Path>("visual_local_trajectory", 10);
    obs_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("visual_obstacles", 10);
    obs_local_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("visual_local_obstacles", 10);


    // 2. 接收来自仿真器/定位模块的当前位置
    currPose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "current_pose",
            10,
            std::bind(&TrajectoryAndObstaclesPublisher::pose_callback, this, std::placeholders::_1)
        );
        

    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose",
        10,
        std::bind(&TrajectoryAndObstaclesPublisher::goal_pose_callback, this, std::placeholders::_1)
    );

    rviz_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point",
        10,
        std::bind(&TrajectoryAndObstaclesPublisher::rviz_point_callback, this, std::placeholders::_1)
    );

    // 注意：这里不再处理 initialpose 来更新 cur_pose_，
    // 因为位置应该由仿真器决定。规划器只负责接收当前位置。
    // 但为了触发重规划，我们保留订阅，仅用于设置标志位。
    pose_estimate_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose",
        10,
        std::bind(&TrajectoryAndObstaclesPublisher::pose_estimate_callback, this, std::placeholders::_1)
    );

    trigger_plan_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/trigger_plan",
        10,
        std::bind(&TrajectoryAndObstaclesPublisher::trigger_plan_callback, this, std::placeholders::_1)
    );

    init_ego_planner_base();

    // 5Hz 规划与发布循环
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&TrajectoryAndObstaclesPublisher::publish_and_plan, this)
    );

    RCLCPP_INFO(this->get_logger(), "Planner Ready. Waiting for global path points (/clicked_point)...");
}


void TrajectoryAndObstaclesPublisher::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // std::cout << "[pose_callback] cur_pose_ " << msg->x << " ," << msg->y << " ," << msg->theta << std::endl;
    // std::lock_guard<std::mutex> lock(data_mutex_);
    // cur_pose_.x = msg->x;
    // cur_pose_.y = msg->y;
    // cur_pose_.z = msg->theta; // 这里 z 存储 theta

    std::lock_guard<std::mutex> lock(data_mutex_);
    // 1. 更新机器人初始位姿（原有逻辑）
    cur_pose_.x = msg->pose.position.x;
    cur_pose_.y = msg->pose.position.y;
    cur_pose_.z = 0;
    // 计算偏航角（使用tf2或手动计算，确保正确）
    tf2::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);
    cur_pose_.z = tf2::getYaw(q);
    // std::cout << "pose_estimate_callback " << std::endl;

    // 收到位置更新，且有全局路径时，应当触发检查是否需要重规划
    // 这里简化逻辑：只要处于规划模式，就认为数据是最新的
}

void TrajectoryAndObstaclesPublisher::init_ego_planner_base()
{
    ego_planner_ = std::make_shared<PlannerInterface>();
    ego_planner_->initParam(max_vel_, max_acc_, max_jerk_);
    ego_planner_->initEsdfMap(
        map_x_size_, map_y_size_, map_z_size_,
        map_resolution_, map_origin_, map_inflate_value_
    );
}

void TrajectoryAndObstaclesPublisher::add_obstacle_at_position(double x, double y)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    ObstacleInfo obs;
    obs.x = x; obs.y = y; obs.z = 0.0;
    obstacles_.push_back(obs);
    has_obstacles_ = true;
    needs_replan_ = true;
}

void TrajectoryAndObstaclesPublisher::pose_estimate_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    // 仅用于触发重规划，实际位置更新依赖 SimNode 发来的 current_pose
    RCLCPP_INFO(this->get_logger(), "收到重置信号，准备重规划...");
    needs_replan_ = true;
   // 1. 更新机器人初始位姿（原有逻辑）
    cur_pose_.x = msg->pose.pose.position.x;
    cur_pose_.y = msg->pose.pose.position.y;
    cur_pose_.z = 0;
    // 计算偏航角（使用tf2或手动计算，确保正确）
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    cur_pose_.z = tf2::getYaw(q);
}

void TrajectoryAndObstaclesPublisher::trigger_plan_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    should_plan_ = msg->data;
    if(should_plan_) needs_replan_ = true;
    RCLCPP_INFO(this->get_logger(), "规划触发状态: %s", should_plan_ ? "ON" : "OFF");
}

void TrajectoryAndObstaclesPublisher::goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    add_obstacle_at_position(msg->pose.position.x, msg->pose.position.y);
}

void TrajectoryAndObstaclesPublisher::rviz_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    PathPoint point;
    point.x = msg->point.x;
    point.y = msg->point.y;
    point.z = 0; 
    global_plan_traj_.push_back(point);
    
    RCLCPP_INFO(this->get_logger(), "添加全局路径点: (%.2f, %.2f), 总点数: %zu", point.x, point.y, global_plan_traj_.size());
    
    if (global_plan_traj_.size() >= 2) {
        has_valid_global_path_ = true;
        needs_replan_ = true;
    }
}

void TrajectoryAndObstaclesPublisher::publish_and_plan()
{
    // 必须加锁，因为 cur_pose_ 在回调中更新
    // std::lock_guard<std::mutex> lock(data_mutex_);

    if (!has_valid_global_path_ || global_plan_traj_.empty()) {
        publish_global_path(); // 即使空的也发布，保持RViz状态
        return;
    }

    // 简单策略：总是尝试规划，或者根据 needs_replan_
    // 为了演示流畅性，这里只要允许规划就一直运行
    {
            std::vector<ObstacleInfo> local_obstacles;

            std::lock_guard<std::mutex> lock(data_mutex_); // 获取锁
            std::vector<ObstacleInfo> local_obstacles_res;
            // 只有当有路径且允许规划时，才拷贝数据
            local_pose = cur_pose_;
            local_obstacles = obstacles_; // 拷贝障碍物列表
            for(int i = 0; i < local_obstacles.size();i++)
            {
                double dist =  sqrt(pow(local_obstacles[i].x - local_pose.x,2) + pow(local_obstacles[i].y - local_pose.y,2));
                // distance(local_obstacles[i], local_pose); 
                if(dist < 10)
                {
                    local_obstacles_res.push_back(local_obstacles[i]);
                }
            }
            ego_planner_->resetMap();
            ego_planner_->setGridMap(local_pose);
            ego_planner_->setObstacles(local_obstacles_res);
    } 



    if (collisionDetection(planned_traj) || (planned_traj.size() < 10) || needs_replan_) 
    {
        std::cout << "replan..." << std::endl;
        needs_replan_ = false;
        std::cout << "[publish_and_plan] cur_pose_.x  = " << cur_pose_.x << " cur_pose_.y =" << cur_pose_.y << std::endl;
        a_star_pathes_.clear();

        std::vector<ObstacleInfo> local_obstacles;
        {
            std::lock_guard<std::mutex> lock(data_mutex_); // 获取锁

            // 只有当有路径且允许规划时，才拷贝数据
            local_pose = cur_pose_;
            local_obstacles = obstacles_; // 拷贝障碍物列表
        } // <--- 锁在这里自动释放！后续
        // 设置全局路径
        if (!global_plan_traj_.empty())
        {
            std::vector<PathPoint> global_plan_traj_temp;

            discretize_trajectory(global_plan_traj_, global_plan_traj_temp, 0.3);
             
            float mindist = 100000000;
            int minddex = 0;
            for(int i = 0; i < global_plan_traj_temp.size();i++)
            {
                double dist =  distance(global_plan_traj_temp[i], local_pose); 
                if(dist < mindist)
                {
                    mindist = dist;
                    minddex = i;
                } 
            }

            std::vector<PathPoint> global_plan_traj_after;
            global_plan_traj_after.push_back(local_pose);
            for(int i = minddex; i < global_plan_traj_temp.size();i++)
            {
                global_plan_traj_after.push_back(global_plan_traj_temp[i]);
            }
            discretize_trajectory(global_plan_traj_after, global_plan_traj_temp, 0.3);
            
            int length = 50;
            if(global_plan_traj_temp.size() < length) length = global_plan_traj_temp.size();
            global_plan_traj_res_.clear();
            for(int i = 0; i < length;i++)
            {
                global_plan_traj_res_.push_back(global_plan_traj_temp[i]);
            }
            // ego_planner_->setPathPoint(global_plan_traj_res_);
        }
        ego_planner_->setPathPoint(global_plan_traj_res_);

        {
            ego_planner_->resetMap();
            ego_planner_->setGridMap(local_pose);
            flag_ = true;
        }
        
        ego_planner_->setCurrentVehiclePos(local_pose);
        
        std::vector<ObstacleInfo> local_obstacles_res;
        for(int i = 0; i < local_obstacles.size();i++)
        {
            double dist =  sqrt(pow(local_obstacles[i].x - local_pose.x,2) + pow(local_obstacles[i].y - local_pose.y,2));
            // distance(local_obstacles[i], local_pose); 
            if(dist < 10)
            {
                local_obstacles_res.push_back(local_obstacles[i]);
            }
        }
        ego_planner_->setObstacles(local_obstacles_res);
        // 3. 执行规划
        // if(global_plan_traj_res_.size() > 5)
        std::cout << "global_plan_traj_res_.size()  =" << global_plan_traj_res_.size()  << std::endl;
        // if(global_plan_traj_res_.size() > 5 && !global_plan_traj_.empty())
        {
            ego_planner_->makePlan();
            ego_planner_->getAStarPath(a_star_pathes_);
        }

        //if (ego_planner_->makePlan()) {
            // ego_planner_->getAStarPath(a_star_pathes_);
            // 规划成功后，needs_replan_ 可以置 false，
            // 但如果是连续控制，通常每一帧都规划。
        needs_replan_ = false; 
        // } else {
        //     RCLCPP_WARN(this->get_logger(), "规划失败!");
        // }
    }

    // 发布数据
    publish_global_path();
    publish_planned_trajectory();
    publish_obstacles();
    publish_a_star_path();
    publish_local_obstacles();
}

bool TrajectoryAndObstaclesPublisher::collisionDetection(std::vector<PathPoint>& planned_traj)
{
    for(int i = 0; i < planned_traj.size();i++)
    {
        bool status = ego_planner_->getInflateOccupancy(planned_traj[i]);
        if(status)
        {
            std::cout << "[collisionDetection] ok=" << std::endl; 
            return true;
        }
    }

    return false; 
}

// 发布可视化全局路径
void TrajectoryAndObstaclesPublisher::publish_global_path()
{
    // if (global_plan_traj_.empty()) return;

    nav_msgs::msg::Path visual_path;
    visual_path.header.stamp = this->now();
    visual_path.header.frame_id = "map";

    for (const auto& path_point : global_plan_traj_)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = visual_path.header;
        pose.pose.position.x = path_point.x;
        pose.pose.position.y = path_point.y;
        pose.pose.position.z = path_point.z;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        visual_path.poses.push_back(pose);
    }

    global_path_pub_->publish(visual_path);
}

void TrajectoryAndObstaclesPublisher::publish_a_star_path()
{
    // 如果没有路径数据，直接返回
    if (a_star_pathes_.empty())
    {
    //   RCLCPP_WARN(this->get_logger(), "a_star_pathes_ is empty, no trajectories to publish.");
      return;
    }

    // 创建一个MarkerArray消息
    visualization_msgs::msg::MarkerArray marker_array_msg;

    // 遍历 a_star_pathes_ 中的每一条路径
    for (size_t i = 0; i < a_star_pathes_.size(); ++i)
    {
      const std::vector<Eigen::Vector2d>& current_path = a_star_pathes_[i];

      // 1. 创建一个Marker对象
      visualization_msgs::msg::Marker marker;

      // 2. 设置Marker的基本信息
      marker.header.frame_id = "map"; // 非常重要！指定轨迹所在的坐标系
      marker.header.stamp = this->get_clock()->now();
      marker.ns = "a_star_paths"; // 命名空间，用于分组管理Marker
      marker.id = i;              // 每条路径必须有唯一的ID
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP; // 类型为线串
      marker.action = visualization_msgs::msg::Marker::ADD;      // 动作是添加

      // 3. 设置线的视觉属性
      marker.scale.x = 0.1; // 线宽为0.1米

      // 设置颜色（RGBA格式），每条路径可以设置不同的颜色
      marker.color.r = 1.0 - (i * 0.3); // 红色分量
      marker.color.g = 0.2;             // 绿色分量
      marker.color.b = 0.2 + (i * 0.3); // 蓝色分量
      marker.color.a = 1.0;             // 透明度（1.0为完全不透明）

      // 4. 填充路径点
      for (const auto& eigen_point : current_path)
      {
        geometry_msgs::msg::Point ros_point;
        ros_point.x = eigen_point.x();
        ros_point.y = eigen_point.y();
        ros_point.z = 0.0; // 2D轨迹，z轴设为0

        marker.points.push_back(ros_point);
      }

      // 5. 将当前路径的Marker添加到MarkerArray中
      marker_array_msg.markers.push_back(marker);
    }

    // 6. 发布MarkerArray消息
    a_star_path_pub_->publish(marker_array_msg);
    // RCLCPP_INFO(this->get_logger(), "a_star_path_pub_ Published %zu trajectories.", a_star_pathes_.size());

}

// 发布Ego Planner规划后的局部轨迹
void TrajectoryAndObstaclesPublisher::publish_planned_trajectory()
{
    std::cout << "发布Ego Planner规划后的局部轨迹" << std::endl;
    planned_traj.clear();
    std::cout << "global_plan_traj_res_.size()  =" << global_plan_traj_res_.size()  << std::endl;
    // if(global_plan_traj_res_.size() > 5)
    {
        ego_planner_->getLocalPlanTrajResults(planned_traj);
        std::cout << "planned start x =  " << planned_traj[0].x << " , start y =" <<  planned_traj[0].y << std::endl;
    }
    // else
    // { 
    //     std::cout << "use global path !!!" << std::endl;
    //     planned_traj = global_plan_traj_res_;
    // }
    
   
    // if (planned_traj.empty()) return;

    nav_msgs::msg::Path visual_traj;
    visual_traj.header.stamp = this->now();
    visual_traj.header.frame_id = "map";

    for (size_t i = 0; i < planned_traj.size(); ++i)
    {
        // std::cout << "[publish_planned_trajectory] x = " << planned_traj[i].x << " y =" << planned_traj[i].y << std::endl;
        geometry_msgs::msg::PoseStamped pose;
        pose.header = visual_traj.header;
        pose.pose.position.x = planned_traj[i].x;
        pose.pose.position.y = planned_traj[i].y;
        pose.pose.position.z = planned_traj[i].z;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        visual_traj.poses.push_back(pose);
    }

    local_traj_pub_->publish(visual_traj);
}

// 发布可视化障碍物
// void TrajectoryAndObstaclesPublisher::publish_obstacles()
// {
//     // if (obstacles_.empty()) return;

//     sensor_msgs::msg::PointCloud2 visual_obs;
//     visual_obs.header.stamp = this->now();
//     visual_obs.header.frame_id = "map";
//     visual_obs.width = obstacles_.size();
//     visual_obs.height = 1;
//     visual_obs.is_dense = true;

//     sensor_msgs::PointCloud2Modifier modifier(visual_obs);
//     modifier.setPointCloud2FieldsByString(1, "xyz");

//     sensor_msgs::PointCloud2Iterator<float> iter_x(visual_obs, "x");
//     sensor_msgs::PointCloud2Iterator<float> iter_y(visual_obs, "y");
//     sensor_msgs::PointCloud2Iterator<float> iter_z(visual_obs, "z");

//     for (const auto& obs : obstacles_)
//     {
//         *iter_x = static_cast<float>(obs.x);
//         *iter_y = static_cast<float>(obs.y);
//         *iter_z = 0.0f;
//         ++iter_x;
//         ++iter_y;
//         ++iter_z;
//     }

//     obs_pub_->publish(visual_obs);
// }

void TrajectoryAndObstaclesPublisher::publish_obstacles()
{
    sensor_msgs::msg::PointCloud2 visual_obs;
    visual_obs.header.stamp = this->now();
    visual_obs.header.frame_id = "map";
    visual_obs.height = 1;
    visual_obs.width = obstacles_.size();
    visual_obs.is_dense = true;

    // 必须设置字段
    sensor_msgs::PointCloud2Modifier modifier(visual_obs);
    modifier.setPointCloud2FieldsByString(1, "xyz"); // x,y,z float32

    // 必须告诉 ROS2 每个点占 12 字节
    modifier.resize(obstacles_.size());

    // 之后 Iterator 才能写数据
    sensor_msgs::PointCloud2Iterator<float> iter_x(visual_obs, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(visual_obs, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(visual_obs, "z");

    for (const auto& obs : obstacles_)
    {
        *iter_x = static_cast<float>(obs.x);
        *iter_y = static_cast<float>(obs.y);
        *iter_z = 0.0f;

        ++iter_x;
        ++iter_y;
        ++iter_z;
    }

    // 发布（不会再崩）
    obs_pub_->publish(visual_obs);
}


// void TrajectoryAndObstaclesPublisher::publish_local_obstacles()
// {
//     // if (obstacles_.empty()) return;

//     sensor_msgs::msg::PointCloud2 visual_obs;
//     visual_obs.header.stamp = this->now();
//     visual_obs.header.frame_id = "map";
//     visual_obs.width = obstacles_.size();
//     visual_obs.height = 1;
//     visual_obs.is_dense = true;

//     sensor_msgs::PointCloud2Modifier modifier(visual_obs);
//     modifier.setPointCloud2FieldsByString(1, "xyz");

//     sensor_msgs::PointCloud2Iterator<float> iter_x(visual_obs, "x");
//     sensor_msgs::PointCloud2Iterator<float> iter_y(visual_obs, "y");
//     sensor_msgs::PointCloud2Iterator<float> iter_z(visual_obs, "z");

//     std::vector<ObstacleInfo> obstacles;
//     ego_planner_->getObstacles(obstacles);

//     for (const auto& obs : obstacles)
//     {
//         *iter_x = static_cast<float>(obs.x);
//         *iter_y = static_cast<float>(obs.y);
//         *iter_z = 0.0f;
//         ++iter_x;
//         ++iter_y;
//         ++iter_z;
//     }

//     obs_local_pub_->publish(visual_obs);
// }

void TrajectoryAndObstaclesPublisher::publish_local_obstacles()
{
    std::vector<ObstacleInfo> obstacles;
    ego_planner_->getObstacles(obstacles);

    if (obstacles.empty()) return;

    sensor_msgs::msg::PointCloud2 visual_obs;
    visual_obs.header.stamp = this->now();
    visual_obs.header.frame_id = "map";

    // 正确设置点云大小
    visual_obs.height = 1;
    visual_obs.width = obstacles.size();
    visual_obs.is_dense = true;

    // 为点云分配字段
    sensor_msgs::PointCloud2Modifier modifier(visual_obs);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(obstacles.size());   // 关键！分配空间

    // 创建迭代器
    sensor_msgs::PointCloud2Iterator<float> iter_x(visual_obs, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(visual_obs, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(visual_obs, "z");

    for (const auto& obs : obstacles)
    {
        *iter_x = static_cast<float>(obs.x);
        *iter_y = static_cast<float>(obs.y);
        *iter_z = 0.0f;

        ++iter_x;
        ++iter_y;
        ++iter_z;
    }

    obs_local_pub_->publish(visual_obs);
}


// 计算两点之间的欧氏距离（单位：米）
double TrajectoryAndObstaclesPublisher::distance(const PathPoint& p1, const PathPoint& p2) {
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    return std::sqrt(dx*dx + dy*dy);
}

/**
 * 将轨迹离散为均匀间隔的点（间隔10cm）
 * @param original_trajectory 原始轨迹（由多个顶点组成的折线）
 * @param discrete_trajectory 输出的离散轨迹
 * @param interval 间隔距离（单位：米，默认0.1米即10cm）
 */
void TrajectoryAndObstaclesPublisher::discretize_trajectory(const std::vector<PathPoint>& original_trajectory,
                                                            std::vector<PathPoint>& discrete_trajectory,
                                                            double interval) {
    if (original_trajectory.size() < 2) {
        std::cerr << "原始轨迹至少需要2个点！" << std::endl;
        return;
    }

    discrete_trajectory.clear();
    // 添加轨迹起点
    // discrete_trajectory.push_back(cur_pose_);
    discrete_trajectory.push_back(original_trajectory[0]);

    // 遍历原始轨迹的每一段线段
    for (size_t i = 0; i < original_trajectory.size() - 1; ++i) {
        const PathPoint& start = original_trajectory[i];
        const PathPoint& end = original_trajectory[i+1];
        double seg_length = distance(start, end);  // 线段总长度

        if (seg_length < 1e-6) {  // 跳过长度接近0的线段（避免除零）
            continue;
        }

        // 计算当前线段需要插入的点数（不含起点，含终点）
        int num_points = static_cast<int>(seg_length / interval);
        // 最后一个点到终点的距离（避免累积误差）
        double last_interval = seg_length - num_points * interval;

        // 生成线段上的离散点
        for (int j = 1; j <= num_points; ++j) {
            double ratio;
            if (j < num_points) {
                // 前num_points-1个点：按均匀间隔计算
                ratio = (j * interval) / seg_length;
            } else {
                // 最后一个点：直接对齐到线段终点（避免累积误差）
                ratio = 1.0;
            }

            // 线性插值计算点坐标
            PathPoint p;
            p.x = start.x + ratio * (end.x - start.x);
            p.y = start.y + ratio * (end.y - start.y);
            discrete_trajectory.push_back(p);
        }
    }
}

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<TrajectoryAndObstaclesPublisher>();

//     rclcpp::executors::MultiThreadedExecutor executor;
//     executor.add_node(node);
//     executor.spin();

//     rclcpp::shutdown();
//     return 0;
// }

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryAndObstaclesPublisher>();

    // 【强制修改】使用默认的单线程执行器
    // 这能保证回调函数 PoseCallback 和 TimerCallback 永远不会同时运行
    // 彻底根除锁竞争导致的随机崩溃
    rclcpp::spin(node); 

    rclcpp::shutdown();
    return 0;
}
