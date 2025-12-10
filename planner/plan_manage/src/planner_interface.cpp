#include "planner_interface.h"

namespace ego_planner
{

    PlannerInterface::PlannerInterface()
    {

    }

    PlannerInterface::~PlannerInterface()
    {

    }

    void PlannerInterface::initParam(double max_vel,double max_acc,double max_jerk)
    {
        pp_.max_vel_ = max_vel;
        pp_.max_acc_ = max_acc;
        pp_.max_jerk_ = max_jerk;
        pp_.feasibility_tolerance_ = 0.05;
        pp_.ctrl_pt_dist = 0.4;
        pp_.planning_horizen_ = 5.0;
    }
    
    void PlannerInterface::initEsdfMap(double x_size,double y_size,double z_size,double resolution, Eigen::Vector3d origin,double inflate_values)
    {
        std::cout << "x_size =" << x_size << " y_size =" << y_size << " z_size" << z_size << std::endl;
        std::cout << "resolution =" << resolution << std::endl;
        std::cout << "origin =" << origin << std::endl;
        std::cout << "inflate_values =" << inflate_values << std::endl;
        //初始化gridmap地图
        Eigen::Vector2i map_size(x_size,y_size);     
        grid_map_     = std::make_shared<GridMap2D>(resolution,map_size);
        grid_map_->setInflateRadius(inflate_values);

        bspline_optimizer_rebound_.reset(new BsplineOptimizer);
        bspline_optimizer_rebound_->setParam();
        bspline_optimizer_rebound_->setEnvironment(grid_map_);
        bspline_optimizer_rebound_->a_star_.reset(new AStar);
        bspline_optimizer_rebound_->a_star_->initGridMap(grid_map_, Eigen::Vector2i(100, 100));
       
    }

    void PlannerInterface::setPathPoint(std::vector<PathPoint> &plan_traj)
    {
        _global_plan_traj_.clear();
        _global_plan_traj_ = plan_traj;
    }
    
    void PlannerInterface::setObstacles(std::vector<ObstacleInfo> &obstacle)
    {
        for(int i = 0; i < obstacle.size();i++)
        {
            Eigen::Vector2d coord(obstacle[i].x,obstacle[i].y);
            Eigen::Vector2i res;
            res = grid_map_->worldToGrid(coord);
            grid_map_->setObstacle(res, true);
        }

        grid_map_->inflate();

        // for(int i = 0; i < obstacle.size();i++)
        // {
        //     Eigen::Vector3d pos;
        //     pos[0] = obstacle[i].x;
        //     pos[1] = obstacle[i].y;
        //     pos[2] = 1;
        //     grid_map_->getInflateOccupancy(pos);
        //     std::cout << "grid_map_->getInflateOccupancy(pos); =pos " << pos << " ," << grid_map_->getInflateOccupancy(pos) << std::endl;
        // }
       

    }

    void PlannerInterface::setCurrentVehiclePos(PathPoint& cur_pose)
    {
        cur_pose_ = cur_pose;
        // grid_map_->resetGrids();
    }  

    void PlannerInterface::setGridMap(PathPoint& cur_pose)
    {
        grid_map_->setCurPose(cur_pose.x,cur_pose.y); // only once
    }

    void PlannerInterface::getObstacles(std::vector<ObstacleInfo> &obstacle)
    {
        std::vector<Eigen::Vector2d> inflated_cloud = grid_map_->getObstaclePointCloud();

        for (const auto& point : inflated_cloud)
        {
            ObstacleInfo temp;
            temp.x = static_cast<float>(point.x()); // 解析 x
            temp.y = static_cast<float>(point.y()); // 解析 y
            obstacle.push_back(temp);
        }



        //         // 1. 获取膨胀后的障碍物点云（默认）
        // std::vector<Eigen::Vector2d> inflated_cloud = grid_map.getObstaclePointCloud();

        // // 2. 获取原始障碍物点云
        // std::vector<Eigen::Vector2d> raw_cloud = grid_map.getObstaclePointCloud(false);
    }


    void PlannerInterface::makePlan()
    {
        std::cout << "开始规划..." << std::endl;
        Eigen::Vector3d start_pt;
        Eigen::Vector3d start_vel;
        Eigen::Vector3d start_acc;
        Eigen::Vector3d local_target_pt;
        Eigen::Vector3d local_target_vel;
        vector<Eigen::Vector3d> point_set;


        vector<Eigen::Vector3d> traj_pts;

        for(int i = 0; i< _global_plan_traj_.size();i++)
        {
            Eigen::Vector3d plan_pt(_global_plan_traj_[i].x,_global_plan_traj_[i].y,0.2);
            point_set.push_back(plan_pt);
        }

        start_pt[0] = _global_plan_traj_[0].x;
        start_pt[1] = _global_plan_traj_[0].y;
        start_pt[2] = 0.0;
        
        local_target_pt[0] = _global_plan_traj_[_global_plan_traj_.size()-1].x;
        local_target_pt[1] = _global_plan_traj_[_global_plan_traj_.size()-1].y;
        local_target_pt[2] = 0;

        start_vel[0] = cos(cur_pose_.z);  //根据实际需求修改接入
        start_vel[1] = sin(cur_pose_.z);
        start_vel[2] = 0;

        local_target_vel[0] = 0;//根据实际需求修改接入
        local_target_vel[1] = 0;
        local_target_vel[2] = 0;

        start_acc[0] =  cos(cur_pose_.z);   //根据实际需求修改接入
        start_acc[1] =  sin(cur_pose_.z);
        start_acc[2] = 0;


        auto start = std::chrono::system_clock::now();

        bool plan_success = reboundReplan(start_pt,start_vel, start_acc,local_target_pt,local_target_vel, point_set);
        if (plan_success)
           getTraj();
           
        auto end = std::chrono::system_clock::now();

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        printf("MotionPlanner Total Running Time: %d  ms \n", elapsed.count());
    }

    void PlannerInterface::getLocalPlanTrajResults(std::vector<PathPoint> &plan_traj_results)
    {
        plan_traj_results = _plan_traj_results_;
    }
    
    void PlannerInterface::getAStarPath(vector<vector<Eigen::Vector2d>>& a_star_path)
    {
        a_star_path = a_star_pathes_;

    }


    bool PlannerInterface::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel,vector<Eigen::Vector3d> point_set)
    {
        vector<Eigen::Vector3d> start_end_derivatives;
        double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
        start_end_derivatives.push_back(start_vel);
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(start_acc);
        start_end_derivatives.push_back(start_acc);

        Eigen::MatrixXd ctrl_pts_3d;
        UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts_3d);
         
        Eigen::MatrixXd ctrl_pts_2d;

        ctrl_pts_2d = ctrl_pts_3d.topRows(2);

        a_star_pathes_ = bspline_optimizer_rebound_->initControlPoints(ctrl_pts_2d, true);
        
        static int vis_id = 0;
        
        /*** STEP 2: OPTIMIZE ***/
        bool flag_step_1_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts_2d, ts);
        cout << "first_optimize_step_success flag=" << flag_step_1_success << endl;
        if (!flag_step_1_success)
        {
            std::cout << "轨迹优化器优化失败！"  << std::endl;
            continous_failures_count_++;
            return false;
        }
        else 
        {
            std::cout << "轨迹优化器优化成功！"  << std::endl;
        }

        ctrl_pts_3d = bspline_optimizer_rebound_->convert2DTo3D(ctrl_pts_2d);
        /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
        UniformBspline pos = UniformBspline(ctrl_pts_3d, 3, ts);
        pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

        double ratio;
        bool flag_step_2_success = true;
        if (!pos.checkFeasibility(ratio, false))
        {
            cout << "Need to reallocate time." << endl;

            Eigen::MatrixXd optimal_control_points;
            flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
            if (flag_step_2_success)
                pos = UniformBspline(optimal_control_points, 3, ts);
        }

        if (!flag_step_2_success)
        {
            printf("\033[34mThis refined trajectory hits obstacles. It doesn't matter if appeares occasionally. But if continously appearing, Increase parameter \"lambda_fitness\".\n\033[0m");
            continous_failures_count_++;
            // return false;
        }
    
        updateTrajInfo(pos);

        continous_failures_count_ = 0;

        return true;
    }

    bool PlannerInterface::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
    {

        Eigen::MatrixXd optimal_control_point_temp;
        double t_inc;

        Eigen::MatrixXd ctrl_pts; 

        reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

        Eigen::MatrixXd ctrl_pts_2d;

        ctrl_pts_2d = ctrl_pts.topRows(2);

        traj = UniformBspline(ctrl_pts, 3, ts);

        

        double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);

        bspline_optimizer_rebound_->ref_pts_.clear();

        for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
            bspline_optimizer_rebound_->ref_pts_.push_back(traj.evaluateDeBoorT(t).topRows(2));

        bool success = bspline_optimizer_rebound_->BsplineOptimizeTrajRefine(ctrl_pts_2d, ts, optimal_control_point_temp);

        optimal_control_points = bspline_optimizer_rebound_->convert2DTo3D(ctrl_pts_2d);
         
        return success;
    }

    void PlannerInterface::updateTrajInfo(const UniformBspline &position_traj)
    {
        local_data_.position_traj_ = position_traj;
        local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
        local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
        local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
        local_data_.duration_ = local_data_.position_traj_.getTimeSum();
        local_data_.traj_id_ += 1;
    }

    void PlannerInterface::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                            Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
    {
        double time_origin = bspline.getTimeSum();
        int seg_num = bspline.getControlPoint().cols() - 3;

        bspline.lengthenTime(ratio);
        double duration = bspline.getTimeSum();
        dt = duration / double(seg_num);
        time_inc = duration - time_origin;

        vector<Eigen::Vector3d> point_set;
        for (double time = 0.0; time <= duration + 1e-4; time += dt)
        {
            point_set.push_back(bspline.evaluateDeBoorT(time));
        }

        UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
    }

    void PlannerInterface::getTraj()
    {
        auto info = &local_data_;

        ego_planner::Bspline bspline;
        bspline.order = 3;
        bspline.traj_id = info->traj_id_;

        Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
        bspline.pos_pts.reserve(pos_pts.cols());

        for (int i = 0; i < pos_pts.cols(); ++i)
        {
            geometry_msgs::Point pt;
            pt.x = pos_pts(0, i);
            pt.y = pos_pts(1, i);
            pt.z = pos_pts(2, i);
            bspline.pos_pts.push_back(pt);
        }

        Eigen::VectorXd knots = info->position_traj_.getKnot();
        bspline.knots.reserve(knots.rows());

        for (int i = 0; i < knots.rows(); ++i)
        {
            bspline.knots.push_back(knots(i));
        }

        vector<ego_planner::UniformBspline> traj_;
        double traj_duration_;

        ego_planner::UniformBspline pos_traj(pos_pts, bspline.order, 0.1);
        pos_traj.setKnot(knots);
        traj_.clear();
        traj_.push_back(pos_traj);
        traj_.push_back(traj_[0].getDerivative());
        traj_.push_back(traj_[1].getDerivative());
        traj_duration_ = traj_[0].getTimeSum();


        Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
        _plan_traj_results_.clear();
        for (double t_cur = 0; t_cur <= traj_duration_; t_cur += 0.1) 
        {
            pos = traj_[0].evaluateDeBoorT(t_cur);
            vel = traj_[1].evaluateDeBoorT(t_cur);
            acc = traj_[2].evaluateDeBoorT(t_cur);
            PathPoint tempPath;
            tempPath.x = pos(0);
            tempPath.y = pos(1);
            _plan_traj_results_.push_back(tempPath);
        }
    }

}