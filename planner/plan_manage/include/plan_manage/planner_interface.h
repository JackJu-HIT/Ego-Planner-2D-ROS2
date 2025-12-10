/*
 * @Function:Ego Planner:Trajectory Optimize Base Bspline
 * @Create by:juchunyu@qq.com
 * @Date:2025-08-02 17:40:01
 */

#ifndef _PLANNER_INTERFACE_H_
#define _PLANNER_INTERFACE_H_

#include <stdlib.h>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <plan_env/grid_map.h>
#include <plan_manage/plan_container.hpp>
#include "Bspline.h"
#include <chrono>


namespace  ego_planner
{
    struct PathPoint
    {
        float x;
        float y;
        float z;
        float v;
    };

    struct ObstacleInfo
    {
        float x;
        float y;
        float z;
    };

    class PlannerInterface
    {
           
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                                Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                                Eigen::Vector3d local_target_vel,vector<Eigen::Vector3d> point_set);
            PlanParameters pp_;
            LocalTrajData local_data_;
            shared_ptr<GridMap2D> grid_map_;
            PathPoint  cur_pose_;

        private:

            BsplineOptimizer::Ptr bspline_optimizer_rebound_;

            int continous_failures_count_{0};

            void updateTrajInfo(const UniformBspline &position_traj);

            void reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
                            double &time_inc);

            bool refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);
   
            
        private:

           
            std::vector<PathPoint> _global_plan_traj_;
            std::vector<PathPoint> _plan_traj_results_;
            vector<vector<Eigen::Vector2d>> a_star_pathes_;



        public:
            PlannerInterface();

            ~PlannerInterface();

            void initParam(double max_vel,double max_acc,double max_jerk);

            void initEsdfMap(double x_size,double y_size,double z_size,double resolution, Eigen::Vector3d org,double inflate_values);

            void setPathPoint(std::vector<PathPoint> &plan_traj);

            void setObstacles(std::vector<ObstacleInfo> &obstacle);

            void setCurrentVehiclePos(PathPoint& cur_pose);

            void makePlan();

            void getLocalPlanTrajResults(std::vector<PathPoint> &plan_traj_results);  

            void getObstacles(std::vector<ObstacleInfo> &obstacle);

            void getTraj();

            void setGridMap(PathPoint& cur_pose);

            void getAStarPath(vector<vector<Eigen::Vector2d>>& a_star_path);

    };


}


#endif
