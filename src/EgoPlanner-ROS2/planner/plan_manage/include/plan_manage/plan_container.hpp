#ifndef _PLAN_CONTAINER_H_
#define _PLAN_CONTAINER_H_

#include <Eigen/Eigen>
#include <vector>
#include <bspline_opt/uniform_bspline.h>

using std::vector;

namespace ego_planner
{


  struct PlanParameters
  {
    /* planning algorithm parameters */
    double max_vel_, max_acc_, max_jerk_; // physical limits
    double ctrl_pt_dist;                  // distance between adjacient B-spline control points
    double feasibility_tolerance_;        // permitted ratio of vel/acc exceeding limits
    double planning_horizen_;

    /* processing time */
    double time_search_ = 0.0;
    double time_optimize_ = 0.0;
    double time_adjust_ = 0.0;
  };

  struct LocalTrajData
  {
    /* info of generated traj */

    int traj_id_;
    double duration_;
    double global_time_offset; // This is because when the local traj finished and is going to switch back to the global traj, the global traj time is no longer matches the world time.
    //ros::Time start_time_;
    Eigen::Vector3d start_pos_;
    UniformBspline position_traj_, velocity_traj_, acceleration_traj_;
  };

} // namespace ego_planner

#endif