#include "bspline_opt/bspline_optimizer.h"
#include "bspline_opt/gradient_descent_optimizer.h"

namespace ego_planner
{

void BsplineOptimizer::setParam()
{

    lambda1_ = 12.0;    // 平滑 - 关键
    lambda2_ = 1.0;     // 碰撞
    lambda3_ = 3.0;     // feasibility
    lambda4_ = 1.0;
    lambda5_ = 0.1;

    dist0_   = 0.6;

    max_vel_ = 1.0;
    max_acc_ = 0.5;

    order_   = 3;

}

void BsplineOptimizer::setEnvironment(const shared_ptr<GridMap2D> &env)
{
  this->grid_map_ = env;
}

void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd &points)
{
  cps_.points = points;
}

void BsplineOptimizer::setBsplineInterval(const double &ts) { bspline_interval_ = ts; }

std::vector<std::vector<Eigen::Vector2d>> BsplineOptimizer::initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init /*= true*/)
{

    if (flag_first_init)
    {
      cps_.clearance = dist0_;
      cps_.resize(init_points.cols());
      cps_.points = init_points;
    }

    /*** Segment the initial trajectory according to obstacles ***/
    constexpr int ENOUGH_INTERVAL = 2;
    double step_size = grid_map_->getResolution() / ((init_points.col(0) - init_points.rightCols(1)).norm() / (init_points.cols() - 1)) / 2;
    int in_id, out_id;
    vector<std::pair<int, int>> segment_ids;
    int same_occ_state_times = ENOUGH_INTERVAL + 1;
    bool occ, last_occ = false;
    bool flag_got_start = false, flag_got_end = false, flag_got_end_maybe = false;
    int i_end = (int)init_points.cols() - order_ - ((int)init_points.cols() - 2 * order_) / 3; // only check closed 2/3 points.
    for (int i = order_; i <= i_end; ++i)
    {
      for (double a = 1.0; a >= 0.0; a -= step_size)
      {
        occ = grid_map_->getInflateOccupancy(a * init_points.col(i - 1) + (1 - a) * init_points.col(i));
        // cout << setprecision(5);
        // cout << (a * init_points.col(i-1) + (1-a) * init_points.col(i)).transpose() << " occ1=" << occ << endl;

        if (occ && !last_occ)
        {
          if (same_occ_state_times > ENOUGH_INTERVAL || i == order_)
          {
            in_id = i - 1;
            flag_got_start = true;
          }
          same_occ_state_times = 0;
          flag_got_end_maybe = false; // terminate in advance
        }
        else if (!occ && last_occ)
        {
          out_id = i;
          flag_got_end_maybe = true;
          same_occ_state_times = 0;
        }
        else
        {
          ++same_occ_state_times;
        }

        if (flag_got_end_maybe && (same_occ_state_times > ENOUGH_INTERVAL || (i == (int)init_points.cols() - order_)))
        {
          flag_got_end_maybe = false;
          flag_got_end = true;
        }

        last_occ = occ;

        if (flag_got_start && flag_got_end)
        {
          flag_got_start = false;
          flag_got_end = false;
          segment_ids.push_back(std::pair<int, int>(in_id, out_id));
        }
      }
    }

    // for (size_t i = 0; i < segment_ids.size(); ++i)
    // {
    //   cout << "in=" << init_points.col(segment_ids[i].first) << " out=" <<init_points.col(segment_ids[i].second) << endl;
    // }
    /*** a star search ***/
    vector<vector<Eigen::Vector2d>> a_star_pathes;
    for (size_t i = 0; i < segment_ids.size(); ++i)
    {
      // cout << "in=" << in.transpose() << " out=" << out.transpose() << endl;
      Eigen::Vector2d in(init_points.col(segment_ids[i].first)), out(init_points.col(segment_ids[i].second));
      if (a_star_->AstarSearch(/*(in-out).norm()/10+0.05*/ 0.1, in, out))
      {
        a_star_pathes.push_back(a_star_->getPath());
      }
      else
      {
        printf("a star error, force return!");
        return a_star_pathes;
      }
    }

    /*** calculate bounds ***/
    int id_low_bound, id_up_bound;
    vector<std::pair<int, int>> bounds(segment_ids.size());
    for (size_t i = 0; i < segment_ids.size(); i++)
    {

      if (i == 0) // first segment
      {
        id_low_bound = order_;
        if (segment_ids.size() > 1)
        {
          id_up_bound = (int)(((segment_ids[0].second + segment_ids[1].first) - 1.0f) / 2); // id_up_bound : -1.0f fix()
        }
        else
        {
          id_up_bound = init_points.cols() - order_ - 1;
        }
      }
      else if (i == segment_ids.size() - 1) // last segment, i != 0 here
      {
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        id_up_bound = init_points.cols() - order_ - 1;
      }
      else
      {
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        id_up_bound = (int)(((segment_ids[i].second + segment_ids[i + 1].first) - 1.0f) / 2);  // id_up_bound : -1.0f fix()
      }

      bounds[i] = std::pair<int, int>(id_low_bound, id_up_bound);
    }

    // cout << "+++++++++" << endl;
    // for ( int j=0; j<bounds.size(); ++j )
    // {
    //   cout << bounds[j].first << "  " << bounds[j].second << endl;
    // }

    /*** Adjust segment length ***/
    vector<std::pair<int, int>> final_segment_ids(segment_ids.size());
    constexpr double MINIMUM_PERCENT = 0.0; // Each segment is guaranteed to have sufficient points to generate sufficient thrust
    int minimum_points = round(init_points.cols() * MINIMUM_PERCENT), num_points;
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      /*** Adjust segment length ***/
      num_points = segment_ids[i].second - segment_ids[i].first + 1;
      //cout << "i = " << i << " first = " << segment_ids[i].first << " second = " << segment_ids[i].second << endl;
      if (num_points < minimum_points)
      {
        double add_points_each_side = (int)(((minimum_points - num_points) + 1.0f) / 2);

        final_segment_ids[i].first = segment_ids[i].first - add_points_each_side >= bounds[i].first ? segment_ids[i].first - add_points_each_side : bounds[i].first;

        final_segment_ids[i].second = segment_ids[i].second + add_points_each_side <= bounds[i].second ? segment_ids[i].second + add_points_each_side : bounds[i].second;
      }
      else
      {
        final_segment_ids[i].first = segment_ids[i].first;
        final_segment_ids[i].second = segment_ids[i].second;
      }

      //cout << "final:" << "i = " << i << " first = " << final_segment_ids[i].first << " second = " << final_segment_ids[i].second << endl;
    }

    /*** Assign data to each segment ***/
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      // step 1
      for (int j = final_segment_ids[i].first; j <= final_segment_ids[i].second; ++j)
        cps_.flag_temp[j] = false;

      // step 2
      int got_intersection_id = -1;
      for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
      {
        Eigen::Vector2d ctrl_pts_law(cps_.points.col(j + 1) - cps_.points.col(j - 1)), intersection_point;
        int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
        double val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law), last_val = val;
        while (Astar_id >= 0 && Astar_id < (int)a_star_pathes[i].size())
        {
          last_Astar_id = Astar_id;

          if (val >= 0)
            --Astar_id;
          else
            ++Astar_id;

          val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law);

          if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
          {
            intersection_point =
                a_star_pathes[i][Astar_id] +
                ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(cps_.points.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                );

            //cout << "i=" << i << " j=" << j << " Astar_id=" << Astar_id << " last_Astar_id=" << last_Astar_id << " intersection_point = " << intersection_point.transpose() << endl;

            got_intersection_id = j;
            break;
          }
        }

        if (got_intersection_id >= 0)
        {
          cps_.flag_temp[j] = true;
          double length = (intersection_point - cps_.points.col(j)).norm();
          if (length > 1e-5)
          {
            for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
            {
              occ = grid_map_->getInflateOccupancy((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));

              if (occ || a < grid_map_->getResolution())
              {
                if (occ)
                  a += grid_map_->getResolution();
                cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));
                cps_.direction[j].push_back((intersection_point - cps_.points.col(j)).normalized());
                break;
              }
            }
          }
        }
      }

      /* Corner case: the segment length is too short. Here the control points may outside the A* path, leading to opposite gradient direction. So I have to take special care of it */
      if (segment_ids[i].second - segment_ids[i].first == 1)
      {
        Eigen::Vector2d ctrl_pts_law(cps_.points.col(segment_ids[i].second) - cps_.points.col(segment_ids[i].first)), intersection_point;
        Eigen::Vector2d middle_point = (cps_.points.col(segment_ids[i].second) + cps_.points.col(segment_ids[i].first)) / 2;
        int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
        double val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law), last_val = val;
        while (Astar_id >= 0 && Astar_id < (int)a_star_pathes[i].size())
        {
          last_Astar_id = Astar_id;

          if (val >= 0)
            --Astar_id;
          else
            ++Astar_id;

          val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law);

          if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
          {
            intersection_point =
                a_star_pathes[i][Astar_id] +
                ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(middle_point - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                );

            if ((intersection_point - middle_point).norm() > 0.01) // 1cm.
            {
              cps_.flag_temp[segment_ids[i].first] = true;
              cps_.base_point[segment_ids[i].first].push_back(cps_.points.col(segment_ids[i].first));
              cps_.direction[segment_ids[i].first].push_back((intersection_point - middle_point).normalized());

              got_intersection_id = segment_ids[i].first;
            }
            break;
          }
        }
      }

      //step 3
      if (got_intersection_id >= 0)
      {
        for (int j = got_intersection_id + 1; j <= final_segment_ids[i].second; ++j)
          if (!cps_.flag_temp[j])
          {
            cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
            cps_.direction[j].push_back(cps_.direction[j - 1].back());
          }

        for (int j = got_intersection_id - 1; j >= final_segment_ids[i].first; --j)
          if (!cps_.flag_temp[j])
          {
            cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
            cps_.direction[j].push_back(cps_.direction[j + 1].back());
          }
      }
      else
      {
        printf(" ailed to generate direction! Just ignore, it does not matter \n");
        // Just ignore, it does not matter ^_^.
        // ROS_ERROR("Failed to generate direction! segment_id=%d", i);
      }
    }

    return a_star_pathes;
}
int BsplineOptimizer::earlyExit(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
{
  BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);
  return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
}

double BsplineOptimizer::costFunctionRebound(void *func_data, const double *x, double *grad, const int n)
{
  BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);
  double cost;
  opt->combineCostRebound(x, grad, cost, n);
  opt->iter_num_ += 1;
  return cost;
}

double BsplineOptimizer::costFunctionRefine(void *func_data, const double *x, double *grad, const int n)
{
  BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);
  double cost;
  opt->combineCostRefine(x, grad, cost, n);
  opt->iter_num_ += 1;
  return cost;
}

void BsplineOptimizer::calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost,
                                                 Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost)
{
    cost = 0.0;
    int end_idx = q.cols() - order_;
    double demarcation = cps_.clearance;
    double a = 3 * demarcation, b = -3 * pow(demarcation, 2), c = pow(demarcation, 3);

    force_stop_type_ = DONT_STOP;
    if (iter_num > 3 && smoothness_cost / (cps_.size - 2 * order_) < 0.1) // 0.1 is an experimental value that indicates the trajectory is smooth enough.
    {
      check_collision_and_rebound();
    }

    /*** calculate distance cost and gradient ***/
    for (auto i = order_; i < end_idx; ++i)
    {
      for (size_t j = 0; j < cps_.direction[i].size(); ++j)
      {
        double dist = (cps_.points.col(i) - cps_.base_point[i][j]).dot(cps_.direction[i][j]);
        double dist_err = cps_.clearance - dist;
        Eigen::Vector2d dist_grad = cps_.direction[i][j];

        if (dist_err < 0)
        {
          /* do nothing */
        }
        else if (dist_err < demarcation)
        {
          cost += pow(dist_err, 3);
          gradient.col(i) += -3.0 * dist_err * dist_err * dist_grad;
        }
        else
        {
          cost += a * dist_err * dist_err + b * dist_err + c;
          gradient.col(i) += -(2.0 * a * dist_err + b) * dist_grad;
        }
      }
    }
}

void BsplineOptimizer::calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
{
  cost = 0.0;
  int end_idx = q.cols() - order_;
  double a2 = 25, b2 = 1;

  for (auto i = order_ - 1; i < end_idx + 1; ++i)
  {
    // 2D 控制点加权平均
    Eigen::Vector2d x = (q.col(i - 1) + 4 * q.col(i) + q.col(i + 1)) / 6.0 - ref_pts_[i - 1];
    Eigen::Vector2d v = (ref_pts_[i] - ref_pts_[i - 2]).normalized();

    double xdotv = x.dot(v);
    double xcrossv_mag = x(0)*v(1) - x(1)*v(0); // 2D 叉乘（大小）
    double f = xdotv*xdotv / a2 + xcrossv_mag*xcrossv_mag / b2;
    cost += f;

    // 2D 梯度计算
    Eigen::Vector2d df_dx;
    df_dx(0) = 2 * xdotv * v(0) / a2 + 2 * xcrossv_mag * v(1) / b2;
    df_dx(1) = 2 * xdotv * v(1) / a2 - 2 * xcrossv_mag * v(0) / b2;

    // 仅更新 x、y 梯度
    gradient.col(i - 1) += df_dx / 6;
    gradient.col(i) += 4 * df_dx / 6;
    gradient.col(i + 1)+= df_dx / 6;
  }
}

void BsplineOptimizer::calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                                          Eigen::MatrixXd &gradient, bool falg_use_jerk /* = true*/)
{
  cost = 0.0;
  gradient.setZero(2, q.cols()); // 梯度矩阵改为 2 行（x、y）

  if (falg_use_jerk)
  {
    Eigen::Vector2d jerk, temp_j;
    for (int i = 0; i < q.cols() - 3; i++)
    {
      // 2D jerk 计算
      jerk = q.col(i + 3) - 3 * q.col(i + 2) + 3 * q.col(i + 1) - q.col(i);
      cost += jerk.squaredNorm();
      temp_j = 2.0 * jerk;

      // 仅更新 x、y 梯度
      gradient.col(i + 0) += -temp_j;
      gradient.col(i + 1) += 3.0 * temp_j;
      gradient.col(i + 2) += -3.0 * temp_j;
      gradient.col(i + 3) += temp_j;
    }
  }
  else
  {
    Eigen::Vector2d acc, temp_acc;
    for (int i = 0; i < q.cols() - 2; i++)
    {
      // 2D acc 计算
      acc = q.col(i + 2) - 2 * q.col(i + 1) + q.col(i);
      cost += acc.squaredNorm();
      temp_acc = 2.0 * acc;

      // 仅更新 x、y 梯度
      gradient.col(i + 0) += temp_acc;
      gradient.col(i + 1) += -2.0 * temp_acc;
      gradient.col(i + 2) += temp_acc;
    }
  }
}


  void BsplineOptimizer::calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                                             Eigen::MatrixXd &gradient)
  {

    //#define SECOND_DERIVATIVE_CONTINOUS

#ifdef SECOND_DERIVATIVE_CONTINOUS

    cost = 0.0;
    double demarcation = 1.0; // 1m/s, 1m/s/s
    double ar = 3 * demarcation, br = -3 * pow(demarcation, 2), cr = pow(demarcation, 3);
    double al = ar, bl = -br, cl = cr;

    /* abbreviation */
    double ts, ts_inv2, ts_inv3;
    ts = bspline_interval_;
    ts_inv2 = 1 / ts / ts;
    ts_inv3 = 1 / ts / ts / ts;

    /* velocity feasibility */
    for (int i = 0; i < q.cols() - 1; i++)
    {
      Eigen::Vector2d vi = (q.col(i + 1) - q.col(i)) / ts;

      for (int j = 0; j < 2; j++)
      {
        if (vi(j) > max_vel_ + demarcation)
        {
          double diff = vi(j) - max_vel_;
          cost += (ar * diff * diff + br * diff + cr) * ts_inv3; // multiply ts_inv3 to make vel and acc has similar magnitude

          double grad = (2.0 * ar * diff + br) / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) > max_vel_)
        {
          double diff = vi(j) - max_vel_;
          cost += pow(diff, 3) * ts_inv3;
          ;

          double grad = 3 * diff * diff / ts * ts_inv3;
          ;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) < -(max_vel_ + demarcation))
        {
          double diff = vi(j) + max_vel_;
          cost += (al * diff * diff + bl * diff + cl) * ts_inv3;

          double grad = (2.0 * al * diff + bl) / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) < -max_vel_)
        {
          double diff = vi(j) + max_vel_;
          cost += -pow(diff, 3) * ts_inv3;

          double grad = -3 * diff * diff / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else
        {
          /* nothing happened */
        }
      }
    }

    /* acceleration feasibility */
    for (int i = 0; i < q.cols() - 2; i++)
    {
      Eigen::Vector2d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

      for (int j = 0; j < 2; j++)
      {
        if (ai(j) > max_acc_ + demarcation)
        {
          double diff = ai(j) - max_acc_;
          cost += ar * diff * diff + br * diff + cr;

          double grad = (2.0 * ar * diff + br) * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) > max_acc_)
        {
          double diff = ai(j) - max_acc_;
          cost += pow(diff, 3);

          double grad = 3 * diff * diff * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) < -(max_acc_ + demarcation))
        {
          double diff = ai(j) + max_acc_;
          cost += al * diff * diff + bl * diff + cl;

          double grad = (2.0 * al * diff + bl) * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) < -max_acc_)
        {
          double diff = ai(j) + max_acc_;
          cost += -pow(diff, 3);

          double grad = -3 * diff * diff * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else
        {
          /* nothing happened */
        }
      }
    }

#else

    cost = 0.0;
    /* abbreviation */
    double ts, /*vm2, am2, */ ts_inv2;
    // vm2 = max_vel_ * max_vel_;
    // am2 = max_acc_ * max_acc_;

    ts = bspline_interval_;
    ts_inv2 = 1 / ts / ts;

    /* velocity feasibility */
    for (int i = 0; i < q.cols() - 1; i++)
    {
      Eigen::Vector2d vi = (q.col(i + 1) - q.col(i)) / ts;

      //cout << "temp_v * vi=" ;
      for (int j = 0; j < 2; j++)
      {
        if (vi(j) > max_vel_)
        {
          // cout << "fuck VEL" << endl;
          // cout << vi(j) << endl;
          cost += pow(vi(j) - max_vel_, 2) * ts_inv2; // multiply ts_inv3 to make vel and acc has similar magnitude

          gradient(j, i + 0) += -2 * (vi(j) - max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) - max_vel_) / ts * ts_inv2;
        }
        else if (vi(j) < -max_vel_)
        {
          cost += pow(vi(j) + max_vel_, 2) * ts_inv2;

          gradient(j, i + 0) += -2 * (vi(j) + max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) + max_vel_) / ts * ts_inv2;
        }
        else
        {
          /* code */
        }
      }
    }

    /* acceleration feasibility */
    for (int i = 0; i < q.cols() - 2; i++)
    {
      Eigen::Vector2d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

      //cout << "temp_a * ai=" ;
      for (int j = 0; j < 2; j++)
      {
        if (ai(j) > max_acc_)
        {
          // cout << "fuck ACC" << endl;
          // cout << ai(j) << endl;
          cost += pow(ai(j) - max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) - max_acc_) * ts_inv2;
        }
        else if (ai(j) < -max_acc_)
        {
          cost += pow(ai(j) + max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) + max_acc_) * ts_inv2;
        }
        else
        {
          /* code */
        }
      }
      //cout << endl;
    }

#endif
  }


bool BsplineOptimizer::check_collision_and_rebound(void)
{

    int end_idx = cps_.size - order_;

    /*** Check and segment the initial trajectory according to obstacles ***/
    int in_id, out_id;
    vector<std::pair<int, int>> segment_ids;
    bool flag_new_obs_valid = false;
    int i_end = end_idx - (end_idx - order_) / 3;
    for (int i = order_ - 1; i <= i_end; ++i)
    {

      bool occ = grid_map_->getInflateOccupancy(cps_.points.col(i));

      /*** check if the new collision will be valid ***/
      if (occ)
      {
        for (size_t k = 0; k < cps_.direction[i].size(); ++k)
        {
          cout.precision(2);
          if ((cps_.points.col(i) - cps_.base_point[i][k]).dot(cps_.direction[i][k]) < 1 * grid_map_->getResolution()) // current point is outside all the collision_points.
          {
            occ = false; // Not really takes effect, just for better hunman understanding.
            break;
          }
        }
      }

      if (occ)
      {
        flag_new_obs_valid = true;

        int j;
        for (j = i - 1; j >= 0; --j)
        {
          occ = grid_map_->getInflateOccupancy(cps_.points.col(j));
          if (!occ)
          {
            in_id = j;
            break;
          }
        }
        if (j < 0) // fail to get the obs free point
        {
          printf("ERROR! the drone is in obstacle. This should not happen.");
          in_id = 0;
        }

        for (j = i + 1; j < cps_.size; ++j)
        {
          occ = grid_map_->getInflateOccupancy(cps_.points.col(j));

          if (!occ)
          {
            out_id = j;
            break;
          }
        }
        if (j >= cps_.size) // fail to get the obs free point
        {
          printf("WARN! terminal point of the current trajectory is in obstacle, skip this planning.");

          force_stop_type_ = STOP_FOR_ERROR;
          return false;
        }

        i = j + 1;

        segment_ids.push_back(std::pair<int, int>(in_id, out_id));
      }
    }

    if (flag_new_obs_valid)
    {
      vector<vector<Eigen::Vector2d>> a_star_pathes;
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        /*** a star search ***/
        Eigen::Vector2d in(cps_.points.col(segment_ids[i].first)), out(cps_.points.col(segment_ids[i].second));
        if (a_star_->AstarSearch(/*(in-out).norm()/10+0.05*/ 0.1, in, out))
        {
          a_star_pathes.push_back(a_star_->getPath());
        }
        else
        {
          printf("a star error");
          segment_ids.erase(segment_ids.begin() + i);
          i--;
        }
      }

      /*** Assign parameters to each segment ***/
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        // step 1
        for (int j = segment_ids[i].first; j <= segment_ids[i].second; ++j)
          cps_.flag_temp[j] = false;

        // step 2
        int got_intersection_id = -1;
        for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
        {
          Eigen::Vector2d ctrl_pts_law(cps_.points.col(j + 1) - cps_.points.col(j - 1)), intersection_point;
          int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
          double val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law), last_val = val;
          while (Astar_id >= 0 && Astar_id < (int)a_star_pathes[i].size())
          {
            last_Astar_id = Astar_id;

            if (val >= 0)
              --Astar_id;
            else
              ++Astar_id;

            val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law);

            // cout << val << endl;

            if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
            {
              intersection_point =
                  a_star_pathes[i][Astar_id] +
                  ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                   (ctrl_pts_law.dot(cps_.points.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                  );

              got_intersection_id = j;
              break;
            }
          }

          if (got_intersection_id >= 0)
          {
            cps_.flag_temp[j] = true;
            double length = (intersection_point - cps_.points.col(j)).norm();
            if (length > 1e-5)
            {
              for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
              {
                bool occ = grid_map_->getInflateOccupancy((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));

                if (occ || a < grid_map_->getResolution())
                {
                  if (occ)
                    a += grid_map_->getResolution();
                  cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));
                  cps_.direction[j].push_back((intersection_point - cps_.points.col(j)).normalized());
                  break;
                }
              }
            }
            else
            {
              got_intersection_id = -1;
            }
          }
        }

        //step 3
        if (got_intersection_id >= 0)
        {
          for (int j = got_intersection_id + 1; j <= segment_ids[i].second; ++j)
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
              cps_.direction[j].push_back(cps_.direction[j - 1].back());
            }

          for (int j = got_intersection_id - 1; j >= segment_ids[i].first; --j)
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
              cps_.direction[j].push_back(cps_.direction[j + 1].back());
            }
        }
        else
          printf("Failed to generate direction. It doesn't matter.");
      }

      force_stop_type_ = STOP_FOR_REBOUND;
      return true;
    }

    return false;
  }

bool BsplineOptimizer::BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts)
{
  setBsplineInterval(ts);
  bool flag_success = rebound_optimize();
  optimal_points = cps_.points; // 输出 2D 控制点
  return flag_success;
}

bool BsplineOptimizer::BsplineOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points)
{
  setControlPoints(init_points);
  setBsplineInterval(ts);
  bool flag_success = refine_optimize();
  optimal_points = cps_.points; // 输出 2D 控制点
  return flag_success;
}

Eigen::MatrixXd BsplineOptimizer::convert2DTo3D(const Eigen::MatrixXd& points_2d) const 
{
  if (points_2d.rows() != 2) {
        std::cerr << "Error: Input matrix must be 2 rows for 2D to 3D conversion." << std::endl;
        // 返回一个空矩阵表示错误
        return Eigen::MatrixXd();
  }

    // 1. 获取2D矩阵的列数
  int num_cols = points_2d.cols();

  // 2. 创建一个3 x N 的输出矩阵
  Eigen::MatrixXd points_3d(3, num_cols);

  // 3. 将x, y分量复制到3D矩阵的前两行
  points_3d.topRows(2) = points_2d;

  // 4. 将z分量（第三行）全部设置为0
  points_3d.bottomRows(1).setZero();

  return points_3d;
}

// 8. 反弹优化：适配 2D 优化变量
bool BsplineOptimizer::rebound_optimize()
{
  iter_num_ = 0;
  int start_id = order_;
  int end_id = cps_.points.cols() - order_;
  variable_num_ = 2 * (end_id - start_id); // 优化变量：2*(控制点数量)
  double final_cost;

  int restart_nums = 0, rebound_times = 0;
  bool flag_force_return, flag_occ, success;
  new_lambda2_ = lambda2_;
  constexpr int MAX_RESART_NUMS_SET = 3;
  std::cout << "start optimze" << std::endl;

  do
  {
    min_cost_ = std::numeric_limits<double>::max();
    iter_num_ = 0;
    flag_force_return = false;
    flag_occ = false;
    success = false;
    double q[variable_num_];
    memcpy(q, cps_.points.data() + 2 * start_id, variable_num_ * sizeof(q[0]));


    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size = 16;
    lbfgs_params.max_iterations = 200;
    lbfgs_params.g_epsilon = 0.001;

    int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost, BsplineOptimizer::costFunctionRebound, NULL, BsplineOptimizer::earlyExit, this, &lbfgs_params);
    std::cout << "result =" << result << std::endl;

    if (result == lbfgs::LBFGS_CONVERGENCE ||
        result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
        result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
        result == lbfgs::LBFGS_STOP)
    {
      flag_force_return = false;

      // 碰撞检测（仅用 x、y）
      UniformBspline traj = UniformBspline(cps_.points, 3, bspline_interval_);
      double tm, tmp;
      traj.getTimeSpan(tm, tmp);
      double t_step = (tmp - tm) / ((traj.evaluateDeBoorT(tmp) - traj.evaluateDeBoorT(tm)).norm() / grid_map_->getResolution());
      for (double t = tm; t < tmp * 2 / 3; t += t_step)
      {
        Eigen::Vector2d ctrl_point_2d = traj.evaluateDeBoorT(t);
        flag_occ = grid_map_->getInflateOccupancy(ctrl_point_2d);
        if (flag_occ)
        {
          if (t <= bspline_interval_)
          {
            std::cout << cps_.points.col(1).transpose() << "\n"
                      << cps_.points.col(2).transpose() << "\n"
                      << cps_.points.col(3).transpose() << "\n"
                      << cps_.points.col(4).transpose() << std::endl;
            printf("First 3 control points in obstacles! return false, t=%f", t);
            return false;
          }
          break;
        }
      }

      if (!flag_occ)
      {
        success = true;
      }
      else
      {
        restart_nums++;
        initControlPoints(cps_.points, false);
        new_lambda2_ *= 2;
      }
    }
    else if (result == lbfgs::LBFGSERR_CANCELED)
    {
      flag_force_return = true;
      rebound_times++;
      std::cout << "iter=" << iter_num_ << ",rebound." << std::endl;
    }
    else
    {
      printf("Solver error. Return = %d, %s. Skip this planning.\n", result, lbfgs::lbfgs_strerror(result));
    }

  } while ((flag_occ && restart_nums < MAX_RESART_NUMS_SET) ||
           (flag_force_return && force_stop_type_ == STOP_FOR_REBOUND && rebound_times <= 20));
  
  std::cout << "optimuze complete" << std::endl;
  return success;
}

bool BsplineOptimizer::refine_optimize()
{
  iter_num_ = 0;
  int start_id = order_;
  int end_id = cps_.points.cols() - order_;
  variable_num_ = 2 * (end_id - start_id); // 2D 优化变量数量

  // 提取 x、y 到 q
  double q[variable_num_];
  memcpy(q, cps_.points.data() + 2 * start_id, variable_num_ * sizeof(q[0]));

  double final_cost;
  double origin_lambda4 = lambda4_;
  bool flag_safe = true;
  int iter_count = 0;

  do
  {
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size = 16;
    lbfgs_params.max_iterations = 200;
    lbfgs_params.g_epsilon = 0.001;

    int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost, BsplineOptimizer::costFunctionRefine, NULL, NULL, this, &lbfgs_params);
    if (!(result == lbfgs::LBFGS_CONVERGENCE ||
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
          result == lbfgs::LBFGS_STOP))
    {
      printf("Solver error in refining!, return = %d, %s", result, lbfgs::lbfgs_strerror(result));
    }

    // 碰撞检测（仅 x、y）
    UniformBspline traj = UniformBspline(cps_.points, 3, bspline_interval_);
    double tm, tmp;
    traj.getTimeSpan(tm, tmp);
    double t_step = (tmp - tm) / ((traj.evaluateDeBoorT(tmp).topRows(2) - traj.evaluateDeBoorT(tm).topRows(2)).norm() / grid_map_->getResolution());
    for (double t = tm; t < tmp * 2 / 3; t += t_step)
    {
      Eigen::Vector2d ctrl_point_2d = traj.evaluateDeBoorT(t).topRows(2);
      if (grid_map_->getInflateOccupancy(ctrl_point_2d))
      {
        flag_safe = false;
        break;
      }
    }

    if (!flag_safe)
      lambda4_ *= 2;

    iter_count++;
  } while (!flag_safe && iter_count <= 0);

  lambda4_ = origin_lambda4;
  return flag_safe;
}

// 10. 组合代价（反弹）：适配 2D 梯度
void BsplineOptimizer::combineCostRebound(const double *x, double *grad, double &f_combine, const int n)
{

  memcpy(cps_.points.data() + 2 * order_, x, n * sizeof(x[0]));

  /* 计算各代价（均为 2D）*/
  double f_smoothness = 0, f_distance = 0, f_feasibility = 0,f_turn = 0,f_kappa = 0;
  Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(2, cps_.points.cols()); // 2D 梯度
  Eigen::MatrixXd g_distance = Eigen::MatrixXd::Zero(2, cps_.points.cols());
  Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(2, cps_.points.cols());
  Eigen::MatrixXd g_turn = Eigen::MatrixXd::Zero(2, cps_.size);
  Eigen::MatrixXd g_kappa = Eigen::MatrixXd::Zero(2, cps_.size);

  calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
  calcDistanceCostRebound(cps_.points, f_distance, g_distance, iter_num_, f_smoothness);
  calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);

  // calTurnCost(cps_.points,f_turn,g_turn);
  // calKappaCost(cps_.points,f_kappa,g_kappa);

  printf(" lambda1_ * f_smoothness  %f,  new_lambda2_ * f_distance = %f lambda3_ * f_feasibility = %f \n",lambda1_ * f_smoothness,new_lambda2_ * f_distance,lambda3_ * f_feasibility);


  /* 组合代价和梯度 */
  f_combine = lambda1_ * f_smoothness + new_lambda2_ * f_distance + lambda3_ * f_feasibility;// +  f_kappa;
  Eigen::MatrixXd grad_2D = lambda1_ * g_smoothness + new_lambda2_ * g_distance + lambda3_ * g_feasibility;// +  g_kappa;// +;

  memcpy(grad, grad_2D.data() + 2 * order_, n * sizeof(grad[0]));
}

// 曲率代价：仅基于 x、y 计算  曲率约束还不稳定 juchunyu@qq.com
void BsplineOptimizer::calKappaCost(const Eigen::MatrixXd &q, double &cost,
                           Eigen::MatrixXd &gradient)
{
  cost = 0.0;
  double ts = bspline_interval_;
  double ts_inv2 = 1 / ts / ts;
  double k_max = 2;
  double K_weight = lambda5_;
  const double epsilon = 1e-6;

  gradient.setZero(2, q.cols()); // 2D 梯度

  for (int i = 0; i < q.cols() - 2; i++)
  {
    // 2D 速度和加速度
    Eigen::Vector2d vi = (q.col(i + 1) - q.col(i)) / ts;
    Eigen::Vector2d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;
    
    double ax = ai(0), ay = ai(1);
    double vx = vi(0), vy = vi(1);
    double v_square = vx*vx + vy*vy + epsilon;
    double v_mag_3 = pow(v_square, 3.0/2.0);
    double K = (vx * ay - vy * ax) / v_mag_3;

    if(K > k_max)
    {
      cost += pow((K - k_max),2) * K_weight;
      double djdk = 2 * (K - k_max) * K_weight;

      // 2D 梯度计算
      double dkdax = -vy / v_mag_3;
      double dkday = vx / v_mag_3;
      double dkdvx = ay / v_mag_3 - 3 * K * vx / v_square;
      double dkdvy = -ax / v_mag_3 - 3 * K * vy / v_square;
      
      double dadq1 = ts_inv2;
      double dadq2 = -2 * ts_inv2;
      double dadq3 = ts_inv2;
      double dvdq1 = -1 / ts;
      double dvdq2 = 1 / ts;
      double dvdq3 = 0;

      // 仅更新 x、y 梯度
      gradient(0, i + 0) += djdk * (dkdax * dadq1 + dkdvx * dvdq1);
      gradient(1, i + 0) += djdk * (dkday * dadq1 + dkdvy * dvdq1);
      gradient(0, i + 1) += djdk * (dkdax * dadq2 + dkdvx * dvdq2);
      gradient(1, i + 1) += djdk * (dkday * dadq2 + dkdvy * dvdq2);
      gradient(0, i + 2) += djdk * (dkdax * dadq3 + dkdvx * dvdq3);
      gradient(1, i + 2) += djdk * (dkday * dadq3 + dkdvy * dvdq3);
    }
    else if(K  < -k_max)
    {
      cost += pow((K + k_max),2) * K_weight;
      double djdk = 2 * (K + k_max) * K_weight;

      double dkdax = -vy / v_mag_3;
      double dkday = vx / v_mag_3;
      double dkdvx = ay / v_mag_3 - 3 * K * vx / v_square;
      double dkdvy = -ax / v_mag_3 - 3 * K * vy / v_square;
      
      double dadq1 = ts_inv2;
      double dadq2 = -2 * ts_inv2;
      double dadq3 = ts_inv2;
      double dvdq1 = -1 / ts;
      double dvdq2 = 1 / ts;
      double dvdq3 = 0;

      gradient(0, i + 0) += djdk * (dkdax * dadq1 + dkdvx * dvdq1);
      gradient(1, i + 0) += djdk * (dkday * dadq1 + dkdvy * dvdq1);
      gradient(0, i + 1) += djdk * (dkdax * dadq2 + dkdvx * dvdq2);
      gradient(1, i + 1) += djdk * (dkday * dadq2 + dkdvy * dvdq2);
      gradient(0, i + 2) += djdk * (dkdax * dadq3 + dkdvx * dvdq3);
      gradient(1, i + 2) += djdk * (dkday * dadq3 + dkdvy * dvdq3);
    }
  }

  std::cout << "[calKappaCost] cost =" << cost << std::endl;
}

// 转向代价：仅基于 x、y 计算  约束还不稳定 juchunyu@qq.com
void BsplineOptimizer::calTurnCost(const Eigen::MatrixXd &q, double &cost,
                           Eigen::MatrixXd &gradient)
{
  cost = 0.0;
  double ts = bspline_interval_;
  double ts_inv2 = 1 / ts / ts;
  double w_max = 1.0;
  double w_weight = 0.1;
  gradient.setZero(2, q.cols()); // 2D 梯度

  for (int i = 0; i < q.cols() - 2; i++)
  {
    Eigen::Vector2d vi = (q.col(i + 1) - q.col(i)) / ts;
    Eigen::Vector2d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;
    
    double ax = ai(0), ay = ai(1);
    double vx = vi(0), vy = vi(1);

    double v_square = vx*vx + vy*vy;
    if(v_square < 0.0001)
    {
      v_square = 0.1;
    }

    // 2D 转向角计算（W = (ay*vx - vy*ax)/v_square）
    double W = (ay * vx - vy * ax) / v_square;

    if(W > w_max)
    {
      cost += pow((W - w_max), 2) * w_weight;
      double djdw = 2 * (W - w_max) * w_weight;

      // 2D 梯度计算
      double dwdax = -vy / v_square;
      double dwday = vx / v_square;
      double dwdvx = (ay * v_square - 2 * vx * (ay*vx - vy*ax)) / (v_square * v_square);
      double dwdvy = (-ax * v_square - 2 * vy * (ay*vx - vy*ax)) / (v_square * v_square);
      
      double dadq1 = ts_inv2;
      double dadq2 = -2 * ts_inv2;
      double dadq3 = ts_inv2;
      double dvdq1 = -1 / ts;
      double dvdq2 = 1 / ts;
      double dvdq3 = 0;

      // 仅更新 x、y 梯度
      gradient(0, i + 0) += djdw * (dwdax * dadq1 + dwdvx * dvdq1);
      gradient(1, i + 0) += djdw * (dwday * dadq1 + dwdvy * dvdq1);
      gradient(0, i + 1) += djdw * (dwdax * dadq2 + dwdvx * dvdq2);
      gradient(1, i + 1) += djdw * (dwday * dadq2 + dwdvy * dvdq2);
      gradient(0, i + 2) += djdw * (dwdax * dadq3 + dwdvx * dvdq3);
      gradient(1, i + 2) += djdw * (dwday * dadq3 + dwdvy * dvdq3);
    }
    else if(W < -w_max)
    {
      cost += pow(W + w_max, 2) * w_weight;
      double djdw = 2 * (W + w_max) * w_weight;

      double dwdax = -vy / v_square;
      double dwday = vx / v_square;
      double dwdvx = (ay * v_square - 2 * vx * (ay*vx - vy*ax)) / (v_square * v_square);
      double dwdvy = (-ax * v_square - 2 * vy * (ay*vx - vy*ax)) / (v_square * v_square);
      
      double dadq1 = ts_inv2;
      double dadq2 = -2 * ts_inv2;
      double dadq3 = ts_inv2;
      double dvdq1 = -1 / ts;
      double dvdq2 = 1 / ts;
      double dvdq3 = 0;

      gradient(0, i + 0) += djdw * (dwdax * dadq1 + dwdvx * dvdq1);
      gradient(1, i + 0) += djdw * (dwday * dadq1 + dwdvy * dvdq1);
      gradient(0, i + 1) += djdw * (dwdax * dadq2 + dwdvx * dvdq2);
      gradient(1, i + 1) += djdw * (dwday * dadq2 + dwdvy * dvdq2);
      gradient(0, i + 2) += djdw * (dwdax * dadq3 + dwdvx * dvdq3);
      gradient(1, i + 2) += djdw * (dwday * dadq3 + dwdvy * dvdq3);
    }
  }
  std::cout << "[turn cost cost =" << cost << std::endl;
}

// 13. 组合代价（精修）：适配 2D 优化
void BsplineOptimizer::combineCostRefine(const double *x, double *grad, double &f_combine, const int n)
{
  memcpy(cps_.points.data() + 2 * order_, x, n * sizeof(x[0]));
  /* 计算各代价*/
  double f_smoothness = 0, f_fitness = 0, f_feasibility = 0;
  Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(2, cps_.points.cols()); // 2D 梯度矩阵
  Eigen::MatrixXd g_fitness = Eigen::MatrixXd::Zero(2, cps_.points.cols());
  Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(2, cps_.points.cols());

  calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
  calcFitnessCost(cps_.points, f_fitness, g_fitness);
  calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);

  /* 组合代价和梯度 */
  f_combine = lambda1_ * f_smoothness + lambda4_ * f_fitness + lambda3_ * f_feasibility;
  Eigen::MatrixXd grad_2D = lambda1_ * g_smoothness + lambda4_ * g_fitness + lambda3_ * g_feasibility;
  memcpy(grad, grad_2D.data() + 2 * order_, n * sizeof(grad[0]));
}

} // namespace ego_planner

