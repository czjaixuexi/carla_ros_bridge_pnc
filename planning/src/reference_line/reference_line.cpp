/**
 * @file reference_line.cpp
 * @author czj
 * @brief
 * @version 0.1
 * @date 2023-05-22
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "reference_line/reference_line.h"

using namespace std;

namespace carla_pnc
{
  // /***********************************辅助函数**************************************/
  // /**
  //  * @brief 计算两点间的距离
  //  *
  //  * @param x1
  //  * @param y1
  //  * @param x2
  //  * @param y2
  //  * @return double
  //  */
  // double cal_distance(double x1, double y1, double x2, double y2)
  // {
  //   double dx = x2 - x1;
  //   double dy = y2 - y1;
  //   return std::sqrt(dx * dx + dy * dy);
  // }

  // /**
  //  * @brief 寻找最近匹配点下标
  //  *
  //  * @param cur_x
  //  * @param cur_y
  //  * @param path
  //  * @param pre_match_index
  //  * @return
  //  */
  // int search_match_index(const double &cur_x, const double &cur_y,
  //                        const std::vector<path_point> &path,
  //                        const int &pre_match_index)
  // {
  //   double dist;
  //   double min_dist = DBL_MAX;
  //   int match_index = 0;
  //   for (int i = pre_match_index; i < path.size(); i++)
  //   {
  //     dist = cal_distance(path[i].x, path[i].y, cur_x, cur_y);
  //     if (dist < min_dist)
  //     {
  //       min_dist = dist;
  //       match_index = i;
  //     }
  //   }
  //   return match_index;
  // }

  // /**
  //  * @brief 通过匹配点求投影点
  //  *
  //  * @param match_point 匹配点
  //  * @param projection_point 投影点
  //  * @return path_point
  //  * https://zhuanlan.zhihu.com/p/429676544
  //  */
  // path_point match_to_projection(const car_state &cur_pose,
  //                                const path_point &match_point)
  // {
  //   // 投影点其他值都与匹配点相同,求其x,y,yaw
  //   path_point projection_point = match_point;

  //   // 匹配点切向量tor
  //   Eigen::Matrix<double, 2, 1> tor;
  //   tor << cos(match_point.yaw), sin(match_point.yaw);

  //   // 匹配点至自车向量d
  //   Eigen::Matrix<double, 2, 1> d;
  //   d << cur_pose.x - match_point.x, cur_pose.y - match_point.y;

  //   // d在tor方向上的投影分量
  //   double e_s = tor.transpose() * d;

  //   // 求投影点x,y
  //   projection_point.x = match_point.x + e_s * cos(match_point.yaw);
  //   projection_point.y = match_point.y + e_s * sin(match_point.yaw);

  //   // 求投影点的yaw
  //   projection_point.yaw = match_point.yaw + match_point.cur * e_s;
  //   return projection_point;
  // }

  /*******************************Class ReferenceLine ******************************************/

  /**
   * @brief Construct a new Reference Line:: Reference Line object
   *
   * @param lookahead_distance
   */
  ReferenceLine::ReferenceLine(double lookahead_distance,
                               std::unordered_map<std::string, double> &referline_params)
  {
    lookahead_dist = lookahead_distance;
    match_index = 0;
    ref_weight_smooth = referline_params["ref_weight_smooth"];           // 参考线平滑代价
    ref_weight_path_length = referline_params["ref_weight_path_length"]; // 参考线轨迹长度代价
    ref_weight_ref_deviation = referline_params["w_lat_offset"];         // 参考线偏移代价
    // 二次规划几何相似度约束
    x_lower_bound = referline_params["x_lower_bound"];
    x_upper_bound = referline_params["x_upper_bound"];
    y_lower_bound = referline_params["y_lower_bound"];
    y_upper_bound = referline_params["y_upper_bound"];

  }

  /**
   * @brief 寻找最远路径点下标
   *
   * @param cur_x
   * @param cur_y
   * @param path
   * @param lookahead_distance
   * @return int
   */
  int ReferenceLine::search_target_index(const double &cur_x, const double &cur_y,
                                         const std::vector<path_point> &path,
                                         const double &lookahead_distance)
  {
    double dist;
    for (int i = match_index; i < path.size(); i++)
    {
      dist = cal_distance(cur_x, cur_y, path[i].x, path[i].y);
      if (dist > lookahead_distance)
      {
        return i;
      }
    }
    return path.size() - 1;
  }

  /**
   * @brief 截取局部路径点（reference_line）
   *
   * @param cur_pose 当前车辆位置
   * @param global_path 全局路径
   * @param pre_match_index 上一轮匹配点下标
   * @return
   */
  std::vector<path_point> ReferenceLine::local_path_truncation(const car_state &cur_pose,
                                                               const std::vector<path_point> &global_path,
                                                               const int &pre_match_index)
  {
    this->match_index = search_match_index(cur_pose.x, cur_pose.y, global_path, pre_match_index);
    // ROS_INFO("Match point_index is %d:", match_index);

    int target_index = search_target_index(cur_pose.x, cur_pose.y, global_path, lookahead_dist);
    // ROS_INFO("Tatget point_index is %d:", target_index);

    vector<path_point> target_path(global_path.begin() + this->match_index,
                                   global_path.begin() + target_index + 1);
    // ROS_INFO("Size of target_path :%d", target_path.size());
    return target_path;
  }

  /**
   * @brief cublic Spiline平滑
   *
   * @param ref_frenet
   * @param local_path
   * @return std::vector<path_point>
   */
  std::vector<path_point> ReferenceLine::smoothing(Spline2D &ref_frenet, const std::vector<path_point> &local_path)
  {
    std::vector<path_point> ref_path;
    ref_path.clear();
    for (double i = 0; i < ref_frenet.s.back(); i += 0.1)
    {
      std::array<double, 2> point_ = ref_frenet.calc_postion(i);
      path_point ref_point;
      ref_point.x = point_[0];
      ref_point.y = point_[1];
      ref_point.yaw = ref_frenet.calc_yaw(i);
      ref_point.cur = ref_frenet.calc_curvature(i);
      ref_point.s_ = i;
      ref_path.push_back(ref_point);
    }
    // ROS_INFO("The size of ref_path is:%zu", ref_path.size());
    return ref_path;
  }

  /**
   * @brief 离散点平滑（Apollo）
   *
   * @param local_path
   */
  std::vector<path_point> ReferenceLine::discrete_smooth(const std::vector<path_point> &local_path)
  {
    std::vector<path_point> smoothed_path;
    std::vector<std::pair<double, double>> path_point2d;
    for (auto point : local_path)
    {
      path_point2d.push_back(std::make_pair(point.x, point.y));
    }
    // 二次规划求解
    discrete_points_osqp(path_point2d);
    for (auto point2d : path_point2d)
    {
      path_point p;
      p.x = point2d.first;
      p.y = point2d.second;
      smoothed_path.push_back(p);
    }
    cal_heading(smoothed_path);
    return smoothed_path;
  }

  /**
   * @brief 离散点平滑的二次规划求解
   *
   * 二次规划QP
   * 0.5x'Hx + f'x = min
   * lb < Ax < ub
   * 二次规划形式

   * H1 = w_cost_smooth*(A1'*A1) + w_cost_length*(A2'*A2) + w_cost_ref*(A3'*A3)
   * f = -2 * w_cost_ref * referenceline_init
   * x'H1x + f'x = 0.5 * x'(2H1)*x + f'x
   * A1 = [1  0 -2  0  1  0
   *       0  1  0 -2  0  1
   *             1  0 -2  0  1  0
   *             0  1  0 -2  0  1
   *                   ...............

   *A2 = [1  0 -1  0
   *      0  1  0 -1
   *            1  0 -1  0
   *            0  1  0 -1
   *                  ...........
   *A3 为单位矩阵
   * @param path_point2d
   */
  void ReferenceLine::discrete_points_osqp(std::vector<std::pair<double, double>> &path_point2d)
  {
    int n = path_point2d.size();

    // 初始化A1,A2,A3，f,lb,ub矩阵
    // 平滑代价系数矩阵，x'A1'A1x, (n-2)
    Eigen::SparseMatrix<double> A1(2 * n, 2 * n);
    // 路径长度代价矩阵 x'A2'A2x,(n-1)
    Eigen::SparseMatrix<double> A2(2 * n, 2 * n);
    // 参考线偏离代价矩阵 x'A3'A3x,单位阵
    Eigen::SparseMatrix<double> A3(2 * n, 2 * n);

    Eigen::SparseMatrix<double> H(2 * n, 2 * n); // 必须是稀疏矩阵
    Eigen::VectorXd f = Eigen::VectorXd::Zero(2 * n);
    Eigen::SparseMatrix<double> A(2 * n, 2 * n);
    Eigen::VectorXd lb = Eigen::VectorXd::Zero(2 * n);
    Eigen::VectorXd ub = Eigen::VectorXd::Zero(2 * n);
    Eigen::VectorXd qp_solution = Eigen::VectorXd::Zero(2 * n);

    A.setIdentity();

    // 赋值f,lb,ub;
    //  MatrixXd下标从(0,0)开始,(1,2)表示第1行第2列
    for (int i = 0; i < n; i++)
    {
      f(2 * i) = path_point2d[i].first;
      f(2 * i + 1) = path_point2d[i].second;

      lb(2 * i) = f(2 * i) + x_lower_bound;
      lb(2 * i + 1) = f(2 * i + 1) + y_lower_bound;

      ub(2 * i) = f(2 * i) + x_upper_bound;
      ub(2 * i + 1) = f(2 * i + 1) + y_upper_bound;
    }

    // 赋值A1
    for (int j = 0; j < n - 2; j++)
    {
      A1.insert(2 * j, 2 * j) = 1;
      A1.insert(2 * j, 2 * j + 2) = -2;
      A1.insert(2 * j, 2 * j + 4) = 1;
      A1.insert(2 * j + 1, 2 * j + 1) = 1;
      A1.insert(2 * j + 1, 2 * j + 3) = -2;
      A1.insert(2 * j + 1, 2 * j + 5) = 1;
    }
    // 赋值A2
    for (int k = 0; k < n - 1; k++)
    {
      A2.insert(2 * k, 2 * k) = 1;
      A2.insert(2 * k, 2 * k + 2) = -1;
      A2.insert(2 * k + 1, 2 * k + 1) = 1;
      A2.insert(2 * k + 1, 2 * k + 3) = 1;
    }

    A3.setIdentity();

    // H = 2 * (config_.weight_smooth * (A1.transpose().dot(A1)) +
    //          config_.weight_path_length * (A2.transpose().dot(A2)) +
    //          config_.weight_ref_deviation * A3);
    H = 2 * (ref_weight_smooth * A1.transpose() * A1 +
             ref_weight_path_length * A2.transpose() * A2 +
             ref_weight_ref_deviation * A3);

    f = -2 * ref_weight_ref_deviation * f;

    OsqpEigen::Solver solver;
    solver.settings()->setWarmStart(true);
    solver.settings()->setVerbosity(false);
    solver.data()->setNumberOfVariables(2 * n);
    solver.data()->setNumberOfConstraints(2 * n);
    solver.data()->setHessianMatrix(H);
    solver.data()->setGradient(f);
    solver.data()->setLinearConstraintsMatrix(A);
    solver.data()->setLowerBound(lb);
    solver.data()->setUpperBound(ub);

    if (!solver.initSolver())
    {
      ROS_INFO("QSOP init failed");
      return;
    }
    if (!solver.solve())
    {
      ROS_INFO("QSOP solve failed");
      return;
    }
    qp_solution = solver.getSolution();

    for (int i = 0; i < n; i++)
    {
      path_point2d[i].first = qp_solution(2 * i);
      path_point2d[i].second = qp_solution(2 * i + 1);
    }
  }

  void ReferenceLine::cal_heading(vector<path_point> &waypoints)
  {
    double x_delta = 0.0;
    double y_delta = 0.0;
    double x_delta_2 = 0.0;
    double y_delta_2 = 0.0;
    for (int i = 0; i < waypoints.size(); i++)
    {
      if (i == 0)
      {
        x_delta = (waypoints[i + 1].x - waypoints[i].x);
        y_delta = (waypoints[i + 1].y - waypoints[i].y);
        x_delta_2 = (waypoints[i + 2].x - waypoints[i + 1].x) - (waypoints[i + 1].x - waypoints[i].x);
        y_delta_2 = (waypoints[i + 2].y - waypoints[i + 1].y) - (waypoints[i + 1].y - waypoints[i].y);
      }
      else if (i == waypoints.size() - 1)
      {
        x_delta = (waypoints[i].x - waypoints[i - 1].x);
        y_delta = (waypoints[i].y - waypoints[i - 1].y);
        x_delta_2 = (waypoints[i].x - waypoints[i - 1].x) - (waypoints[i - 1].x - waypoints[i - 2].x);
        y_delta_2 = (waypoints[i].y - waypoints[i - 1].y) - (waypoints[i - 1].y - waypoints[i - 2].y);
      }
      else
      {
        x_delta = 0.5 * (waypoints[i + 1].x - waypoints[i - 1].x);
        y_delta = 0.5 * (waypoints[i + 1].y - waypoints[i - 1].y);
        x_delta_2 = (waypoints[i + 1].x - waypoints[i].x) - (waypoints[i].x - waypoints[i - 1].x);
        y_delta_2 = (waypoints[i + 1].y - waypoints[i].y) - (waypoints[i].y - waypoints[i - 1].y);
      }
      waypoints[i].yaw = std::atan2(y_delta, x_delta);
      //  参数方程曲率计算
      waypoints[i].cur = std::abs(y_delta_2 * x_delta - x_delta_2 * y_delta) / std::pow((x_delta * x_delta + y_delta * y_delta), 3 / 2);
    }
  }

} // namespace carla_pnc