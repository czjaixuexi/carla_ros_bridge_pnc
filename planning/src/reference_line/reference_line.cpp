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
  /***********************************辅助函数**************************************/
  /**
   * @brief 计算两点间的距离
   *
   * @param x1
   * @param y1
   * @param x2
   * @param y2
   * @return double
   */
  double cal_distance(double x1, double y1, double x2, double y2)
  {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
  }

  /**
   * @brief 寻找最近匹配点下标
   *
   * @param cur_x
   * @param cur_y
   * @param path
   * @param pre_match_index
   * @return
   */
  int search_match_index(const double &cur_x, const double &cur_y,
                         const std::vector<path_point> &path,
                         const int &pre_match_index)
  {
    double dist;
    double min_dist = DBL_MAX;
    int match_index = 0;
    for (int i = pre_match_index; i < path.size(); i++)
    {
      dist = cal_distance(path[i].x, path[i].y, cur_x, cur_y);
      if (dist < min_dist)
      {
        min_dist = dist;
        match_index = i;
      }
    }
    return match_index;
  }

  /**
   * @brief 通过匹配点求投影点
   *
   * @param match_point 匹配点
   * @param projection_point 投影点
   * @return path_point
   * https://zhuanlan.zhihu.com/p/429676544
   */
  path_point match_to_projection(const car_state &cur_pose,
                                 const path_point &match_point)
  {
    // 投影点其他值都与匹配点相同,求其x,y,yaw
    path_point projection_point = match_point;

    // 匹配点切向量tor
    Eigen::Matrix<double, 2, 1> tor;
    tor << cos(match_point.yaw), sin(match_point.yaw);

    // 匹配点至自车向量d
    Eigen::Matrix<double, 2, 1> d;
    d << cur_pose.x - match_point.x, cur_pose.y - match_point.y;

    // d在tor方向上的投影分量
    double e_s = tor.transpose() * d;

    // 求投影点x,y
    projection_point.x = match_point.x + e_s * cos(match_point.yaw);
    projection_point.y = match_point.y + e_s * sin(match_point.yaw);

    // 求投影点的yaw
    projection_point.yaw = match_point.yaw + match_point.cur * e_s;
    return projection_point;
  }

  /*******************************Class ReferenceLine ******************************************/

  /**
   * @brief Construct a new Reference Line:: Reference Line object
   *
   * @param lookahead_distance
   */
  ReferenceLine::ReferenceLine(double lookahead_distance)
  {
    lookahead_dist = lookahead_distance;
    match_index = 0;
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
   * @brief reference_line 平滑
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



  // /**
  //  * @brief 获取碰撞BOX，用8个点表示
  //  *
  //  * @param point
  //  * @param x_rad
  //  * @param y_rad
  //  * @return std::vector<car_state>
  //  */
  // std::vector<car_state> cal_collision_box(const car_state &point, const double &x_rad, const double &y_rad)
  // {
  //   vector<car_state> collision_box(8);
  //   double x = point.x;
  //   double y = point.y;
  //   double yaw = point.yaw;
  //   // 获取BOX边上8个点的坐标矩阵
  //   Eigen::MatrixXd position_matrix(8, 2), translation_matrix(8, 2), rotation_matrix(2, 2);

  //   position_matrix << x, y,
  //       x, y,
  //       x, y,
  //       x, y,
  //       x, y,
  //       x, y,
  //       x, y,
  //       x, y;

  //   translation_matrix << -x_rad, -y_rad,
  //       -x_rad, 0,
  //       -x_rad, y_rad,
  //       0, y_rad,
  //       x_rad, y_rad,
  //       x_rad, 0,
  //       x_rad, -y_rad,
  //       0, -y_rad;

  //   rotation_matrix << cos(yaw), sin(yaw),
  //       -sin(yaw), cos(yaw);

  //   position_matrix = translation_matrix * rotation_matrix + position_matrix;

  //   for (int i = 0; i < position_matrix.rows(); i++)
  //   {
  //     collision_box[i].x = position_matrix(i, 0);
  //     collision_box[i].y = position_matrix(i, 1);
  //     collision_box[i].z = point.z;
  //     collision_box[i].yaw = point.yaw;
  //     collision_box[i].vx = point.vx;
  //     collision_box[i].vy = point.vy;
  //     collision_box[i].v = point.v;
  //   }

  //   return collision_box;
  // }

} // namespace carla_pnc