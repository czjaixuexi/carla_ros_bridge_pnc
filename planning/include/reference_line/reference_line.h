/**
 * @file reference_line.h
 * @author czj
 * @brief
 * @version 0.1
 * @date 2023-05-22
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef REFERENCELINE_H
#define REFERENCELINE_H

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include "point_types.h"
#include "reference_line/cubic_spline.hpp"

namespace carla_pnc
{
  /***********************************辅助函数**************************************/
  double cal_distance(double x1, double y1, double x2, double y2);

  int search_match_index(const double &cur_x, const double &cur_y,
                         const std::vector<path_point> &waypoints,
                         const int &pre_match_index);


  path_point match_to_projection(const car_state &cur_pose,
                                 const path_point &match_point);

  // std::vector<car_state> cal_collision_box(const car_state &point, const double &x_rad, const double &y_rad);

  class ReferenceLine
  {
  public:
    double lookahead_dist;
    int match_index;
    ReferenceLine(double lookahead_distance);

    int search_target_index(const double &cur_x, const double &cur_y,
                            const std::vector<path_point> &waypoints,
                            const double &lookahead_distance);

    std::vector<path_point> local_path_truncation(const car_state &cur_pose,
                                                  const std::vector<path_point> &global_path,
                                                  const int &pre_match_index);

    std::vector<path_point> smoothing(Spline2D &ref_frenet, const std::vector<path_point> &local_path);
  };
} // namespace carla_pnc
#endif // REFERENCELINE_H
