/**
 * @file common.h
 * @author czj
 * @brief planning模块中通用的辅助函数
 * @version 0.1
 * @date 2023-07-18
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef COMMON_H
#define COMMON_H

#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <point_types.h>
#include <Eigen/Eigen>
#include "OsqpEigen/OsqpEigen.h"

namespace carla_pnc
{

    /***********************************辅助函数**************************************/
    double cal_distance(double x1, double y1, double x2, double y2);

    int search_match_index(const double &cur_x, const double &cur_y,
                           const std::vector<path_point> &waypoints,
                           const int &pre_match_index);

    path_point match_to_projection(const car_state &cur_pose,
                                   const path_point &match_point);

    // Cartesian转Frenet
    FrenetPoint Cartesian2Frenet(const car_state &global_point,
                                 const path_point &projection_point);

    FrenetPoint calc_frenet(const car_state &global_point,
                            const std::vector<path_point> &ref_path);

} // carla_pnc

#endif // COMMON_H