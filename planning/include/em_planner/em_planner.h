/**
 * @file em_planner.h
 * @author czj
 * @brief
 * @version 0.1
 * @date 2023-07-17
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef EM_PLANNER_H
#define EM_PLANNER_H

#include <sys/time.h>
#include <iostream>
#include <limits>
#include <vector>
#include <algorithm>
#include <vector>
#include <cmath>
#include <future>
#include <thread>

#include <ros/ros.h>
#include "reference_line/cubic_spline.hpp"
#include "reference_line/reference_line.h"
#include "point_types.h"
#include "polynomial/quintic_polynomial.hpp"
#include "collision_detection/collision_detection.h"

namespace carla_pnc
{
    class EMPlanner
    {
    public:
        CollisionDetection collision_detection; // 碰撞检测模块
        double desired_speed = 8.0;

        EMPlanner() = default;

        EMPlanner(const CollisionDetection &collision_detection,
                  std::unordered_map<std::string, double> &dp_path_params,
                  std::unordered_map<std::string, double> &qp_path_params);

        /***********************************DP path相关**************************************/
        void dp_sampling(const FrenetPoint &initial_point);
        double calc_dppath_cost(const FrenetPoint &start, const FrenetPoint &end);
        void calc_dp_path(const FrenetPoint &initial_point);
        void dp_path_interpolation(const std::vector<FrenetPoint> &dp_path);

        /***********************************QP path相关**************************************/

        void calc_convex_space(const std::vector<FrenetPoint> &dp_final_path);

        int get_obstacle_index(const std::vector<FrenetPoint> &dp_final_path,
                               const double &ob_s);
        void calc_qp_path(const std::vector<FrenetPoint> &dp_final_path,
                          const Eigen::VectorXd &l_min,
                          const Eigen::VectorXd &l_max);

        /*************************************************************************/
        int get_cartesian_paths(std::vector<FrenetPoint> &frenet_path, Spline2D &ref_frenet);

        FrenetPath planning(Spline2D &ref_frenet,
                            const FrenetPoint &initial_point);

    private:
        double path_ds = 0.1; // 路径规划点的间隔
        /***********************************DP path相关**************************************/
        double dp_sample_l; // dp采样横向距离间隔
        double dp_sample_s; // dp采样纵向距离间隔
        int dp_sample_rows; // dp采样行数（横向）
        int dp_sample_cols; // dp采样列数（纵向）

        double dp_cost_collision; // dp碰撞代价
        double dp_cost_dl;
        double dp_cost_ddl;
        double dp_cost_dddl;
        double dp_cost_ref;
        std::vector<std::vector<FrenetPoint>> dp_sample_path; // DP采样路径
        std::vector<FrenetPoint> dp_path;                     // 动态规划获得的dp_path
        std::vector<FrenetPoint> dp_final_path;               // 插值后的dp_path

        //***********************************QP path相关**************************************/
        // QP Path的路径边界
        Eigen::VectorXd l_min;
        Eigen::VectorXd l_max;

        // QP Path cost
        double qp_cost_l;
        double qp_cost_dl;
        double qp_cost_ddl;
        double qp_cost_dddl;
        double qp_cost_ref;
        double qp_cost_end_l;
        double qp_cost_end_dl;
        double qp_cost_end_ddl;

        std::vector<FrenetPoint> qp_path; // 二次规划求得的QP_path
    };

} // carla_pnc

#endif // EM_PLANNER_H