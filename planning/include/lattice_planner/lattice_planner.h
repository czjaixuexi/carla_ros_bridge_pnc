/**
 * @file lattice_planner.h
 * @author czj
 * @brief
 * @version 0.1
 * @date 2023-06-13
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef LATTICE_PLANNER_H
#define LATTICE_PLANNER_H

#include <sys/time.h>
#include <iostream>
#include <limits>
#include <vector>
#include <queue>

#include <ros/ros.h>

#include "reference_line/cubic_spline.hpp"
#include "reference_line/reference_line.h"
#include "point_types.h"
#include "polynomial/quartic_polynomial.hpp"
#include "polynomial/quintic_polynomial.hpp"
#include "collision_detection/collision_detection.h"

namespace carla_pnc
{
    // Cartesian转Frenet
    FrenetPoint Cartesian2Frenet(const car_state &global_point,
                                 const path_point &projection_point);


    class cmp
    {
    public:
        bool
        operator()(const FrenetPath &a, const FrenetPath &b)
        {
            return a.cost > b.cost;
        }
    };
    class LatticePlanner
    {
    public:
        // 采样相关params
        double sample_max_time;     // 最大采样时间
        double sample_min_time;     // 最小采样时间
        double sample_time_step;    // 采样时间step
        double sample_lat_width;    // 采样横向距离
        double sample_width_length; // 采样横向距离间隔

        // 代价函数权重params
        double w_object;     // 纵向目标代价
        double w_lon_jerk;   // 纵向舒适代价
        double w_lat_offset; // 横向偏离代价
        double w_lat_acc;    // 横向舒适代价

        double desired_speed;                   // 期望速度
        CollisionDetection collision_detection; // 碰撞检测模块

        double cruise_speed; // 巡航车速

        FrenetPath best_path;
        FrenetPath pre_best_path;

        LatticePlanner() = default;

        LatticePlanner(const double &sample_max_time, const double &sample_min_time, const double &sample_time_step,
                       const double &sample_lat_width, const double &sample_width_length,
                       const double &w_object, const double &w_lon_jerk,
                       const double &w_lat_offset, const double &w_lat_acc,
                       const double &cruise_speed,
                       const CollisionDetection &collision_detection);

        /***********************************纵向cost计算**************************************/
        double calc_lon_objective_cost(const FrenetPath &fp, const double target_speed);

        double calc_lon_jerk_cost(const FrenetPath &fp);

        // TO-DO 先不加看看效果
        double calc_centri_acc_cost(const FrenetPath &fp);

        /***********************************横向cost计算**************************************/

        double calc_lat_offset_cost(const FrenetPath &fp, const FrenetPoint &initial_point);

        double calc_lat_acc_cost(const FrenetPath &fp);

        /***********************************横纵向轨迹离散值计算**************************************/
        void calc_lon_values(QuarticPolynomial &lon_qp, FrenetPoint &fpoint);
        void calc_lon_values(QuinticPolynomial &lon_qp, FrenetPoint &fpoint);

        void calc_lat_values(QuinticPolynomial &lat_qp, FrenetPoint &fpoint);

        /***********************************轨迹采样**************************************/
        std::vector<FrenetPath> sampling_cruising_frenet_paths(const FrenetPoint &initial_point);

        std::vector<FrenetPath> sampling_following_frenet_paths(const FrenetPoint &initial_point, const FrenetPoint &leader_point);

        void get_cartesian_paths(std::vector<FrenetPath> &fp_list, Spline2D &ref_frenet);

        // bool check_collision(FrenetPath &path,
        //                      const std::vector<std::vector<car_state>> &obstacles_list);

        // 根据cost排序的小根堆
        std::priority_queue<FrenetPath, std::vector<FrenetPath>, cmp>
        get_valid_paths(std::vector<FrenetPath> &fp_list,
                        const FrenetPoint &leader_point,
                        const bool &car_following);

        // std::vector<FrenetPath> get_valid_paths(const std::vector<FrenetPath> &fp_list, const Vec_Poi &ob);

        FrenetPath planning(Spline2D &ref_frenet,
                            const FrenetPoint &initial_point,
                            const FrenetPoint &leader_point,
                            const bool &car_following);

        std::vector<FrenetPath> get_planning_paths(Spline2D &ref_frenet,
                                                   const FrenetPoint &initial_point,
                                                   const FrenetPoint &leader_point,
                                                   const bool &car_following);
    };

} // carla_pnc

#endif // LATTICE_PLANNER_H