/**
 * @file collision_detection.h
 * @author czj
 * @brief
 * @version 0.1
 * @date 2023-06-17
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef COLLISION_DETECTION_H
#define COLLISION_DETECTION_H

#include <cfloat>
#include <cmath>
#include <iostream>
#include <vector>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include "point_types.h"
#include "reference_line/reference_line.h"

namespace carla_pnc
{

    class Obstacle
    {
    public:
        car_state point;
        double x_rad;                         // x方向半径
        double y_rad;                         // y方向半径
        std::vector<car_state> collision_box; // 碰撞BOX，用8个点表示(4个顶点+4条边的中点)
    };

    class CollisionDetection
    {
    public:
        double collision_distance; // 碰撞距离
        std::vector<Obstacle> detected_objects;
        std::vector<Obstacle> static_obstacle_list;
        std::vector<Obstacle> dynamic_obstacle_list;
        
        CollisionDetection() = default;
        
        CollisionDetection(const std::vector<Obstacle> &detected_objects,
                           const double &collision_distance);

        void obstacle_classification(std::vector<Obstacle> &detected_objects);

        void cal_collision_box(Obstacle &object);

        bool check_collision(FrenetPath &path,
                             const FrenetPoint &leader_point,
                             const bool &car_following);
    };

} // carla_pnc

#endif // COLLISION_DETECTION_H