/**
 * @file collision_detection.cpp
 * @author czj
 * @brief
 * @version 0.1
 * @date 2023-06-17
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "collision_detection/collision_detection.h"
using namespace std;

namespace carla_pnc
{
    /**
     * @brief Construct a new carla pnc::CollisionDetection::CollisionDetection object
     *
     * @param detected_objects
     * @param collision_distance
     */
    CollisionDetection::CollisionDetection(const std::vector<Obstacle>  &detected_objects,
                                           const double &collision_distance)
    {
        this->detected_objects = detected_objects;
        this->collision_distance = collision_distance;
        static_obstacle_list.clear();
        dynamic_obstacle_list.clear();
        obstacle_classification(this->detected_objects);
    }

    /**
     * @brief 计算障碍物碰撞BOX，并分类（动态/静态）
     *
     * @param detected_objects
     */
    void CollisionDetection::obstacle_classification(std::vector<Obstacle> &detected_objects)
    {
        for (Obstacle &obstacle : detected_objects)
        {
            // 计算碰撞矩阵
            cal_collision_box(obstacle);
            // ROS_INFO("Get collision_box successfully");
            if (obstacle.point.v > 0.2)
            {
                dynamic_obstacle_list.push_back(obstacle);
            }
            else
            {
                static_obstacle_list.push_back(obstacle);
            }
        }
    }

    /**
     * @brief 获取碰撞BOX，用8个点表示
     *
     * @param obstacle
     */
    void CollisionDetection::cal_collision_box(Obstacle &obstacle)
    {
        vector<car_state> collision_box(8);
        double x = obstacle.point.x;
        double y = obstacle.point.y;
        double yaw = obstacle.point.yaw;
        double x_rad = obstacle.x_rad;
        double y_rad = obstacle.y_rad;

        // 获取BOX边上8个点的坐标矩阵
        Eigen::MatrixXd position_matrix(8, 2), translation_matrix(8, 2), rotation_matrix(2, 2);

        // 旋转矩阵
        position_matrix << x, y,
            x, y,
            x, y,
            x, y,
            x, y,
            x, y,
            x, y,
            x, y;

        translation_matrix << -x_rad, -y_rad,
            -x_rad, 0,
            -x_rad, y_rad,
            0, y_rad,
            x_rad, y_rad,
            x_rad, 0,
            x_rad, -y_rad,
            0, -y_rad;

        rotation_matrix << cos(yaw), sin(yaw),
            -sin(yaw), cos(yaw);

        position_matrix = translation_matrix * rotation_matrix + position_matrix;

        for (int i = 0; i < position_matrix.rows(); i++)
        {
            collision_box[i].x = position_matrix(i, 0);
            collision_box[i].y = position_matrix(i, 1);
            collision_box[i].z = obstacle.point.z;
            collision_box[i].yaw = obstacle.point.yaw;
            collision_box[i].vx = obstacle.point.vx;
            collision_box[i].vy = obstacle.point.vy;
            collision_box[i].v = obstacle.point.v;
        }
        obstacle.collision_box = collision_box;
    }

    /**
     * @brief 碰撞检测并计算碰撞代价
     *
     * @param path 规划的frenet轨迹
     * @param leader_point 跟车目标
     * @param car_following 跟车状态
     * @return true
     * @return false
     */
    bool CollisionDetection::check_collision(FrenetPath &path,
                                             const FrenetPoint &leader_point,
                                             const bool &car_following)
    {
        // 遍历每个障碍物
        for (Obstacle obstacle : detected_objects)
        {   
            // ROS_INFO("Obstacle point x:%.2f,y:%.2f",obstacle.point.x,obstacle.point.y);

            // 遍历box中的每个点
            for (car_state box_point : obstacle.collision_box)
            {   
                // ROS_INFO("box point x:%.2f, y:%.2f",box_point.x,box_point.y);
                // 遍历路径上的每个点
                for (unsigned int i = 0; i < path.size_; i++)
                {
                    double dist = cal_distance(path.frenet_path[i].x, path.frenet_path[i].y,
                                               box_point.x, box_point.y);
                    
                    // 计算碰撞cost (不计算跟车目标的碰撞cost)
                    if (dist < 3.5 &&
                        !(car_following && cal_distance(obstacle.point.x, obstacle.point.y, leader_point.x, leader_point.y) < 2.0))
                    {   

                        path.cost += dist/3.0;
                    };

                    if (dist <= collision_distance)
                    {
                        return false;
                    }
                }
            }
        }
        return true;
    }

} // carla_pnc