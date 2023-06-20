/**
 * @file controller_node.h
 * @author czj
 * @brief
 * @version 0.1
 * @date 2023-06-07
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef CONTROLLER_NODE_H
#define CONTROLLER_NODE_H
#include <cmath>
#include <iostream>
#include <math.h>
#include <string>

// ros
#include "ros/ros.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/transform_datatypes.h"

// msg
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"

#include "carla_msgs/CarlaEgoVehicleControl.h"

#include "waypoint_msgs/Waypoint.h"
#include "waypoint_msgs/WaypointArray.h"


#include <Eigen/Eigen>

#include "controller.h"

namespace carla_pnc
{
    class ControllerNode
    {
    public:
        ControllerNode();
        void MainLoop();

    protected:
        std::string role_name, control_method;
        
        double k_pure; //PurePursuit"增益系数
        double k_cte;  //Stanley"增益系数
        double Q_ed, Q_ed_dot, Q_ephi, Q_ephi_dot, R_value; //LQR Q R矩阵参数
        double kp, ki, kd; //纵向PID

        car_state cur_pose;                     // 车辆当前状态
        std::vector<car_state> local_waypoints; // 局部规划路径点
        nav_msgs::Path path_msg;                // 行驶路径信息

        /***********************************Sbuscriber**************************************/
        ros::Subscriber cur_pose_sub;   // 订阅`/carla/<ROLE NAME>/odometry`, 获取车辆当前状态
        ros::Subscriber local_path_sub; // 订阅`/reference_line/local_waypoint`，获取路径规划轨迹点

        /***********************************Publisher**************************************/
        ros::Publisher path_pub;        // 发布车辆行驶路径信息
        ros::Publisher control_cmd_pub; // 发布控制命令
        // ros::Publisher path_pub;     // 发布参考线路径信息

        /***********************************callback**************************************/
        void callbackCarlaOdom(const nav_msgs::Odometry::ConstPtr &msg);

        void callbackLocalPath(const waypoint_msgs::WaypointArray::ConstPtr &msg);
    };

} // carla_pnc

#endif // CONTROLLER_NODE_H
