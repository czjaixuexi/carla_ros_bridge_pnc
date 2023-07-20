/**
 * @file planning_node.h.cpp
 * @author czj
 * @brief
 * @version 0.1
 * @date 2023-05-25
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "planning_node.h"
using namespace std;

namespace carla_pnc
{
    PlanningNode::PlanningNode()
    {
        cruise_speed = 5.0;
        ros::NodeHandle n("~"); // 句柄

        n.param<string>("role_name", role_name, "ego_vehicle");
        n.param<double>("path_length", path_length, 50.0);
        n.param<bool>("planner_activate", planner_activate, true);

        n.param<string>("planning_method", planning_method, "EM");

        // collision_detection params
        n.param<double>("collision_distance", collision_distance, 1.15);

        // lattice_planner params
        n.param<double>("sample_max_time", lattice_params["sample_max_time"], 4.0);
        n.param<double>("sample_min_time", lattice_params["sample_min_time"], 2.0);
        n.param<double>("sample_time_step", lattice_params["sample_time_step"], 0.5);
        n.param<double>("sample_lat_width", lattice_params["sample_lat_width"], 3.5);
        n.param<double>("sample_width_length", lattice_params["sample_width_length"], 0.5);

        n.param<double>("w_object", lattice_params["w_object"], 1.0);
        n.param<double>("w_lon_jerk", lattice_params["w_lon_jerk"], 0.01);
        n.param<double>("w_lat_offset", lattice_params["w_lat_offset"], 10.0);
        n.param<double>("w_lat_acc", lattice_params["w_lat_acc"], 0.01);

        // reference_line smoother params(仅用于离散点平滑方法)
        n.param<double>("ref_weight_smooth", referline_params["ref_weight_smooth"], 70.0);
        n.param<double>("ref_weight_path_length", referline_params["ref_weight_path_length"], 10.0);
        n.param<double>("ref_weight_ref_deviation", referline_params["w_lat_offset"], 20.0);
        n.param<double>("x_lower_bound", referline_params["x_lower_bound"], -2.0);
        n.param<double>("x_upper_bound", referline_params["x_upper_bound"], 2.0);
        n.param<double>("y_lower_bound", referline_params["y_lower_bound"], -2.0);
        n.param<double>("y_upper_bound", referline_params["y_upper_bound"], 2.0);
        n.param<bool>("use_discrete_smooth", use_discrete_smooth, false);

        // EM planner params
        // DP path
        n.param<double>("dp_sample_l", dp_path_params["dp_sample_l"], 1.0);
        n.param<double>("dp_sample_s", dp_path_params["dp_sample_s"], 5.0);
        n.param<double>("dp_sample_rows", dp_path_params["dp_sample_rows"], 5);
        n.param<double>("dp_sample_cols", dp_path_params["dp_sample_cols"], 5);

        n.param<double>("dp_cost_collision", dp_path_params["dp_cost_collision"], 10e8);
        n.param<double>("dp_cost_dl", dp_path_params["dp_cost_dl"], 150);
        n.param<double>("dp_cost_ddl", dp_path_params["dp_cost_ddl"], 10);
        n.param<double>("dp_cost_dddl", dp_path_params["dp_cost_dddl"], 1);
        n.param<double>("dp_cost_ref", dp_path_params["dp_cost_ref"], 100);

        // QP path
        n.param<double>("qp_cost_l", qp_path_params["qp_cost_l"], 15);
        n.param<double>("qp_cost_dl", qp_path_params["qp_cost_dl"], 1500);
        n.param<double>("qp_cost_ddl", qp_path_params["qp_cost_ddl"], 10);
        n.param<double>("qp_cost_dddl", qp_path_params["qp_cost_dddl"], 1);
        n.param<double>("qp_cost_ref", qp_path_params["qp_cost_ref"], 5);
        n.param<double>("qp_cost_end_l", qp_path_params["qp_cost_end_l"], 0);
        n.param<double>("qp_cost_end_dl", qp_path_params["qp_cost_end_dl"], 0);
        n.param<double>("qp_cost_end_ddl", qp_path_params["qp_cost_end_ddl"], 0);

        // setup subscriber
        cur_pose_sub = n.subscribe("/carla/" + role_name + "/odometry", 10, &PlanningNode::callbackCarlaOdom, this);
        global_path_sub = n.subscribe("/carla/" + role_name + "/waypoints", 10, &PlanningNode::callbackGlobalPath, this);
        cruise_speed_sub = n.subscribe("/cruise_speed", 10, &PlanningNode::callbackCruiseSpeed, this);
        imu_sub = n.subscribe("/carla/" + role_name + "/imu", 10, &PlanningNode::callbackIMU, this);
        detected_objects_sub = n.subscribe("/carla/" + role_name + "/objects", 10, &PlanningNode::callbackDetectedObjects, this);

        // setup publishers
        local_waypoints_pub = n.advertise<waypoint_msgs::WaypointArray>("/reference_line/local_waypoint", 10);
        ref_path_pub = n.advertise<nav_msgs::Path>("/reference_line/ref_path", 10);
        sample_paths_pub = n.advertise<nav_msgs::Path>("/reference_line/sample_paths", 10);
        final_path_pub = n.advertise<nav_msgs::Path>("/reference_line/final_path", 10);
        history_paths_pub = n.advertise<nav_msgs::Path>("/reference_line/history_paths", 10);
        speed_marker_pub = n.advertise<visualization_msgs::Marker>("/speed_marker_text", 10);

        pre_match_index = 0;
        get_odom = false;
        detected_objects.clear();
        car_follow = false;
        first_loop = false;
    }

    /**
     * @brief carla里程计信息的回调函数, 根据里程计信息获取车辆当前的位置
     *
     * @param msg  nav_msgs/Odometry
     */
    void PlanningNode::callbackCarlaOdom(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // 坐标转换
        geometry_msgs::Quaternion odom_quat = msg->pose.pose.orientation;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(odom_quat, quat);

        // 根据转换后的四元数，获取roll pitch yaw
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        cur_pose.x = msg->pose.pose.position.x;
        cur_pose.y = msg->pose.pose.position.y;
        cur_pose.z = msg->pose.pose.position.z;
        cur_pose.yaw = yaw;

        cur_pose.vx = msg->twist.twist.linear.x;
        cur_pose.vy = msg->twist.twist.linear.y;
        cur_pose.v = sqrt(pow(cur_pose.vx, 2) + pow(cur_pose.vy, 2));

        get_odom = true;
        // ROS_INFO("Current pose: x: %.2f, y: %.2f, yaw: %.2f, x_speed:%.2f,y_speed:%.2f,v_speed:%.2f",
        //          cur_pose.x, cur_pose.y, cur_pose.yaw, cur_pose.vx, cur_pose.vy, cur_pose.v);
    }

    void PlanningNode::callbackCruiseSpeed(const std_msgs::Float32::ConstPtr &msg)
    {
        cruise_speed = msg->data;
    }

    /**
     * @brief 处理获取的全局路径
     *
     * @param msg carla的全局路径
     */
    void PlanningNode::callbackGlobalPath(const nav_msgs::Path::ConstPtr &msg)
    {
        global_path.clear();
        path_point global_path_point;
        // 将path信息转换到global_path中
        for (int i = 0; i < msg->poses.size(); i++)
        {
            global_path_point.x = msg->poses[i].pose.position.x;
            global_path_point.y = msg->poses[i].pose.position.y;
            global_path_point.z = msg->poses[i].pose.position.z;
            // 坐标转换
            geometry_msgs::Quaternion odom_quat = msg->poses[i].pose.orientation;
            tf::Quaternion quat;
            tf::quaternionMsgToTF(odom_quat, quat);
            // 根据转换后的四元数，获取roll pitch yaw
            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            global_path_point.yaw = yaw;

            if (global_path.size() == 0)
            {
                global_path.push_back(global_path_point);
            }
            // 防止路径点重合
            else if ((global_path.size() > 0) &&
                     abs(global_path_point.x - global_path[global_path.size() - 1].x) >
                         0.001 &&
                     abs(global_path_point.y - global_path[global_path.size() - 1].y) >
                         0.001)
            {
                global_path.push_back(global_path_point);
                // ROS_INFO("global_x:%.f,global_y:%.f",path_point.x,path_point.y);
            }
        }
        ROS_INFO("Received global_path successfully, The size of global is :%zu", global_path.size());
    }

    /**
     * @brief 获取IMU信息(求加速度)
     *
     * @param msg
     */
    void PlanningNode::callbackIMU(const sensor_msgs::Imu::ConstPtr &msg)
    {
        // cur_pose.angular_velocity =
        //     msg->angular_velocity.z; // 平面角速度(绕z轴转动的角速度)
        cur_pose.ax = msg->linear_acceleration.x;
        cur_pose.ay = msg->linear_acceleration.y;
        cur_pose.a = sqrt(cur_pose.ax * cur_pose.ax +
                          cur_pose.ay * cur_pose.ay); // 加速度
    }

    /**
     * @brief 获取探测到的目标信息
     * 信息详细说明：https://docs.ros.org/en/melodic/api/derived_object_msgs/html/msg/ObjectArray.html
     * @param msg
     */
    void PlanningNode::callbackDetectedObjects(
        const derived_object_msgs::ObjectArray::ConstPtr &msg)
    {
        // ROS_INFO("Received dectected objects successfully ...");

        // 转化objects数据
        detected_objects.resize(msg->objects.size());

        for (int i = 0; i < msg->objects.size(); i++)
        {
            Obstacle ob;

            ob.point.x = msg->objects[i].pose.position.x;
            ob.point.y = msg->objects[i].pose.position.y;
            ob.point.z = msg->objects[i].pose.position.z;
            ob.point.vx = msg->objects[i].twist.linear.x;
            ob.point.vy = msg->objects[i].twist.linear.y;
            ob.point.v = sqrt(pow(ob.point.vx, 2) + pow(ob.point.vy, 2));
            // ROS_INFO("leader car speedvx: %.2f , position,vy:%.2f,v:%.2f",object.vx,object.vy,object.v);
            // 坐标转换
            geometry_msgs::Quaternion ob_quat = msg->objects[i].pose.orientation;
            tf::Quaternion quat;
            tf::quaternionMsgToTF(ob_quat, quat);

            // 根据转换后的四元数，获取roll pitch yaw
            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

            ob.point.yaw = yaw;

            ob.x_rad = msg->objects[i].shape.dimensions[0] / 2.0;
            ob.y_rad = msg->objects[i].shape.dimensions[1] / 2.0;
            double z_rad = 0.8;

            detected_objects[i] = ob;
        }
    }

    /**
     * @brief 主循环
     *
     */
    void PlanningNode::MainLoop()
    {
        ros::Rate rate(10.0);

        /***********************************路径规划**************************************/

        // carla_pnc::ReferenceLine reference_line(path_length,
        //                                         ref_weight_smooth,
        //                                         ref_weight_path_length,
        //                                         ref_weight_ref_deviation,
        //                                         x_lower_bound,
        //                                         x_upper_bound,
        //                                         y_lower_bound,
        //                                         y_upper_bound);

        carla_pnc::ReferenceLine reference_line(path_length,
                                                referline_params);

        // double prev_timestamp = ros::Time::now().toSec();

        int start_index = pre_match_index;

        while (ros::ok())
        {
            // ROS_INFO("w_object%.2f, w_lon_jerk%.2f,  w_lat_offset%.2f ,w_lat_acc%.2f", w_object, w_lon_jerk, w_lat_offset, w_lat_acc);
            // double current_timestamp = ros::Time::now().toSec();
            // 获取定位信息后开始规划
            if (get_odom && !global_path.empty())
            {
                start_index = pre_match_index;
                /***********************************Step1 从全局路径获取局部规划参考线**************************************/
                local_path = reference_line.local_path_truncation(cur_pose, global_path, start_index);
                // ROS_INFO("The pre_match_index is %d:", reference_line.match_index);
                // ROS_INFO("The size of Global path is %d:", global_path.size());
                pre_match_index = reference_line.match_index;
                // prev_timestamp = current_timestamp;

                if (local_path.size() > 1)
                {
                    /***********************************Step2 参考线平滑**************************************/

                    // 用Spline2D构建平滑的Frenet曲线坐标系
                    std::vector<double> x_set, y_set;
                    for (int i = 0; i < local_path.size(); i++)
                    {
                        x_set.push_back(local_path[i].x);
                        y_set.push_back(local_path[i].y);
                    }
                    Spline2D ref_frenet(x_set, y_set);

                    // 离散点平滑
                    if (use_discrete_smooth)
                    {
                        // ROS_INFO("discrete_smooth");
                        ref_path = reference_line.discrete_smooth(local_path);
                    }
                    // cubic Spline平滑
                    else
                    {
                        // ROS_INFO("curblic Spline smooth");
                        ref_path = reference_line.smoothing(ref_frenet, local_path);
                    }
                    /***********************************Step3 确认规划起点，并投影到Frenet坐标系中得到（s0,l0）**************************************/
                    if (planner_activate)
                    {
                        car_state global_initial_point; // 全局坐标系下的规划起始点

                        // 若是第一次运行，或上个周期规划轨迹不存在，则用车辆则用车辆当前位置作为本周期的规划起点，
                        if (first_loop || pre_final_path.frenet_path.size() < 5)
                        {
                            global_initial_point = cur_pose;
                            first_loop = false;
                        }
                        /*
                         * 比较车辆当前位置与上个周期current_time所匹配的轨迹点
                         * 若相差过大，则根据车辆当前位置，通过运动学公式后获取推一个周期的位置，作为本周期的规划起点
                         */
                        else if (fabs(cur_pose.x - pre_final_path.frenet_path[0].x) > 2.0 ||
                                 fabs(cur_pose.y - pre_final_path.frenet_path[0].y) > 0.5)
                        {
                            // 运动学公式后推100ms的位置
                            car_state next_pose;
                            double dt = 0.1;
                            next_pose = cur_pose;

                            // 车身速度转全局速度
                            double vx = cur_pose.vx * cos(cur_pose.yaw) - cur_pose.vy * sin(cur_pose.yaw);
                            double vy = cur_pose.vy * sin(cur_pose.yaw) + cur_pose.vy * cos(cur_pose.yaw);

                            next_pose.x = cur_pose.x + vx * dt + 0.5 * vx * dt * dt;
                            next_pose.y = cur_pose.y + vy * dt + 0.5 * vy * dt * dt;

                            next_pose.vx = cur_pose.vx + cur_pose.ax * dt;
                            next_pose.vy = cur_pose.vy + cur_pose.ay * dt;
                            next_pose.v = sqrt(pow(next_pose.vx, 2) + pow(next_pose.vy, 2));

                            global_initial_point = next_pose;
                        }
                        // 若相差不大,则选用current_time+100ms的轨迹点作为规划起点进行规划，作为本周期的规划起点
                        else if (pre_final_path.frenet_path.size())
                        {
                            if (planning_method == "Lattice")
                            {
                                global_initial_point = pre_final_path.frenet_path[5];
                            }
                            else
                            {   
                                // EM Planner目前只完成路径规划部分，还没有时间信息，暂时先用运动学推导位置
                                // 运动学公式后推100ms的位置
                                car_state next_pose;
                                double dt = 0.1;
                                next_pose = cur_pose;

                                // 车身速度转全局速度
                                double vx = cur_pose.vx * cos(cur_pose.yaw) - cur_pose.vy * sin(cur_pose.yaw);
                                double vy = cur_pose.vy * sin(cur_pose.yaw) + cur_pose.vy * cos(cur_pose.yaw);

                                next_pose.x = cur_pose.x + vx * dt + 0.5 * vx * dt * dt;
                                next_pose.y = cur_pose.y + vy * dt + 0.5 * vy * dt * dt;

                                next_pose.vx = cur_pose.vx + cur_pose.ax * dt;
                                next_pose.vy = cur_pose.vy + cur_pose.ay * dt;
                                next_pose.v = sqrt(pow(next_pose.vx, 2) + pow(next_pose.vy, 2));

                                global_initial_point = next_pose;
                            }

                            // global_initial_point = cur_pose;
                        }

                        // 将规划起点投影到Frenet坐标系中
                        // // 计算匹配点下标
                        // int frenet_match_index = search_match_index(global_initial_point.x, global_initial_point.y, ref_path, 0);

                        // // 通过匹配点求投影点
                        // path_point projection_point = match_to_projection(global_initial_point, ref_path[frenet_match_index]);

                        // // 计算Frenet坐标
                        // FrenetPoint frenet_initial_point = Cartesian2Frenet(global_initial_point, projection_point);

                        FrenetPoint frenet_initial_point = calc_frenet(global_initial_point, ref_path);
                        // ROS_INFO("Get Initial Point Successfully");

                        /***********************************Step4 Lattice Planner**************************************/

                        CollisionDetection collision_detection(detected_objects, collision_distance, ref_path); // 碰撞检测模块

                        if (planning_method == "Lattice")
                        {
                            // LatticePlanner planner(sample_max_time, sample_min_time, sample_time_step,
                            //                        sample_lat_width, sample_width_length,
                            //                        w_object, w_lon_jerk,
                            //                        w_lat_offset, w_lat_acc,
                            //                        cruise_speed,
                            //                        collision_detection);

                            LatticePlanner planner(lattice_params,
                                                   cruise_speed,
                                                   collision_detection);
                            car_follow = false;
                            FrenetPoint leader_car;
                            // 将动态障碍物投影到Frenet坐标系中
                            for (Obstacle &object : collision_detection.dynamic_obstacle_list)
                            {
                                // 计算匹配点下标
                                int frenet_match_index = search_match_index(object.point.x, object.point.y, ref_path, 0);

                                // 通过匹配点求投影点
                                path_point pro_point = match_to_projection(object.point, ref_path[frenet_match_index]);

                                // ROS_INFO("leader car speed:%.2f , position,x:%.2f,y:%.2f", object.point.v, object.point.x, object.point.y);

                                // 计算障碍物的Frenet坐标
                                FrenetPoint ob_fp = Cartesian2Frenet(object.point, pro_point);
                                // ROS_INFO("leader car speed:%.2f , position,x:%.2f,y:%.2f",ob_fp.s_d,ob_fp.s,ob_fp.l);
                                // ROS_INFO("self car speed:%.2f , position,x:%.2f,y:%.2f",frenet_initial_point.s_d,frenet_initial_point.s,frenet_initial_point.l);

                                // 跟车判定条件： 目标大于循环车速 * 0.6
                                /**
                                 * 跟车判定条件：
                                 * 1.在本车前方
                                 * 2.横向距离小于2.0(本车道内)
                                 * 3.纵向距离小于 3.0 * 巡航车速
                                 * 4.纵向距离大于安全距离(在Planner中实现)
                                 * 4.车速大于等于0.6*巡航车速
                                 */
                                if (ob_fp.s > frenet_initial_point.s &&
                                    fabs(ob_fp.l - frenet_initial_point.l) < 2.0 &&
                                    fabs(ob_fp.s - frenet_initial_point.s) < 3.0 * cruise_speed &&
                                    object.point.v > 0.6 * cruise_speed)
                                {
                                    // ROS_INFO("leader car speed:%.2f , position,x:%.2f,y:%.2f",ob_fp.s_d,ob_fp.s,ob_fp.l);
                                    car_follow = true;
                                    leader_car = ob_fp;
                                    break;
                                }
                            }
                            // 所有的采样轨迹，用于可视化
                            sample_paths = planner.get_planning_paths(ref_frenet, frenet_initial_point, leader_car, car_follow);

                            // 获取最佳轨迹
                            final_path = planner.planning(ref_frenet, frenet_initial_point, leader_car, car_follow);

                            pre_final_path = final_path; // 用于存储上一个周期的轨迹

                            history_paths.push_back(final_path); // 历史轨迹，用于可视化

                            ROS_INFO("Selected best path successfully,the size is%d", final_path.size_);
                            // ROS_INFO("Selected paths successfully,the size is%zu", sample_paths.size());

                            // for(unsigned int i = 0; i< final_path.size_;i++){
                            //     ROS_INFO("best path____x:%.2f,  y:%.2f", final_path.frenet_path[i].x,final_path.frenet_path[i].y);
                            // }
                        }
                        else if (planning_method == "EM")
                        {
                            EMPlanner planner(collision_detection,
                                              dp_path_params,
                                              qp_path_params);
                                              
                            final_path = planner.planning(ref_frenet, frenet_initial_point);
                            pre_final_path = final_path;         // 用于存储上一个周期的轨迹
                            history_paths.push_back(final_path); // 历史轨迹，用于可视化
                            ROS_INFO("Selected best path successfully,the size is%d", final_path.size_);
                        }
                        /*******************************visualization******************************************/
                        // 最优规划轨迹Rviz可视化
                        final_path_visualization(final_path);

                        // 采样轨迹Rviz可视化
                        sample_paths_visualization(sample_paths);

                        // 显示障碍物的速度
                        object_speed_visualization(collision_detection.detected_objects);
                    }
                }
            }
            // 参考线可视化
            ref_path_visualization(ref_path);

            // 历史规划轨迹Rviz可视化
            history_path_visualization(history_paths);

            /***********************************Publish**************************************/
            // 发布路径规划点给控制器
            waypoint_msgs::WaypointArray local_waypoints;
            local_waypoints.header.frame_id = "map";

            if (planner_activate)
            {
                // ROS_INFO("planner activates");
                for (int i = 0; i < final_path.size_; i++)
                {
                    waypoint_msgs::Waypoint point;
                    point.pose.pose.position.x = final_path.frenet_path[i].x;
                    point.pose.pose.position.y = final_path.frenet_path[i].y;
                    point.twist.twist.linear.x = final_path.frenet_path[i].v;
                    // point.twist.twist.linear.x = cruise_speed;
                    // ROS_INFO("The target_v is vx:%.2f,vy:%.2f,v:%.2f", fabs(final_path.frenet_path[i].v * cos(final_path.frenet_path[i].yaw)),
                    //          final_path.frenet_path[i].v * sin(final_path.frenet_path[i].yaw),
                    //          final_path.frenet_path[i].v);
                    // point.twist.twist.linear.y = cruise_speed;
                    local_waypoints.waypoints.push_back(point);
                }
            }
            else
            {
                // ROS_INFO("planner unactivates");
                for (int i = 0; i < ref_path.size(); i++)
                {
                    waypoint_msgs::Waypoint point;
                    point.pose.pose.position.x = ref_path[i].x;
                    point.pose.pose.position.y = ref_path[i].y;
                    point.twist.twist.linear.x = cruise_speed;
                    local_waypoints.waypoints.push_back(point);
                }
            }
            // ROS_INFO("The size of local path is:%zu", ref_path.size());
            local_waypoints_pub.publish(local_waypoints);

            ros::spinOnce();
            rate.sleep();
            // ROS_INFO("The iteration end.");
        }
    }

    /***********************************visualization**************************************/

    /**
     * @brief 发布参考线用于Rviz可视化
     *
     * @param ref_path
     */
    void PlanningNode::ref_path_visualization(const std::vector<path_point> &ref_path)
    {
        nav_msgs::Path path;
        path.header.frame_id = "map";
        for (int i = 0; i < ref_path.size(); i++)
        {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = ref_path[i].x;
            pose.pose.position.y = ref_path[i].y;
            // pose.pose.position.z = final_path.z[i];

            geometry_msgs::Quaternion quat =
                tf::createQuaternionMsgFromYaw(ref_path[i].yaw);
            pose.pose.orientation = quat;
            path.poses.push_back(pose);
        }
        ref_path_pub.publish(path);
    }

    /**
     * @brief 发布最优规划轨迹用于Rviz可视化
     *
     * @param final_path
     */
    void PlanningNode::final_path_visualization(const FrenetPath &final_path)
    {
        nav_msgs::Path best_path;
        best_path.header.frame_id = "map";
        for (int i = 0; i < final_path.size_; i++)
        {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = final_path.frenet_path[i].x;
            pose.pose.position.y = final_path.frenet_path[i].y;
            // pose.pose.position.z = final_path.z[i];

            geometry_msgs::Quaternion quat =
                tf::createQuaternionMsgFromYaw(final_path.frenet_path[i].yaw);
            pose.pose.orientation = quat;
            best_path.poses.push_back(pose);
        }
        final_path_pub.publish(best_path);
    }

    /**
     * @brief 发布采样轨迹用于Rviz可视化
     *
     * @param sample_paths
     */
    void PlanningNode::sample_paths_visualization(const std::vector<FrenetPath> &sample_paths)
    {
        //
        nav_msgs::Path path;
        path.header.frame_id = "map";
        path.header.stamp = ros::Time::now();
        for (auto final_path : sample_paths)
        {
            for (int i = 0; i < final_path.size_; i += 10)
            {
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = final_path.frenet_path[i].x;
                pose.pose.position.y = final_path.frenet_path[i].y;
                // pose.pose.position.z = final_path.z[i];

                geometry_msgs::Quaternion quat =
                    tf::createQuaternionMsgFromYaw(final_path.frenet_path[i].yaw);
                pose.pose.orientation = quat;
                path.poses.push_back(pose);
            }
            sample_paths_pub.publish(path);
        }
    }

    /**
     * @brief 发布历史规划轨迹用于Rviz可视化
     *
     * @param history_path
     */
    void PlanningNode::history_path_visualization(const std::vector<FrenetPath> &history_paths)
    {
        nav_msgs::Path history_path;
        history_path.header.frame_id = "map";
        history_path.header.stamp = ros::Time::now();
        for (auto final_path : history_paths)
        {
            int count = 0;
            for (int i = 0; i < final_path.size_ && count < 1; i++, count++)
            {
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = final_path.frenet_path[i].x;
                pose.pose.position.y = final_path.frenet_path[i].y;
                // pose.pose.position.z = final_path.z[i];

                geometry_msgs::Quaternion quat =
                    tf::createQuaternionMsgFromYaw(final_path.frenet_path[i].yaw);
                pose.pose.orientation = quat;
                history_path.poses.push_back(pose);
            }
            history_paths_pub.publish(history_path);
        }
    }

    /**
     * @brief 用于显示障碍物的速度
     *
     * @param detected_objects
     */
    void PlanningNode::object_speed_visualization(const std::vector<Obstacle> &detected_objects)
    {
        int id_ = 0;
        for (auto &object : detected_objects)
        {
            visualization_msgs::Marker speed_marker;
            speed_marker.header.frame_id = "map";
            speed_marker.header.stamp = ros::Time::now();
            speed_marker.ns = "planning/speed_marker";
            speed_marker.action = visualization_msgs::Marker::ADD;
            speed_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

            speed_marker.pose.orientation.w = 1.0;
            speed_marker.id = id_++;

            speed_marker.scale.x = 1.5;
            speed_marker.scale.y = 1.5;
            speed_marker.scale.z = 1.5;

            speed_marker.color.b = 0;
            speed_marker.color.g = 0;
            speed_marker.color.r = 255;
            speed_marker.color.a = 1;

            std::stringstream ss;
            ss << std::fixed << std::setprecision(2) << object.point.v; // 设置精度为2位

            speed_marker.text = ss.str() + "m/s";

            speed_marker.pose.position.x = object.point.x;
            speed_marker.pose.position.y = object.point.y;
            speed_marker.pose.position.z = object.point.z + 1;

            speed_marker_pub.publish(speed_marker);
        }
    }

} // carla_pnc
