/**
 * @file controller_node.cpp
 * @author czj
 * @brief
 * @version 0.1
 * @date 2023-06-07
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "controller_node.h"

using namespace std;

namespace carla_pnc
{
  ControllerNode::ControllerNode()
  {
    ros::NodeHandle n("~"); // 句柄
    n.param<string>("role_name", role_name, "ego_vehicle");
    n.param<string>("control_method", control_method, "LQR");

    n.param<double>("k_pure", k_pure, 1.0); // PurePuresuit增益系数
    n.param<double>("k_cte", k_cte, 3.0);   // Stanley增益系数

    // 纵向PID参数
    n.param<double>("kp", kp, 0.85);
    n.param<double>("ki", ki, 0.02);
    n.param<double>("kd", kd, 0.1);

    // LQR Q  R矩阵权重
    n.param<double>("Q_ed", Q_ed, 67.0);
    n.param<double>("Q_ed_dot", Q_ed_dot, 1.0);
    n.param<double>("Q_ephi", Q_ephi, 70.0);
    n.param<double>("Q_ephi_dot", Q_ephi_dot, 1.0);
    n.param<double>("R_value", R_value, 35.0);

    // setup subscriber
    cur_pose_sub = n.subscribe("/carla/" + role_name + "/odometry", 10, &ControllerNode::callbackCarlaOdom, this);
    local_path_sub = n.subscribe("/reference_line/local_waypoint", 10, &ControllerNode::callbackLocalPath, this);

    // setup publishers
    path_pub = n.advertise<nav_msgs::Path>("/trajectory", 10);
    // ref_pub = n.advertise<nav_msgs::Path>("/ref_trajectory", 10);
    control_cmd_pub = n.advertise<carla_msgs::CarlaEgoVehicleControl>(
        "/carla/" + role_name + "/vehicle_control_cmd", 10);
  }

  /**
   * @brief 主循环
   *
   */
  void ControllerNode::MainLoop()
  {
    double frequency = 50.0;
    ros::Rate rate(frequency);
    std::vector<double> prev_p(2);

    // Q矩阵
    Eigen::Matrix4d Q;
    Q(0, 0) = Q_ed;
    Q(1, 1) = Q_ed_dot;
    Q(2, 2) = Q_ephi;
    Q(3, 3) = Q_ephi_dot;

    // R矩阵
    Eigen::Matrix<double, 1, 1> R;
    R(0, 0) = R_value;
    if(control_method == "LQR")
    {
     CreateLqrOffline(Q,R);}

    carla_pnc::Controller mc(control_method,
                             k_pure, k_cte,
                             Q, R,
                             kp, ki, kd,
                             lqr_k_table);
    vector<double> cmd(3);
    vector<double> pre_cmd(3);
    while (ros::ok())
    {
      // ROS_INFO("start Inteartion");
      double current_timestamp = ros::Time::now().toSec();

      double dist = hypot(cur_pose.x - prev_p[0], cur_pose.y - prev_p[1]);

      if (dist > 10.0)
      {
        mc.reset_all_previous();
        local_waypoints.clear();
        // cout << "Restarting ego car" << endl;
      }

      prev_p[0] = cur_pose.x;
      prev_p[1] = cur_pose.y;

      /***********************************控制主体**************************************/
      if (local_waypoints.size() > 1)
      {
        mc.update_waypoints(local_waypoints);
        // cout<<"更新路径点成功"<<endl;
        mc.update_car_state(cur_pose, current_timestamp);
        // cout<<"更新状态成功"<<endl;
        mc.update_controls(frequency);
        // cout<<"更新控制成功"<<endl;
        cmd = mc.get_commands();
        pre_cmd = cmd;
      }
      else
      {
        cmd = {0, 0, 1.0};
      }

      ROS_INFO("throttle: %2f, steer: %2f, brake: %2f", cmd[0], cmd[1], cmd[2]);

      /***********************************发布控制指令**************************************/
      carla_msgs::CarlaEgoVehicleControl control_cmd;
      control_cmd.throttle = cmd[0];
      control_cmd.steer = cmd[1];
      control_cmd.brake = cmd[2];
      control_cmd.hand_brake = false;
      control_cmd.reverse = false;
      control_cmd_pub.publish(control_cmd);

      ros::spinOnce();
      rate.sleep();

      // ROS_INFO("The iteration end.");
    }
  }

  /**
   * @brief 求解离散LQR矩阵
   *
   * @param vx 纵向速度
   * @return Eigen::Matrix<double, 1, 4>
   */
  Eigen::Matrix<double, 1, 4> ControllerNode::calc_dlqr(double vx,const Eigen::Matrix4d &Q, const Eigen::Matrix<double, 1, 1> &R)
  {
    vx = vx + 0.00001; // 防止速度为0时相除出错
    // A矩阵
    /*
    A matrix (Gear Drive)
    [0.0,                             1.0,                           0.0,                                            0.0;
     0.0,          (cf + cr) / (mass * vx),            -(cf + cr) / mass,              (lf * cf - lr * cr) / (mass * vx);
     0.0,                             0.0,                           0.0,                                            1.0;
     0.0, (lf * cf - lr * cr) / (Iz * vx),     -(lf * cf - lr * cr) / Iz,       (lf * lf * cf + lr * lr * cr) / (Iz * vx);]
    */
    Eigen::Matrix4d A;
    A << 0.0, 1.0, 0.0, 0.0,
        0.0, (cf + cr) / (mass * vx), -(cf + cr) / mass, (lf * cf - lr * cr) / (mass * vx),
        0.0, 0.0, 0.0, 1.0,
        0.0, (lf * cf - lr * cr) / (Iz * vx), -(lf * cf - lr * cr) / Iz, (lf * lf * cf + lr * lr * cr) / (Iz * vx);

    // B矩阵 ：B = [0.0, -cf / mass, 0.0, -lf * cf / Iz]^T

    Eigen::Matrix<double, 4, 1> B;
    B << 0.0, -cf / mass, 0.0, -lf * cf / Iz;

    /***********************************离散化状态方程**************************************/
    double ts = 0.01;
    // 单位矩阵
    Eigen::Matrix4d eye;
    eye.setIdentity(4, 4);
    // 离散化A,B矩阵
    Eigen::Matrix4d Ad;
    Ad = (eye - ts * 0.5 * A).inverse() * (eye + ts * 0.5 * A);
    Eigen::Matrix<double, 4, 1> Bd;
    Bd = B * ts;

    /***********************************求解Riccati方程**************************************/
    int max_iteration = 100; // 设置最大迭代次数
    int tolerance = 0.001;   // 迭代求解精度

    Eigen::Matrix4d P = Q;
    Eigen::Matrix4d P_next;
    for (int i = 0; i < max_iteration; i++)
    {
      P_next = Ad.transpose() * P * Ad -
               Ad.transpose() * P * Bd * (R + Bd.transpose() * P * Bd).inverse() * Bd.transpose() * P * Ad + Q;
      P = P_next;
      if (fabs((P_next - P).maxCoeff()) < tolerance)
      {
        break;
      }
    }
    // 求解k值
    Eigen::Matrix<double, 1, 4> k;
    k = (R + Bd.transpose() * P * Bd).inverse() * Bd.transpose() * P * Ad;
    return k;
  }

  /**
   * @brief 离线求解LQR表
   *
   * @param vx
   */
  void ControllerNode::CreateLqrOffline(const Eigen::Matrix4d &Q, const Eigen::Matrix<double, 1, 1> &R)
  {
    lqr_k_table.clear();
    for (double v = 0; v < 30.0; v += 0.2)
    {
      lqr_k_table.push_back(calc_dlqr(v,Q,R));
    }
  }

  /**
   * @brief carla里程计信息的回调函数, 根据里程计信息获取车辆当前的位置，并发布行驶路径
   *
   * @param msg
   */
  void ControllerNode::callbackCarlaOdom(const nav_msgs::Odometry::ConstPtr &msg)
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
    cur_pose.yaw = yaw;

    cur_pose.vx = msg->twist.twist.linear.x;
    cur_pose.vy = msg->twist.twist.linear.y;
    cur_pose.v = std::sqrt(cur_pose.vx * cur_pose.vx + cur_pose.vy * cur_pose.vy);

    cur_pose.z = msg->pose.pose.position.z;

    // 将Odometry转换为Path，发布车辆行驶路径
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = msg->pose.pose.position.x;
    this_pose_stamped.pose.position.y = msg->pose.pose.position.y;
    this_pose_stamped.pose.position.z = msg->pose.pose.position.z;

    this_pose_stamped.pose.orientation.x = msg->pose.pose.orientation.x;
    this_pose_stamped.pose.orientation.y = msg->pose.pose.orientation.y;
    this_pose_stamped.pose.orientation.z = msg->pose.pose.orientation.z;
    this_pose_stamped.pose.orientation.w = msg->pose.pose.orientation.w;

    this_pose_stamped.header.stamp = ros::Time::now();
    this_pose_stamped.header.frame_id = "map";
    path_msg.poses.push_back(this_pose_stamped);

    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "map";

    path_pub.publish(path_msg);
  }

  /**
   * @brief 获取局部规划路径
   *
   * @param msg
   */
  void ControllerNode::callbackLocalPath(const waypoint_msgs::WaypointArray::ConstPtr &msg)
  {
    // ROS_INFO("Received final waypoints in trajectory controller ...");
    local_waypoints.resize(msg->waypoints.size());
    for (int i = 0; i < msg->waypoints.size(); i++)
    {
      car_state temp_point;
      temp_point.x = msg->waypoints[i].pose.pose.position.x;
      temp_point.y = msg->waypoints[i].pose.pose.position.y;
      temp_point.v = msg->waypoints[i].twist.twist.linear.x;
      local_waypoints[i] = temp_point;
    }
  }

} // carla_pnc