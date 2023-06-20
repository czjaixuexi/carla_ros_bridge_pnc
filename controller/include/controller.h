/**
 * @file controller.h
 * @author czj
 * @brief
 * @version 0.1
 * @date 2023-06-06
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <cfloat>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <iomanip>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
#include "ros/ros.h"
#include <Eigen/Eigen>
// #include "reference_line.h"

namespace carla_pnc
{

  // 车辆位置信息
  struct car_state
  {
    double x;
    double y;
    double z;
    double yaw; // 横摆角
    double vx;  // x方向速度值
    double vy;  // y方向速度值
    double v;   // 合速度
    double cur; // 曲率
  };

  /***********************************辅助函数**************************************/
  double cal_distance(const double &x1, const double &y1,
                      const double &x2, const double &y2);

  double normalize_angle(double &angle);

  double degree_to_rad(const double &degree);

  double rad_to_steer(const double &steer_in_rad, const double &max_degree);

  class Controller
  {
  public:
    /***********************************整车参数**************************************/
    double L;          // 轴距
    double cf;         // 前轮侧偏刚度,左右轮之和
    double cr;         // 后轮侧偏刚度, 左右轮之和
    double mass_fl;    // 左前悬的质量
    double mass_fr;    // 右前悬的质量
    double mass_rl;    // 左后悬的质量
    double mass_rr;    // 右后悬的质量
    double mass_front; // 前悬质量
    double mass_rear;  // 后悬质量
    double mass;       // 车辆载荷
    double lf;         // 前轴中心到质心的距离
    double lr;         // 后轴中心到质心的距离
    double Iz;         // 车辆绕z轴转动的转动惯量
    double max_degree; // 最大前轮转向角(度)

    Controller(const std::string &lat_control_method,
               const double &k_pure, const double &k_cte,
               const Eigen::Matrix4d &Q, const Eigen::Matrix<double, 1, 1> &R,
               const double &kp, const double &ki, const double &kd);

    void update_car_state(const car_state &cur_pose,
                          const double &timestamp);

    void update_desired_speed();

    void update_waypoints(const std::vector<car_state> &local_waypoints);

    void cal_heading(std::vector<car_state> &waypoints);


    /***********************************控制命令相关**************************************/
    
    std::vector<double> get_commands() { return commands; }

    void set_throttle(const double &input_throttle);

    void set_steer(const double &input_steer_in_rad);

    void set_brake(const double &input_brake);

    void update_controls(const double &frequency_update);

    void reset_all_previous();

  protected:
    std::string lat_control_method;                   // 横向控制方法
    std::unordered_map<std::string, double> previous; // 用于临时存储一些上一循环周期的变量
    car_state cur_pose;                               // 车辆当前状态
    double cur_time;                                  // 当前时间戳
    bool first_loop;                                  // 判断是否为第一次循环
    std::vector<car_state> waypoints;                 // 局部路径信息 x,y, x方向速度
    std::vector<double> commands;                     // throttle, steer, brake

    /***********************************横向控制参数**************************************/
    double k_pure;           // purepursuit前视距离系数
    double k_cte;            // Stanley 增益系数
    int closest_index;       // 最近匹配点下标
    double closest_distance; // 与最近匹配点的距离

    Eigen::Matrix4d Q;             // Q矩阵
    Eigen::Matrix<double, 1, 1> R; // R矩阵

    /***********************************纵向PID参数**************************************/
    double sum_pid_error; // pid累计误差
    double kp;
    double ki;
    double kd;
    double desired_speed; // 期望速度

    int search_closest_index(const car_state &cur_pose,
                             const std::vector<car_state> &waypoints);

    int search_lookahead_index(const car_state &cur_pose,
                               const std::vector<car_state> &waypoints,
                               const double &lookahead_distance);

    int get_steering_direction(const int &target_index);

    double cal_heading_error(const std::vector<car_state> &waypoints,
                             const double &cur_yaw,
                             const double &target_index);

    double cal_cte_heading_error(const double &cur_speed,
                                 const double &e_y);

    double cal_steering();

    double cal_longitudinal(const double &dt, const double &v, const double &desired_v);

    Eigen::Matrix<double, 4, 1> cal_lqr_error(const car_state &cur_pose,
                                              const double &target_index,
                                              const std::vector<car_state> &waypoints);

    Eigen::Matrix<double, 1, 4> cal_dlqr(double vx);

    double cal_forward_angle(const Eigen::Matrix<double, 1, 4> &k,
                             const double &cur,
                             const double &vx);
  };
} // namespace carla_pnc

#endif // CONTROLLER_H
