/**
 * @file point_types.h
 * @author czj
 * @brief
 * @version 0.1
 * @date 2023-06-13
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef POINT_TYPES_H
#define POINT_TYPES_H

#include <array>
#include <iostream>
#include <string>
#include <vector>

namespace carla_pnc
{
  // 全局坐标系路径点
  class path_point
  {
  public:
    double x;
    double y;
    double z;
    double yaw; // 横摆角
    double cur; // 曲率
    double s_;  // 弧长
  };

  // 车辆位置信息(全局坐标系轨迹点)
  class car_state : public path_point
  {
  public:
    double vx; // x方向速度值
    double vy; // y方向速度值
    double v;  // 合速度

    double ax;
    double ay;
    double a; // 加速度

    double t; // 时间（相对）
  };

  // 在全局坐标点的基础上增加Frenet信息
  class FrenetPoint : public car_state
  {
  public:
    double s;
    double l;
    double s_d;
    double l_d;
    double s_d_d;
    double l_d_d;

    // 计算jerk代价用
    double s_d_d_d;
    double l_d_d_d;

    double ds; // 计算曲率用
  };

  class FrenetPath
  {
  public:
    double cost;
    std::vector<FrenetPoint> frenet_path;
    int size_ = 0; // 用于记录有效点的个数

    double max_speed;
    double max_acc;
    double max_curvature;
  };

} // carla_pnc

#endif // POINT_TYPES_H
