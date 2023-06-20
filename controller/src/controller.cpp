/**
 * @file controller.cpp
 * @author czj
 * @brief
 * @version 0.1
 * @date 2023-06-06
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "controller.h"

using namespace std;

namespace carla_pnc
{
  /***********************************辅助函数**************************************/
  /**
   * @brief 计算两点间的距离
   *
   * @param x1
   * @param y1
   * @param x2
   * @param y2
   * @return double
   */
  double cal_distance(const double &x1, const double &y1,
                      const double &x2, const double &y2)
  {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return sqrt(dx * dx + dy * dy);
  }

  /**
   * @brief 角度归一化
   * @param angle
   * @return
   */
  double normalize_angle(double &angle)
  {
    while (angle > M_PI)
    {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI)
    {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  /**
   * @brief 角度转换为弧度
   *
   * @param degree
   * @return double
   */
  double degree_to_rad(const double &degree)
  {
    return degree * M_PI / 180;
  }

  /**
   * @brief 弧度转换为方向盘控制指令
   *
   * @param steer_in_rad
   * @param max_degree
   * @return double
   */
  double rad_to_steer(const double &steer_in_rad, const double &max_degree)
  {
    return steer_in_rad * 180.0 / max_degree / M_PI;
  }

  /**
   * @brief Construct a new Controller:: Controller object
   *
   * @param lat_control_method 控制方法
   * @param k_pure pure pursuit增益系数
   * @param k_cte stanley增益系数
   * @param Q Q矩阵
   * @param R R矩阵
   * @param kp
   * @param ki
   * @param kd
   */
  Controller::Controller(const std::string &lat_control_method,
                         const double &k_pure, const double &k_cte,
                         const Eigen::Matrix4d &Q, const Eigen::Matrix<double, 1, 1> &R,
                         const double &kp, const double &ki, const double &kd)
  {
    this->lat_control_method = lat_control_method;
    this->first_loop = true;
    this->cur_time = 0.0;
    this->commands = {0.0, 0.0, 0.0}; // throttle, steer, brake

    /***********************************整车参数**************************************/

    this->L = 2.875;                                                                     // 轴距
    this->cf = -155494.663;                                                              // 前轮侧偏刚度,左右轮之和
    this->cr = -155494.663;                                                              // 后轮侧偏刚度, 左右轮之和
    this->mass_fl = 1880.0 / 4;                                                          // 左前悬的质量
    this->mass_fr = 1880.0 / 4;                                                          // 右前悬的质量
    this->mass_rl = 1880.0 / 4;                                                          // 左后悬的质量
    this->mass_rr = 1880.0 / 4;                                                          // 右后悬的质量
    this->mass_front = this->mass_fl + this->mass_fr;                                    // 前悬质量
    this->mass_rear = this->mass_rl + this->mass_rr;                                     // 后悬质量
    this->mass = this->mass_front + this->mass_rear;                                     // 车辆载荷
    this->lf = this->L * (1.0 - this->mass_front / this->mass);                          // 汽车前轮到中心点的距离
    this->lr = this->L * (1.0 - this->mass_rear / this->mass);                           // 汽车后轮到中心点的距离
    this->Iz = pow(this->lf, 2) * this->mass_front + pow(this->lr, 2) * this->mass_rear; // 车辆绕z轴转动的转动惯量
    this->max_degree = 30.0;                                                             // 最大前轮转向角(度)

    /***********************************横向控制参数**************************************/

    this->k_pure = k_pure; // purepursuit前视距离系数
    this->k_cte = k_cte;   // Stanley 增益系数
    this->closest_index = 0;
    this->closest_distance = DBL_MAX;

    // Q矩阵
    this->Q = Q;
    // R矩阵
    this->R = R;

    /***********************************纵向PID控制参数**************************************/

    this->sum_pid_error = 0.0; // PID累计误差
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->desired_speed = 0.0;
  }

  /**
   * @brief 更新车辆当前状态
   *
   * @param cur_pose
   * @param timestamp
   */
  void Controller::update_car_state(const car_state &cur_pose,
                                    const double &timestamp)
  {
    this->cur_pose = cur_pose;
    this->cur_time = timestamp;
  }

  /**
   * @brief 更新期望速度值
   *
   */
  void Controller::update_desired_speed()
  {
    if (this->closest_index < this->waypoints.size() - 1)
    {
      this->desired_speed = this->waypoints[this->closest_index].v;
    }
    else
    {
      this->desired_speed = this->waypoints[this->waypoints.size() - 1].v;
    }
  }

  /**
   * @brief 更新规划轨迹点
   *
   * @param local_waypoints  局部规划轨迹点
   */
  void Controller::update_waypoints(const std::vector<car_state> &local_waypoints)
  {
    this->waypoints = local_waypoints;
    cal_heading(this->waypoints);
    this->closest_index = search_closest_index(this->cur_pose, this->waypoints);
  }

  /***********************************横向控制相关**************************************/

  /**
   * @brief 计算参考路径的横摆角,曲率
   *
   */
  void Controller::cal_heading(vector<car_state> &waypoints)
  {
    double x_delta = 0.0;
    double y_delta = 0.0;
    double x_delta_2 = 0.0;
    double y_delta_2 = 0.0;
    for (int i = 0; i < this->waypoints.size(); i++)
    {
      if (i == 0)
      {
        x_delta = (waypoints[i + 1].x - waypoints[i].x);
        y_delta = (waypoints[i + 1].y - waypoints[i].y);
        x_delta_2 = (waypoints[i + 2].x - waypoints[i + 1].x) - (waypoints[i + 1].x - waypoints[i].x);
        y_delta_2 = (waypoints[i + 2].y - waypoints[i + 1].y) - (waypoints[i + 1].y - waypoints[i].y);
      }
      else if (i == waypoints.size() - 1)
      {
        x_delta = (waypoints[i].x - waypoints[i - 1].x);
        y_delta = (waypoints[i].y - waypoints[i - 1].y);
        x_delta_2 = (waypoints[i].x - waypoints[i - 1].x) - (waypoints[i - 1].x - waypoints[i - 2].x);
        y_delta_2 = (waypoints[i].y - waypoints[i - 1].y) - (waypoints[i - 1].y - waypoints[i - 2].y);
      }
      else
      {
        x_delta = 0.5 * (waypoints[i + 1].x - waypoints[i - 1].x);
        y_delta = 0.5 * (waypoints[i + 1].y - waypoints[i - 1].y);
        x_delta_2 = (waypoints[i + 1].x - waypoints[i].x) - (waypoints[i].x - waypoints[i - 1].x);
        y_delta_2 = (waypoints[i + 1].y - waypoints[i].y) - (waypoints[i].y - waypoints[i - 1].y);
      }
      waypoints[i].yaw = std::atan2(y_delta, x_delta);
      //  参数方程曲率计算
      waypoints[i].cur = std::abs(y_delta_2 * x_delta - x_delta_2 * y_delta) / std::pow((x_delta * x_delta + y_delta * y_delta), 3 / 2);
    }
  }

  /**
   * @brief 寻找匹配点下标
   *
   * @param cur_pose
   * @param waypoints
   * @return int
   */
  int Controller::search_closest_index(const car_state &cur_pose,
                                       const std::vector<car_state> &waypoints)
  {
    double dist;
    for (int i = 0; i < waypoints.size(); i++)
    {
      dist = cal_distance(waypoints[i].x, waypoints[i].y, cur_pose.x, cur_pose.y);
      if (dist < this->closest_distance)
      {
        this->closest_distance = dist;
        this->closest_index = i;
      }
    }
    return this->closest_index;
  }

  /**
   * @brief 寻找预瞄点下标(用于Pure pursuit)
   *
   * @param cur_pose
   * @param waypoints
   * @param lookahead_distance
   * @return int
   */
  int Controller::search_lookahead_index(const car_state &cur_pose,
                                         const std::vector<car_state> &waypoints,
                                         const double &lookahead_distance)
  {
    double dist;
    for (int i = this->closest_index; i < waypoints.size(); i++)
    {
      dist = cal_distance(cur_pose.x, cur_pose.y,
                          waypoints[i].x, waypoints[i].y);

      if (dist > lookahead_distance)
      {
        return i;
      }
    }
    return waypoints.size() - 1;
  }

  /**
   * @brief 判断横向误差方向
   *
   * @param target_index
   * @return int
   */
  int Controller::get_steering_direction(const int &target_index)
  {
    // 将匹配点投影到车身坐标系中
    double y_in_vehicle_frame = -(waypoints[target_index].x - cur_pose.x) * sin(cur_pose.yaw) +
                                (waypoints[target_index].y - cur_pose.y) * cos(cur_pose.yaw);

    if (y_in_vehicle_frame > 0)
    {
      return 1;
    }
    else
      return -1;
  }

  /**
   * @brief 计算航向误差(用于Stanley)
   *
   * @param waypoints
   * @param cur_yaw
   * @param target_index
   * @return double
   */
  double Controller::cal_heading_error(const std::vector<car_state> &waypoints,
                                       const double &cur_yaw,
                                       const double &target_index)
  {
    double heading_error = waypoints[target_index].yaw - cur_yaw;
    return heading_error;
  }

  /**
   * @brief 计算横向跟踪误差的转角(用于Stanley)
   *
   * @param cur_speed
   * @param e_y 横向误差
   * @return double
   */
  double Controller::cal_cte_heading_error(const double &cur_speed,
                                           const double &e_y)
  {
    double cte_heading_error = atan2(k_cte * e_y, cur_speed);
    return cte_heading_error;
  }

  /**
   * @brief 计算LQR横向误差
   *
   * @param cur_pose 车辆当前状态
   * @param target_index 匹配点
   * @param waypoints 路径
   * @return array<double, 4>
   */
  Eigen::Matrix<double, 4, 1> Controller::cal_lqr_error(const car_state &cur_pose,
                                                        const double &target_index,
                                                        const std::vector<car_state> &waypoints)
  {
    Eigen::Matrix<double, 4, 1> err;

    // 匹配点切向量tor
    Eigen::Matrix<double, 2, 1> tor;
    tor << cos(waypoints[target_index].yaw), sin(waypoints[target_index].yaw);

    // 匹配点法向量nor
    Eigen::Matrix<double, 2, 1> nor;
    nor << -sin(waypoints[target_index].yaw), cos(waypoints[target_index].yaw);

    // 计算e_d
    Eigen::Matrix<double, 2, 1> d_err;
    d_err << cur_pose.x - waypoints[target_index].x, cur_pose.y - waypoints[target_index].y;
    double e_d = nor.transpose() * d_err;

    // 计算投影点的角度
    double e_s = tor.transpose() * d_err;
    double projection_theta = waypoints[target_index].yaw +
                              waypoints[target_index].cur * e_s;

    // 计算e_d_dot
    double e_d_dot = cur_pose.vy * cos(cur_pose.yaw - projection_theta) +
                     cur_pose.vx * sin(cur_pose.yaw - projection_theta);

    // 计算e_phi,消除角度的多值性(phi+2π与phi相同)
    double e_phi = sin(cur_pose.yaw - projection_theta);

    // 计算e_phi_dot
    double s_dot = (cur_pose.vx * cos(cur_pose.yaw - projection_theta) -
                    cur_pose.vy * sin(cur_pose.yaw - projection_theta)) /
                   (1 - waypoints[target_index].cur * e_d);
    double phi_dot = cur_pose.vx * waypoints[target_index].cur;
    double e_phi_dot = phi_dot - s_dot * waypoints[target_index].cur;

    err << e_d,
        e_d_dot,
        e_phi,
        e_phi_dot;
    return err;
  }

  /**
   * @brief 求解离散LQR矩阵
   *
   * @param vx x方向速度
   * @return Eigen::Matrix<double, 1, 4>
   */
  Eigen::Matrix<double, 1, 4> Controller::cal_dlqr(double vx)
  {
    vx = vx + 0.0001; // 防止速度为0时相除出错
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
    double ts = 0.001;
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
   * @brief 计算前馈控制转角
   *
   * @param k 求解dlar得到的k
   * @param cur 匹配点的曲率
   * @param vx x方向速度
   * @return double
   */
  double Controller::cal_forward_angle(const Eigen::Matrix<double, 1, 4> &k,
                                       const double &cur,
                                       const double &vx)
  {
    double k3 = k[2];
    // 不足转向系数
    double kv = lr * mass / (cf * L) - lf * mass / (cr * L);

    double forward_angle = L * cur + kv * vx * vx * cur -
                           k3 * (lr * cur - lf * mass * vx * vx * cur / (lr * cr));

    // double forward_angle = cur * (lf + lr - lr * k3 -
    //                               (mass * pow(vx, 2) / (lf + lr)) * (lr / cf + lf * k3 / cr - lf / cr));
    return forward_angle;
  }

  /**
   * @brief 计算方向转角
   *
   * @return double
   */
  double Controller::cal_steering()
  {
    double steering = 0.0;

    if (lat_control_method == "PurePursuit")
    {
      // pure pursuit的预瞄距离
      double lookahead_dist = k_pure * this->cur_pose.v;

      int target_index = search_lookahead_index(this->cur_pose, this->waypoints, lookahead_dist);

      double alpha = std::atan2(waypoints[target_index].y - this->cur_pose.y, this->waypoints[target_index].x - this->cur_pose.y) - this->cur_pose.yaw;

      if (isnan(alpha))
      {
        alpha = previous["alpha"];
      }
      else
      {
        previous["alpha"] = alpha;
      }

      steering = atan((2 * this->L * sin(alpha)) / lookahead_dist);
    }
    else if (lat_control_method == "Stanley")
    {
      int target_index = search_closest_index(this->cur_pose, this->waypoints);
      // 横摆角误差
      double heading_error = cal_heading_error(this->waypoints, this->cur_pose.yaw, target_index);
      // 横向误差
      double cte_heading_error = get_steering_direction(target_index) * cal_cte_heading_error(this->cur_pose.v, this->closest_distance);

      // ROS_INFO("cte_error: %2f, heading_error: %2f", cte_heading_error, cte_heading_error);

      steering = heading_error + 2.0 * cte_heading_error;
    }
    else if (lat_control_method == "LQR")
    {
      int target_index = search_closest_index(this->cur_pose, this->waypoints);

      // 计算横向误差
      Eigen::Matrix<double, 4, 1> err = cal_lqr_error(this->cur_pose, target_index, this->waypoints);

      // 求解LQR k值
      Eigen::Matrix<double, 1, 4> k = cal_dlqr(this->cur_pose.vx);

      // 计算前馈转角
      double forward_angle = cal_forward_angle(k, this->waypoints[target_index].cur, this->cur_pose.vx);

      steering = forward_angle - k * err;
      // printf("当前位置x:%.3f,y:%x,yaw:%.3f\n",cur_pose.x,cur_pose.y,cur_pose.yaw);
      // printf("目标位置x:%.3f,y:%y,yaw:%.3f\n",this->waypoints[target_index].x,this->waypoints[target_index].y,
      // this->waypoints[target_index].yaw);

      // printf("反馈控制:%.f,前馈控制：%.3f\n", -k * err, forward_angle);
    }

    else
    {
      steering = 0;
    }
    if (isnan(steering))
    {
      steering = previous["steering"];
    }
    else
    {
      previous["steering"] = steering;
    }
    return steering;
  }

  void Controller::set_throttle(const double &input_throttle)
  {
    double throttle = max(min(input_throttle, 1.0), 0.0);
    commands[0] = throttle;
  }

  void Controller::set_steer(const double &input_steer_in_rad)
  {
    double steer = rad_to_steer(input_steer_in_rad, this->max_degree);
    commands[1] = steer;
  }

  void Controller::set_brake(const double &input_brake)
  {
    double brake = max(min(input_brake, 1.0), 0.0);
    commands[2] = brake;
  }

  /**
   * @brief 初始化所有临时值
   *
   */
  void Controller::reset_all_previous()
  {
    previous["v"] = 0.0;
    previous["t"] = 0.0;
    previous["throttle"] = 0.0;
    previous["v_error"] = 0.0;
    previous["alpha"] = 0.0;
    previous["steering"] = 0.0;
  }

  /**
   * @brief 纵向PID控制
   *
   * @param dt
   * @param v
   * @param desired_v
   * @return double
   */
  double Controller::cal_longitudinal(const double &dt, const double &v, const double &desired_v)
  {
    double v_error = desired_v - v;
    sum_pid_error += v_error;
    double k_ = this->kp * v_error;
    double integral_ = this->ki * sum_pid_error * dt;
    double diff_ = this->kd * (v_error - previous["v_error"]) / dt;

    double throttle = k_ + integral_ + diff_;

    previous["throttle"] = throttle;
    previous["v_error"] = v_error;
    return throttle;
  }

  /**
   * @brief 更新控制指令
   *
   * @param frequency_update
   */
  void Controller::update_controls(const double &frequency_update)
  {
    update_desired_speed();

    // ROS_INFO("v_desired: %2f, v: %2f", v_desired, v);
    double throttle_output = 0;
    double steer_output = 0;
    double brake_output = 0;

    if (first_loop)
    {
      reset_all_previous();
    }
    else
    {
      if (waypoints.size() > 1)
      {
        /***********************************纵向控制**************************************/
        double dt = 1 / frequency_update;

        throttle_output = cal_longitudinal(dt, this->cur_pose.v, this->desired_speed);

        brake_output = 0;

        /***********************************横向控制**************************************/

        steer_output = cal_steering();
        steer_output = normalize_angle(steer_output);
        // 限制前轮最大转角，这里定义前轮最大转角位于 [-30度～30度]
        if (steer_output >= degree_to_rad(30.0))
        {
          steer_output = degree_to_rad(30.0);
        }
        else if (steer_output <= -degree_to_rad(30.0))
        {
          steer_output = -degree_to_rad(30.0);
        }
      }
      else
      {
        throttle_output = previous["throttle"];
        steer_output = previous["steering"];
        brake_output = 0;
      }

      if (this->desired_speed < 0.01)
      {
        throttle_output = 0;
        steer_output = 0;
        brake_output = 1;
      }

      // 设置控制指令
      set_throttle(throttle_output); //  (0 to 1)
      set_steer(-steer_output);      // (-1 to 1)
      set_brake(brake_output);       // (0 to 1)

      // 存储上一个周期的值
      previous["v"] = this->cur_pose.v;
      previous["t"] = this->cur_time;
    }
    
    first_loop = false;
  }
} // carla_pnc
