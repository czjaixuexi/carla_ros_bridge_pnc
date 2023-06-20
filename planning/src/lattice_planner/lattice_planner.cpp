#include "lattice_planner/lattice_planner.h"

#define MAX_SPEED 50.0 / 3.6 // maximum speed [m/s]
#define MAX_ACCEL 8.0        // maximum acceleration [m/ss]
#define MAX_CURVATURE 100.0  // maximum curvature [1/m]

using namespace std;

namespace carla_pnc
{

  /**
   * @brief Cartesian转Frenet
   *
   * @param global_point 待转换点的全局轨迹点
   * @param projection_point 投影点
   * @return FrenetPoint
   */
  FrenetPoint Cartesian2Frenet(const car_state &global_point,
                               const path_point &projection_point)
  {
    FrenetPoint frenet_point;
    frenet_point.x = global_point.x;
    frenet_point.y = global_point.y;
    frenet_point.v = global_point.v;
    double delta_theta = global_point.yaw - projection_point.yaw;
    // 计算s
    frenet_point.s = projection_point.s_;
    // 计算l
    int sign = ((global_point.y - projection_point.y) * std::cos(projection_point.yaw) -
                (global_point.x - projection_point.x) * std::sin(projection_point.yaw)) > 0
                   ? 1
                   : -1;

    frenet_point.l = sign * cal_distance(global_point.x, global_point.y,
                                         projection_point.x, projection_point.y);

    // 计算s_d
    frenet_point.s_d = global_point.v * std::cos(delta_theta) /
                       (1 - projection_point.cur * frenet_point.l);
    // ROS_INFO("calculated s_d = :%.2f",frenet_point.s_d);
    // 计算l_d
    frenet_point.l_d = global_point.v * std::sin(delta_theta);

    // 计算l_d_d
    frenet_point.l_d_d = global_point.a * std::sin(delta_theta);
    return frenet_point;
  }

  /**
   * @brief Construct a new Lattice Planner:: Lattice Planner object
   *
   * @param sample_max_time   最大采样时间
   * @param sample_min_time  最小采样时间
   * @param sample_time_step 采样时间step
   * @param sample_lat_width 采样横向距离
   * @param sample_width_length  采样横向距离间隔
   * @param w_object  纵向目标代价
   * @param w_lon_jerk  纵向舒适代价
   * @param w_lat_offset  横向偏离代价
   * @param w_lat_acc  横向舒适代价
   * @param cruise_speed 巡航车速
   * @param collision_detection 碰撞检测模块
   */
  LatticePlanner::LatticePlanner(const double &sample_max_time, const double &sample_min_time, const double &sample_time_step,
                                 const double &sample_lat_width, const double &sample_width_length,
                                 const double &w_object, const double &w_lon_jerk,
                                 const double &w_lat_offset, const double &w_lat_acc,
                                 const double &cruise_speed,
                                 const CollisionDetection &collision_detection)
  {
    this->sample_max_time = sample_max_time;
    this->sample_min_time = sample_min_time;
    this->sample_time_step = sample_time_step;
    this->sample_lat_width = sample_lat_width;
    this->sample_width_length = sample_width_length;

    this->w_object = w_object;
    this->w_lon_jerk = w_lon_jerk;
    this->w_lat_offset = w_lat_offset;
    this->w_lat_acc = w_lat_acc;

    this->cruise_speed = cruise_speed;
    this->collision_detection = collision_detection;
    // ROS_INFO("collision size is%zu",collision_detection.detected_objects.size());
  }

  /**
   * @brief 计算Frenet轨迹规划点纵向相关信息（cruising）
   *
   * @param lon_qp s关于t的四次多项式曲线
   * @param fpoint 规划轨迹点
   */
  void LatticePlanner::calc_lon_values(QuarticPolynomial &lon_qp, FrenetPoint &fpoint)
  {
    fpoint.s = lon_qp.calc_point(fpoint.t);
    fpoint.s_d = lon_qp.calc_first_derivative(fpoint.t);
    fpoint.s_d_d = lon_qp.calc_second_derivative(fpoint.t);
    fpoint.s_d_d_d = lon_qp.calc_third_derivative(fpoint.t);
    fpoint.v = desired_speed;
  }

  /**
   * @brief 计算Frenet轨迹规划点纵向相关信息(following)
   *
   * @param lon_qp s关于t的五次多项式曲线
   * @param fpoint
   */
  void LatticePlanner::calc_lon_values(QuinticPolynomial &lon_qp, FrenetPoint &fpoint)
  {
    fpoint.s = lon_qp.calc_point(fpoint.t);
    fpoint.s_d = lon_qp.calc_first_derivative(fpoint.t);
    fpoint.s_d_d = lon_qp.calc_second_derivative(fpoint.t);
    fpoint.s_d_d_d = lon_qp.calc_third_derivative(fpoint.t);
    fpoint.v = desired_speed;
  }

  /**
   * @brief 计算Frenet轨迹规划点横向相关信息
   *
   * @param lat_qp l关于t的五次多项式曲线
   * @param fpoint 规划轨迹点
   */
  void LatticePlanner::calc_lat_values(QuinticPolynomial &lat_qp, FrenetPoint &fpoint)
  {
    fpoint.l = lat_qp.calc_point(fpoint.t);
    fpoint.l_d = lat_qp.calc_first_derivative(fpoint.t);
    fpoint.l_d_d = lat_qp.calc_second_derivative(fpoint.t);
    fpoint.l_d_d_d = lat_qp.calc_third_derivative(fpoint.t);
  }

  /**
   * @brief 计算纵向目标代价,主要考虑当前速度与参考速度的差值以及走过的距离
   *
   * @param fp 规划轨迹
   * @param target_speed 目标速度
   * @return double
   */
  double LatticePlanner::calc_lon_objective_cost(const FrenetPath &fp, const double target_speed)
  {
    double objective_cost = 0.0;
    double speed_cost = 0.0;
    double time_square_sum = 0.1;
    double dist_cost = 0.0;

    for (FrenetPoint fpoint : fp.frenet_path)
    {
      speed_cost += pow(fpoint.t, 2) * fabs(target_speed - fpoint.s_d);
      time_square_sum += pow(fpoint.t, 2);
    }
    speed_cost = speed_cost / time_square_sum;
    dist_cost = 1.0 / (1.0 + fp.frenet_path.back().s);
    objective_cost = (speed_cost + 10 * dist_cost) / 11;

    return objective_cost;
  }

  /**
   * @brief 计算纵向jerk代价
   *
   * @param fp 规划轨迹
   * @return double
   */
  double LatticePlanner::calc_lon_jerk_cost(const FrenetPath &fp)
  {
    double lon_jerk_cost = 0.0;
    double jerk_square_sum = 0.0;
    double jerk_abs_sum = 0.0;
    for (FrenetPoint fpoint : fp.frenet_path)
    {
      jerk_square_sum += pow(fpoint.s_d_d_d / 4.0, 2);
      jerk_abs_sum += fabs(fpoint.s_d_d_d / 4.0);
    }
    lon_jerk_cost = jerk_square_sum / jerk_abs_sum;
    return lon_jerk_cost;
  }

  /**
   * @brief 计算向心加速度代价（TO-DO）
   *
   * @param fp
   * @return double
   */
  double LatticePlanner::calc_centri_acc_cost(const FrenetPath &fp)
  {
    return 0.0;
  }

  /**
   * @brief 横向位移的代价需要考虑车辆的横向偏移与初始的 L 方向，
   * 如果此刻车辆的横向位置与初始时刻的车辆不在同一侧，计算权重取 5；
   * 如果此刻车辆的横向位置与初始时刻的车辆在同一侧，权重为 1。
   *
   * @param fp
   * @param initial_point
   * @return double
   */
  double LatticePlanner::calc_lat_offset_cost(const FrenetPath &fp, const FrenetPoint &initial_point)
  {
    double lat_offset_cost = 0.0;
    double offset_square_sum = 0.0;
    double offset_abs_sum = 0.0;
    for (FrenetPoint fpoint : fp.frenet_path)
    {
      // 判断横向偏移的方向,若不在同一侧
      if (fpoint.l * initial_point.l < 0.0)
      {
        offset_square_sum += pow(fpoint.l / 3.5, 2) * 5;
        offset_abs_sum += fabs(fpoint.l / 3.5) * 5;
      }
      // 若在同一侧
      else
      {
        offset_square_sum += pow(fpoint.l / 3.5, 2);
        offset_abs_sum += fabs(fpoint.l / 3.5);
      }
    }
    lat_offset_cost = offset_square_sum / offset_abs_sum;
    return lat_offset_cost;
  }

  /**
   * @brief 横向最大加速度cost
   *
   * @param fp
   * @return double
   */
  double LatticePlanner::calc_lat_acc_cost(const FrenetPath &fp)
  {
    double max_acc = 0.0;
    double lat_acc_cost = 0.0;
    for (FrenetPoint fpoint : fp.frenet_path)
    {
      if (fabs(max_acc) < fabs(fpoint.l_d_d))
      {
        max_acc = fpoint.l_d_d;
      }
    }
    lat_acc_cost = fabs(max_acc);
    return lat_acc_cost;
  }

  /**
   * @brief 采样cruising的frenet轨迹
   *
   * @param initial_point 规划起点
   * @return std::vector<FrenetPath>
   */
  std::vector<FrenetPath> LatticePlanner::sampling_cruising_frenet_paths(const FrenetPoint &initial_point)
  {
    std::vector<FrenetPath> fp_list;

    /***********************************纵向轨迹采样**************************************/
    double V = cruise_speed;
    // 根据预测时间进行采样
    for (double Ti = sample_min_time; Ti <= sample_max_time; Ti += sample_time_step)
    {
      // s关于t的四次多项式曲线
      QuarticPolynomial lon_qp(initial_point.s, initial_point.s_d, 0.0, V, 0.0, Ti);

      /***********************************横向轨迹采样**************************************/
      // 对横向位移 l 进行采样
      for (double Li = -1 * sample_lat_width; Li <= sample_lat_width; Li += sample_width_length)
      {
        // l关于t的五次多项式曲线
        QuinticPolynomial lat_qp(initial_point.l, initial_point.l_d, initial_point.l_d_d, Li, 0.0, 0.0, Ti);

        // 横纵向结合的轨迹
        FrenetPath fp;
        fp.max_speed = std::numeric_limits<double>::min();
        fp.max_acc = std::numeric_limits<double>::min();
        // 记录[0 - Ti]离散时间下对应的规划点
        for (double t = 0; t < Ti; t += 0.02)
        {
          FrenetPoint fpoint;
          fpoint.t = t;
          // 计算轨迹点的纵向信息
          calc_lon_values(lon_qp, fpoint);
          // 计算轨迹点的横向信息
          calc_lat_values(lat_qp, fpoint);
          fp.frenet_path.push_back(fpoint);

          // 记录最大速度和最大加速度
          if (fpoint.s_d > fp.max_speed)
          {
            fp.max_speed = fpoint.s_d;
          }
          if (fpoint.s_d_d > fp.max_acc)
          {
            fp.max_acc = fpoint.s_d_d;
          }
        }
        /***********************************计算每条轨迹的cost**************************************/

        // 纵向目标cost
        double lon_objective_cost = calc_lon_objective_cost(fp, cruise_speed);
        // 纵向jerk cost
        double lon_jerk_cost = calc_lon_jerk_cost(fp);

        // TO-DO 先不加看看效果
        // double calc_centri_acc_cost(const FrenetPath &fp);

        double lat_offset_cost = calc_lat_offset_cost(fp, initial_point);

        double lat_acc_cost = calc_lat_acc_cost(fp);
        // ROS_INFO("lon_objective_cost is :%.2f",lon_objective_cost);

        // ROS_INFO("lon_jerk_cost is :%.2f",lon_jerk_cost);

        // ROS_INFO("lat_offset_cost :%.2f",lat_offset_cost);

        // ROS_INFO("lat_acc_cost :%.2f",lat_acc_cost);

        // ROS_INFO("The length of path is :%.2f",fp.frenet_path.back().s);
        fp.cost = w_object * lon_objective_cost + w_lon_jerk * lon_jerk_cost +
                  w_lat_offset * lat_offset_cost + w_lat_acc * lat_acc_cost;
        // 将轨迹添加至候选轨迹中
        fp_list.push_back(fp);
      }
    }
    return fp_list;
  }

  /**
   * @brief 采样跟车路径
   *
   * @param initial_point
   * @return std::vector<FrenetPath>
   */
  std::vector<FrenetPath> LatticePlanner::sampling_following_frenet_paths(const FrenetPoint &initial_point, const FrenetPoint &leader_point)
  {
    std::vector<FrenetPath> fp_list;
    /***********************************纵向轨迹采样**************************************/
    double V = cruise_speed;
    // 根据预测时间进行采样
    for (double Ti = sample_min_time; Ti <= sample_max_time; Ti += sample_time_step)
    {
      // s关于t的五次多项式曲线
      QuinticPolynomial lon_qp(initial_point.s, initial_point.s_d, 0.0, leader_point.s - 8.0, min(V, leader_point.v), 0.0, Ti);

      /***********************************横向轨迹采样**************************************/
      // 对横向位移 l 进行采样
      for (double Li = -0.5 * sample_lat_width; Li <= 0.5 * sample_lat_width; Li += sample_width_length)
      {
        // l关于t的五次多项式曲线
        QuinticPolynomial lat_qp(initial_point.l, initial_point.l_d, initial_point.l_d_d, Li, 0.0, 0.0, Ti);

        // 横纵向结合的轨迹
        FrenetPath fp;
        fp.max_speed = std::numeric_limits<double>::min();
        fp.max_acc = std::numeric_limits<double>::min();
        // 记录[0 - Ti]离散时间下对应的规划点
        for (double t = 0; t < Ti; t += 0.02)
        {
          FrenetPoint fpoint;
          fpoint.t = t;
          // 计算轨迹点的纵向信息
          calc_lon_values(lon_qp, fpoint);
          // 计算轨迹点的横向信息
          calc_lat_values(lat_qp, fpoint);
          fp.frenet_path.push_back(fpoint);

          // 记录最大速度和最大加速度
          if (fpoint.s_d > fp.max_speed)
          {
            fp.max_speed = fpoint.s_d;
          }
          if (fpoint.s_d_d > fp.max_acc)
          {
            fp.max_acc = fpoint.s_d_d;
          }
        }
        /***********************************计算每条轨迹的cost**************************************/

        // 纵向目标cost
        double lon_objective_cost = calc_lon_objective_cost(fp, cruise_speed);
        // 纵向jerk cost
        double lon_jerk_cost = calc_lon_jerk_cost(fp);

        // TO-DO 先不加看看效果
        // double calc_centri_acc_cost(const FrenetPath &fp);

        double lat_offset_cost = calc_lat_offset_cost(fp, initial_point);

        double lat_acc_cost = calc_lat_acc_cost(fp);
        // ROS_INFO("lon_objective_cost is :%.2f",lon_objective_cost);

        // ROS_INFO("lon_jerk_cost is :%.2f",lon_jerk_cost);

        // ROS_INFO("lat_offset_cost :%.2f",lat_offset_cost);

        // ROS_INFO("lat_acc_cost :%.2f",lat_acc_cost);

        // ROS_INFO("The length of path is :%.2f",fp.frenet_path.back().s);
        fp.cost = w_object * lon_objective_cost + w_lon_jerk * lon_jerk_cost +
                  w_lat_offset * lat_offset_cost + w_lat_acc * lat_acc_cost;
        // 将轨迹添加至候选轨迹中
        fp_list.push_back(fp);
      }
    }
    return fp_list;
  }

  /**
   * @brief 根据据参考轨迹与frenet采样轨迹数组，计算frenet轨迹在全局坐标系下参数
   *
   * @param fp_list frenet采样轨迹数组
   * @param ref_frenet 参考轨迹
   */
  void LatticePlanner::get_cartesian_paths(std::vector<FrenetPath> &fp_list, Spline2D &ref_frenet)
  {
    for (FrenetPath &fp : fp_list)
    {
      fp.size_ = 0;
      for (unsigned int i = 0; i < fp.frenet_path.size(); i++)
      {
        // 若轨迹比参考轨迹长，则及时截断
        if (fp.frenet_path[i].s >= ref_frenet.s.back())
        {
          break;
        }
        fp.size_++; // 有效点个数

        /*******************************求cartesian 坐标系中 x,y******************************************/
        // 投影点信息
        std::array<double, 2> poi = ref_frenet.calc_postion(fp.frenet_path[i].s);
        double yawi = ref_frenet.calc_yaw(fp.frenet_path[i].s);
        double li = fp.frenet_path[i].l;

        // ROS_INFO("poi____x:%.2f,  y:%.2f", poi[0],poi[1]);

        double x = poi[0] + li * cos(yawi + M_PI / 2.0);
        double y = poi[1] + li * sin(yawi + M_PI / 2.0);
        // ROS_INFO("frenet_path____x:%.2f,  y:%.2f", x,y);
        fp.frenet_path[i].x = x;
        fp.frenet_path[i].y = y;
      }

      /*********************************求cartesian 坐标系中 yaw****************************************/

      double dx = 0.0;
      double dy = 0.0;
      for (unsigned int i = 0; i < fp.size_ - 1; i++)
      {
        dx = fp.frenet_path[i + 1].x - fp.frenet_path[i].x;
        dy = fp.frenet_path[i + 1].y - fp.frenet_path[i].y;

        fp.frenet_path[i].yaw = atan2(dy, dx);
        fp.frenet_path[i].ds = sqrt(dx * dx + dy * dy);
      }
      // 补全缺失的航向角
      fp.frenet_path.back().yaw = atan2(dy, dx);
      fp.frenet_path.back().ds = sqrt(dx * dx + dy * dy);

      /***********************************求cartesian 坐标系中 曲率 和 速度**************************************/
      double v = 0.0;
      fp.max_curvature = std::numeric_limits<double>::min();
      for (unsigned int i = 0; i < fp.size_ - 1; i++)
      {
        // 求曲率
        double dyaw = fp.frenet_path[i + 1].yaw - fp.frenet_path[i].yaw;
        fp.frenet_path[i].cur = (dyaw) / fp.frenet_path[i].ds;

        // 求速度
        double k_cur = ref_frenet.calc_curvature(fp.frenet_path[i].s); // 投影点的曲率

        // v = sqrt(pow(fp.frenet_path[i].s_d, 2) * pow(1 - k_cur * fp.frenet_path[i].l_d, 2) +
        //          pow(fp.frenet_path[i].l_d, 2));
        // v = fp.frenet_path[i].ds / 0.1;
        // v = desired_speed;
        fp.frenet_path[i].v = desired_speed;
        // ROS_INFO("The target_v is %.2f",v);
        if (fp.frenet_path[i].cur > fp.max_curvature)
        {
          fp.max_curvature = fp.frenet_path[i].cur;
        }
      }
      fp.frenet_path.back().cur = 0;
      fp.frenet_path.back().v = v;
    }
  }

  // /**
  //  * @brief 碰撞检测模块
  //  *
  //  * @param path 规划轨迹
  //  * @param obstacles_list 障碍物全局坐标点集
  //  * @return true
  //  * @return false
  //  */
  // bool LatticePlanner::check_collision(FrenetPath &path,
  //                                      const std::vector<std::vector<car_state>> &obstacles_list)
  // {
  //   // 每个障碍物由8个点集组成一个BOX
  //   for (vector<car_state> obstacle : obstacles_list)
  //   {
  //     // 遍历障碍物的每个点
  //     for (car_state ob_point : obstacle)
  //     {
  //       // 遍历路径上的每个点
  //       for (unsigned int i = 0; i < path.size_; i++)
  //       {
  //         // vector<car_state> self_collision_box = cal_collision_box(path.frenet_path[i], 2.0, 1.08);
  //         // for (car_state point : self_collision_box)
  //         // {
  //         //   double dist = cal_distance(point.x, point.y, ob_point.x, ob_point.y);
  //         //   // path.cost += (dist/100.0);
  //         //   if (dist <= ROBOT_RADIUS)
  //         //   {
  //         //     return false;
  //         //   }
  //         // }
  //         double dist = cal_distance(path.frenet_path[i].x, path.frenet_path[i].y, ob_point.x, ob_point.y);
  //         if (dist < 3)
  //         {
  //           path.cost += dist / 10;
  //         };
  //         if (dist <= ROBOT_RADIUS)
  //         {
  //           return false;
  //         }
  //       }
  //     }
  //   }
  //   return true;
  // };

  // /**
  //  * @brief 轨迹合理性校验，并根据cost排序
  //  *
  //  * @param fp_list 轨迹列表
  //  * @param ob 障碍物全局坐标点集(x,y)
  //  * @return std::vector<FrenetPath>
  //  */
  // std::vector<FrenetPath> LatticePlanner::get_valid_paths(std::vector<FrenetPath> &fp_list, const Vec_Poi &ob)
  // {

  //   vector<FrenetPath> valid_paths;
  //   for (FrenetPath fp : path_list)
  //   {
  //     // 最大速度约束， 最大加速度约束，最大曲率约束，避免碰撞
  //     if (fp.max_speed < MAX_SPEED && fp.max_acc < MAX_ACCEL &&
  //         fp.max_curvature < MAX_CURVATURE && check_collision(path, ob))
  //     {
  //       valid_paths.push_back(fp);
  //     }
  //   }
  //   return valid_paths;
  // }

  /**
   * @brief 轨迹合理性校验，并根据cost排序
   *
   * @param fp_list fp_list 轨迹列表
   * @param leader_point 跟车目标
   * @param car_following 跟车状态
   * @return std::priority_queue<FrenetPath, std::vector<FrenetPath>, cmp>
   */
  std::priority_queue<FrenetPath, std::vector<FrenetPath>, cmp>
  LatticePlanner::get_valid_paths(std::vector<FrenetPath> &fp_list,
                                  const FrenetPoint &leader_point,
                                  const bool &car_following)
  {
    std::priority_queue<FrenetPath, std::vector<FrenetPath>, cmp> valid_paths;

    for (FrenetPath &fp : fp_list)
    {
      // ROS_INFO("The path cost without collision is%.2f",fp.cost);
      // 最大速度约束， 最大加速度约束，最大曲率约束，避免碰撞
      if (fp.max_speed < MAX_SPEED &&
          fp.max_acc < MAX_ACCEL &&
          fp.max_curvature < MAX_CURVATURE &&
          collision_detection.check_collision(fp, leader_point, car_following))
      {
        valid_paths.push(fp);
        // ROS_INFO("The path cost with collision is%.2f",fp.cost);
      }
    }
    return valid_paths;
  }

  /**
   * @brief 轨迹规划
   *
   * @param ref_frenet 参考Frenet曲线
   * @param initial_point Frenet坐标系下规划起点
   * @param leader_point 跟车目标
   * @param car_following 跟车状态
   * @return FrenetPath
   */
  FrenetPath LatticePlanner::planning(Spline2D &ref_frenet,
                                      const FrenetPoint &initial_point,
                                      const FrenetPoint &leader_point,
                                      const bool &car_following)
  {
    // 采样
    vector<FrenetPath> frenet_path_list;
    if (car_following)
    {
      ROS_INFO("Following,leader_point speed is %.2f", leader_point.v);
      desired_speed = min(cruise_speed, leader_point.v);
      frenet_path_list = sampling_following_frenet_paths(initial_point, leader_point);
    }
    else
    {
      ROS_INFO("Cruising");
      desired_speed = cruise_speed;
      frenet_path_list = sampling_cruising_frenet_paths(initial_point);
    }
    // ROS_INFO("Sampling Successfully");

    // frenet转cartesian
    get_cartesian_paths(frenet_path_list, ref_frenet);
    // ROS_INFO("Change Successfully");

    // 验证路径并排序
    auto valid_paths = get_valid_paths(frenet_path_list, leader_point, car_following);
    // ROS_INFO("Sort Successfully,the size is%zu", valid_paths.size());

    if (!valid_paths.empty())
    {
      best_path = valid_paths.top();
      pre_best_path = best_path;
      return best_path;
    }
    else
    {
      return pre_best_path;
    }
  }

  /**
   * @brief 获取所有有效的规划轨迹(用于可视化)
   *
   * @param ref_frenet 参考Frenet曲线
   * @param initial_point Frenet坐标系下规划起点
   * @param leader_point 跟车目标
   * @param car_following 跟车状态
   * @return std::vector<FrenetPath>
   */
  std::vector<FrenetPath> LatticePlanner::get_planning_paths(Spline2D &ref_frenet,
                                                             const FrenetPoint &initial_point,
                                                             const FrenetPoint &leader_point,
                                                             const bool &car_following)
  {
    // 采样
    vector<FrenetPath> frenet_path_list;
    if (car_following)
    {
      frenet_path_list = sampling_following_frenet_paths(initial_point, leader_point);
    }
    else
    {
      frenet_path_list = sampling_cruising_frenet_paths(initial_point);
    }

    get_cartesian_paths(frenet_path_list, ref_frenet);

    // 验证路径并排序
    auto valid_paths = get_valid_paths(frenet_path_list, leader_point, car_following);

    vector<FrenetPath> planning_paths;

    while (!valid_paths.empty())
    {
      // ROS_INFO("Cost is :%.2f",valid_paths.top().cost);
      planning_paths.push_back(valid_paths.top());
      valid_paths.pop();
    }
    // vector<FrenetPath> planning_paths(valid_paths.begin(), valid_paths.end());
    return planning_paths;
  };

} // carla_pnc
