#include "common.h"
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
    double cal_distance(double x1, double y1, double x2, double y2)
    {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return std::sqrt(dx * dx + dy * dy);
    }

    /**
     * @brief 寻找最近匹配点下标
     *
     * @param cur_x
     * @param cur_y
     * @param path
     * @param pre_match_index
     * @return
     */
    int search_match_index(const double &cur_x, const double &cur_y,
                           const std::vector<path_point> &path,
                           const int &pre_match_index)
    {
        double dist;
        double min_dist = DBL_MAX;
        int match_index = 0;
        for (int i = pre_match_index; i < path.size(); i++)
        {
            dist = cal_distance(path[i].x, path[i].y, cur_x, cur_y);
            if (dist < min_dist)
            {
                min_dist = dist;
                match_index = i;
            }
        }
        return match_index;
    }

    /**
     * @brief 通过匹配点求投影点
     *
     * @param match_point 匹配点
     * @param projection_point 投影点
     * @return path_point
     * https://zhuanlan.zhihu.com/p/429676544
     */
    path_point match_to_projection(const car_state &cur_pose,
                                   const path_point &match_point)
    {
        // 投影点其他值都与匹配点相同,求其x,y,yaw
        path_point projection_point = match_point;

        // 匹配点切向量tor
        Eigen::Matrix<double, 2, 1> tor;
        tor << cos(match_point.yaw), sin(match_point.yaw);

        // 匹配点至自车向量d
        Eigen::Matrix<double, 2, 1> d;
        d << cur_pose.x - match_point.x, cur_pose.y - match_point.y;

        // d在tor方向上的投影分量
        double e_s = tor.transpose() * d;

        // 求投影点x,y
        projection_point.x = match_point.x + e_s * cos(match_point.yaw);
        projection_point.y = match_point.y + e_s * sin(match_point.yaw);

        // 求投影点的yaw
        projection_point.yaw = match_point.yaw + match_point.cur * e_s;
        return projection_point;
    }

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
        // 保留原有信息（向下转型不安全，此处手动赋值）
        frenet_point.x = global_point.x;
        frenet_point.y = global_point.y;
        frenet_point.z = global_point.z;
        frenet_point.yaw = global_point.yaw;
        frenet_point.cur = global_point.cur;

        frenet_point.vx = global_point.vx;
        frenet_point.vy = global_point.vy;
        frenet_point.v = global_point.v;
        frenet_point.ax = global_point.ax;
        frenet_point.ay = global_point.ay;
        frenet_point.a = global_point.a;
        frenet_point.t = global_point.t;

        double delta_theta = global_point.yaw - projection_point.yaw;
        // 计算s
        frenet_point.s = projection_point.s_;

        // 计算l = (x_or- ->x_r_or) * n_or
        Eigen::Matrix<double, 2, 1> x_or;
        x_or << global_point.x, global_point.y;

        Eigen::Matrix<double, 2, 1> x_r_or;
        x_r_or << projection_point.x, projection_point.y;

        Eigen::Matrix<double, 2, 1> n_or;
        n_or << -sin(projection_point.yaw), cos(projection_point.yaw);

        frenet_point.l = (x_or - x_r_or).transpose() * n_or;
        // int sign = ((global_point.y - projection_point.y) * std::cos(projection_point.yaw) -
        //             (global_point.x - projection_point.x) * std::sin(projection_point.yaw)) > 0
        //                ? 1
        //                : -1;

        // frenet_point.l = sign * cal_distance(global_point.x, global_point.y,
        //                                      projection_point.x, projection_point.y);

        // 计算s_d
        frenet_point.s_d = global_point.v * std::cos(delta_theta) /
                           (1 - projection_point.cur * frenet_point.l);
        // ROS_INFO("calculated s_d = :%.2f",frenet_point.s_d);
        // 计算l_d = v*sin(delta_theta)
        frenet_point.l_d = global_point.v * std::sin(delta_theta);

        // 计算l_ds = l_d / s_d
        if (fabs(frenet_point.s_d) < 1e-6)
        {
            frenet_point.l_ds = 0;
        }
        else
        {
            frenet_point.l_ds = frenet_point.l_d / frenet_point.s_d;
        }

        // 计算s_d_d, 此处忽略dkr/ds，简化为0
        Eigen::Matrix<double, 2, 1> t_or;
        t_or << cos(projection_point.yaw), sin(projection_point.yaw);

        Eigen::Matrix<double, 2, 1> a_or;
        t_or << global_point.ax, global_point.ay;

        frenet_point.s_d_d = (a_or.transpose() * t_or +
                              2 * projection_point.cur * frenet_point.l_ds * pow(frenet_point.s_d, 2)) /
                             (1 - projection_point.cur * frenet_point.l);

        // 计算l_d_d = a*sin(delta_theta)
        frenet_point.l_d_d = global_point.a * std::sin(delta_theta);

        // 计算l_d_ds = (l_d_d - l_ds*s_d_d) / s_d^2
        if (fabs(frenet_point.s_d < 1e-6))
        {
            frenet_point.l_d_ds = 0;
        }
        else
        {
            frenet_point.l_d_ds = (frenet_point.l_d_d - frenet_point.l_ds * frenet_point.s_d_d) /
                                  pow(frenet_point.s_d, 2);
        }
        return frenet_point;
    }

    FrenetPoint calc_frenet(const car_state &global_point,
                            const std::vector<path_point> &ref_path)
    {
        // 计算匹配点下标
        int frenet_match_index = search_match_index(global_point.x, global_point.y, ref_path, 0);

        // 通过匹配点求投影点
        path_point projection_point = match_to_projection(global_point, ref_path[frenet_match_index]);

        // 计算Frenet坐标
        FrenetPoint frenet_point = Cartesian2Frenet(global_point, projection_point);
        return frenet_point;
    }
} // carla_pnc