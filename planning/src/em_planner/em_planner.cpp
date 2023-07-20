#include "em_planner/em_planner.h"

using namespace std;

namespace carla_pnc
{
    /**
     * @brief Construct a new EMPlanner object
     *
     * @param collision_detection_
     * @param dp_path_params
     * @param qp_path_params
     */
    EMPlanner::EMPlanner(const CollisionDetection &collision_detection,
                         std::unordered_map<std::string, double> &dp_path_params,
                         std::unordered_map<std::string, double> &qp_path_params)
    {
        this->collision_detection = collision_detection;

        // DP path params
        this->dp_sample_l = dp_path_params["dp_sample_l"];                         // dp采样横向距离间隔
        this->dp_sample_s = dp_path_params["dp_sample_s"];                         // dp采样纵向距离间隔
        this->dp_sample_rows = static_cast<int>(dp_path_params["dp_sample_rows"]); // dp采样行数（横向）
        this->dp_sample_cols = static_cast<int>(dp_path_params["dp_sample_cols"]); // dp采样列数（纵向）

        this->dp_cost_collision = dp_path_params["dp_cost_collision"]; // dp碰撞代价
        this->dp_cost_dl = dp_path_params["dp_cost_dl"];
        this->dp_cost_ddl = dp_path_params["dp_cost_ddl"];
        this->dp_cost_dddl = dp_path_params["dp_cost_dddl"];
        this->dp_cost_ref = dp_path_params["dp_cost_ref"];

        // QP path params
        this->qp_cost_l = qp_path_params["qp_cost_l"];
        this->qp_cost_dl = qp_path_params["qp_cost_dl"];
        this->qp_cost_ddl = qp_path_params["qp_cost_ddl"];
        this->qp_cost_dddl = qp_path_params["qp_cost_dddl"];
        this->qp_cost_ref = qp_path_params["qp_cost_ref"];
        this->qp_cost_end_l = qp_path_params["qp_cost_end_l"];
        this->qp_cost_end_dl = qp_path_params["qp_cost_end_dl"];
        this->qp_cost_end_ddl = qp_path_params["qp_cost_end_ddl"];
    }

    /**
     * @brief DP路径采样
     * /*           0   1   2   3
                0   x   x   x   x
                1   x   x   x   x
           -----2-  x---x---x---x----------------reference_line
                3   x   x   x   x
                4   x   x   x   x

        */
    void EMPlanner::dp_sampling(const FrenetPoint &initial_point)
    {
        int rows = this->dp_sample_rows;
        int cols = this->dp_sample_cols;
        dp_sample_path.resize(rows);
        for (int i = 0; i < rows; ++i)
        {
            dp_sample_path[i].resize(cols);
            for (int j = 0; j < cols; ++j)
            {
                dp_sample_path[i][j].s = initial_point.s + (j + 1) * this->dp_sample_s;
                dp_sample_path[i][j].l = initial_point.l + ((rows + 1) / 2 - (i + 1)) * this->dp_sample_l;
                dp_sample_path[i][j].l_ds = 0;
                dp_sample_path[i][j].l_d_ds = 0;
            }
        }
    }

    /**
     * @brief 计算两点之间五次多项式连接的代价
     *
     * @param start
     * @param end
     * @return double
     */
    double EMPlanner::calc_dppath_cost(const FrenetPoint &start, const FrenetPoint &end)
    {
        // 五次多项式连接两点
        QuinticPolynomial quintic_path(start.l, start.l_ds, start.l_d_ds,
                                       end.l, end.l_ds, end.l_d_ds,
                                       start.s, end.s);

        /***********************************中间插值10个点，计算代价(车辆向前运动，s单调递增)**************************************/
        FrenetPoint insert_point;
        double cost_smooth = 0;    // 平滑代价
        double cost_ref = 0;       // 偏离参考线代价
        double cost_collision = 0; // 碰撞代价
        int insert_size = 10;

        for (int i = 0; i < insert_size; ++i)
        {
            insert_point.s = start.s + (i + 1) * (end.s - start.s) / insert_size;
            insert_point.l = quintic_path.calc_point(insert_point.s);
            insert_point.l_ds = quintic_path.calc_first_derivative(insert_point.s);
            insert_point.l_d_ds = quintic_path.calc_second_derivative(insert_point.s);
            insert_point.l_d_d_ds = quintic_path.calc_third_derivative(insert_point.s);

            // 平滑代价
            cost_smooth += this->dp_cost_dl * pow(insert_point.l_ds, 2) +
                           this->dp_cost_ddl * pow(insert_point.l_d_ds, 2) +
                           this->dp_cost_dddl * pow(insert_point.l_d_d_ds, 2);
            // 如果ddl或者切向角过大，则增加cost
            if (abs(insert_point.l_d_ds) > 0.5 ||
                abs(atan(insert_point.l_d_d_ds)) > 0.4 * M_PI)
            {
                cost_smooth += 10e7;
            }

            // 偏离参考线代价
            cost_ref += this->dp_cost_ref * pow(insert_point.l, 2);

            // 碰撞代价

            for (auto &static_ob : collision_detection.static_obstacle_list)
            {
                double dist = cal_distance(insert_point.s, insert_point.l,
                                           static_ob.point.s, static_ob.point.l);

                // ROS_INFO("obstacle,s: %.2f,l:%.2f", static_ob.point.s, static_ob.point.l);
                // ROS_INFO("The num of static obstacle is  %zu", collision_detection.static_obstacle_list.size());
                // ROS_INFO("path,s: %.2f,l:%.2f", insert_point.s, insert_point.l);

                if (dist < 6.0 && dist > 4.0)
                {
                    cost_collision += 1000;
                }
                else if (dist < 4.0)
                {
                    cost_collision += this->dp_cost_collision;
                }

                // if (dist < 5.0)
                // {
                //     // 外积判断障碍物左右方向
                //     std::pair<double, double> path_or = make_pair(end.s - start.s, end.l - start.l);                             // 路径方向向量
                //     std::pair<double, double> obstacle_or = make_pair(static_ob.point.s - start.s, static_ob.point.s - start.l); // 障碍物方向向量

                //     // ROS_INFO("obstacle,s: %.2f,l:%.2f", static_ob.point.s, static_ob.point.l);
                //     // ROS_INFO("car,s: %.2f,l:%.2f", start.s, start.l);
                //     // 在左侧，减小碰撞代价（更倾向于从左侧过）
                //     if (path_or.first * obstacle_or.second - path_or.second * obstacle_or.first < 0 && cost_collision > 0)
                //     {
                //         // ROS_INFO("left");
                //         cost_collision -= 1000;
                //     }
                // }
            }

            // // 外积判断障碍物左右方向
            // std::pair<double, double> path_or = make_pair(end.s - start.s, end.l - start.l);                       // 路径方向向量
            // std::pair<double, double> obstacle_or = make_pair(min_ob.point.s - start.s, min_ob.point.l - start.l); // 障碍物方向向量

            // // 在左侧，减小碰撞代价（更倾向于从左侧过）
            // if (path_or.first * obstacle_or.second - path_or.second * obstacle_or.first < 0 && cost_collision > 0)
            // {
            //     cost_collision -= 100;
            // }
        }
        return cost_smooth + cost_ref + cost_collision;
    }

    /**
     * @brief 路径动态规划
     *
     */
    void EMPlanner::calc_dp_path(const FrenetPoint &initial_point)
    {
        int rows = this->dp_sample_rows;
        int cols = this->dp_sample_cols;

        /***********************************动态规划计算cost**************************************/
        // 计算起点到第一列的cost
        for (int i = 0; i < rows; ++i)
        {
            dp_sample_path[i][0].dp_cost = calc_dppath_cost(initial_point, dp_sample_path[i][0]);
        }

        // 计算第一列之后的cost
        for (int j = 1; j < cols; ++j)
        {
            for (int i = 0; i < rows; ++i)
            {
                // std::vector<std::future<double>> cost_result(rows);
                dp_sample_path[i][j].dp_cost = DBL_MAX;
                // // async异步计算cost
                // for (int g = 0; g < rows; ++g)
                // {
                //     cost_result[k] = std::async(std::launch::async, &EMPlanner::calc_dppath_cost, this,
                //                                 this->dp_sample_path[g][j - 1], this->dp_sample_path[i][j]);
                // }

                for (int k = 0; k < rows; ++k)
                {
                    // 计算前一列每一行到当前点的cost
                    double cost = dp_sample_path[k][j - 1].dp_cost +
                                  calc_dppath_cost(dp_sample_path[k][j - 1], dp_sample_path[i][j]);

                    // double cost = dp_sample_path[k][j - 1].dp_cost + cost_result[k].get();

                    if (cost < dp_sample_path[i][j].dp_cost)
                    {
                        dp_sample_path[i][j].dp_cost = cost;
                        dp_sample_path[i][j].dp_pre_row = k; // 记录最优路径上一列的行号
                    }
                }
            }
        }

        /***********************************回溯获得dp最优路径**************************************/
        dp_path.resize(cols + 1); // 采样列数+起点
        dp_path[0] = initial_point;
        // 找到最后一列的最小cost点
        dp_path[cols] = dp_sample_path[0][cols - 1];
        for (int i = 1; i < rows; ++i)
        {
            if (dp_sample_path[i][cols - 1].dp_cost <
                dp_path[cols].dp_cost)
            {
                dp_path[cols] = dp_sample_path[i][cols - 1];
            }
        }
        // 回溯
        for (int j = cols - 1; j > 0; --j)
        {
            int pre_row = dp_path[j + 1].dp_pre_row;
            dp_path[j] = dp_sample_path[pre_row][j - 1];
        }
    }

    /**
     * @brief DP path 插值
     *
     * @param dp_path
     * @return std::vector<FrenetPoint>
     */
    void EMPlanner::dp_path_interpolation(const std::vector<FrenetPoint> &dp_path)
    {
        dp_final_path.clear();
        for (int i = 0; i < dp_path.size() - 1; ++i)
        {
            // 五次多项式连接两点
            QuinticPolynomial quintic_path(dp_path[i].l, dp_path[i].l_ds, dp_path[i].l_d_ds,
                                           dp_path[i + 1].l, dp_path[i + 1].l_ds, dp_path[i + 1].l_d_ds,
                                           dp_path[i].s, dp_path[i + 1].s);
            for (double s = dp_path[i].s; s <= dp_path[i + 1].s; s += path_ds)
            {
                FrenetPoint insert_point;
                insert_point.s = s;
                insert_point.l = quintic_path.calc_point(insert_point.s);
                insert_point.l_ds = quintic_path.calc_first_derivative(insert_point.s);
                insert_point.l_d_ds = quintic_path.calc_second_derivative(insert_point.s);
                insert_point.l_d_d_ds = quintic_path.calc_third_derivative(insert_point.s);
                dp_final_path.push_back(insert_point);
            }
        }
    }

    /**
     * @brief 生成QP path的 l上下限
     *
     * @param dp_final_path
     */
    void EMPlanner::calc_convex_space(const std::vector<FrenetPoint> &dp_final_path)
    {
        // 边界初始化
        int size = dp_final_path.size();
        l_min = Eigen::VectorXd::Ones(size) * -10;
        l_max = Eigen::VectorXd::Ones(size) * 10;

        // 遍历静态障碍物
        for (auto &static_ob : collision_detection.static_obstacle_list)
        {
            // 初始化障碍物边界
            double ob_s_min = static_ob.point.s; // 下边界
            double ob_s_max = static_ob.point.s; // 上边界

            double ob_l_min = static_ob.point.l; // 右边界
            double ob_l_max = static_ob.point.l; // 左边界

            for (auto &box_point : static_ob.collision_box)
            {
                ob_s_min = min(box_point.s, ob_s_min);
                ob_s_max = max(box_point.s, ob_s_max);

                ob_l_min = min(box_point.l, ob_l_min);
                ob_l_max = max(box_point.l, ob_l_max);
            }
            // ROS_INFO("static ob s:%.2f, %.2f", static_ob.point.s, static_ob.point.l);
            // ROS_INFO("ob_s_min:%.2f,ob_s_max:%.2f,ob_l_min:%.2f,ob_l_max:%.2f", ob_s_min, ob_s_max, ob_l_min, ob_l_max);

            // int start_index = get_obstacle_index(dp_final_path, ob_s_min);
            // int end_index = get_obstacle_index(dp_final_path, ob_s_max);

            int start_index = get_obstacle_index(dp_final_path, ob_s_min);
            int end_index = get_obstacle_index(dp_final_path, ob_s_max);
            int mid_index = get_obstacle_index(dp_final_path, static_ob.point.s);

            // 若障碍物不在边界内，跳过
            // if ((start_index == 0 && end_index == 0) ||
            //     (start_index == (size - 1) && end_index == (size - 1)))
            if (start_index == 0 || end_index == (size - 1))
            {
                continue;
            }
            double path_l = dp_final_path[mid_index].l;

            // ROS_INFO("start_index:%d,end_index:%d", start_index, end_index);
            for (int i = start_index; i <= end_index; ++i)
            {
                // 从左边过
                if (path_l >= static_ob.point.l)
                {
                    // l_min(i) = max(l_min(i), static_ob.point.l);
                    l_min(i) = max(l_min(i), ob_l_max);
                    // ROS_INFO("index:%d, l_min:%.2f", i, l_min(i));
                }
                // 从右边过
                else
                {
                    // l_max(i) = min(l_max(i), static_ob.point.l);
                    l_max(i) = min(l_max(i), ob_l_min);
                    // ROS_INFO("index:%d, l_max:%.2f", i, l_max(i));
                }
            }
        }
    }

    /**
     * @brief 获取障碍物在dp path中的匹配下标
     *
     * @param dp_final_path
     * @param ob_s
     * @return int
     */
    int EMPlanner::get_obstacle_index(const std::vector<FrenetPoint> &dp_final_path,
                                      const double &ob_s)
    {

        // 障碍物不在边界内
        if (dp_final_path.front().s > ob_s)
        {
            return 0;
        }
        else if (dp_final_path.back().s < ob_s)
        {
            return dp_final_path.size() - 1;
        }

        int index = 0;
        for (int i = 1; i < dp_final_path.size() - 1; ++i)
        {
            if (dp_final_path[i].s > ob_s)
            {
                index = i;
                break;
            }
        }
        if (fabs(dp_final_path[index].s - ob_s) > fabs(dp_final_path[index - 1].s - ob_s))
        {
            return index - 1;
        }
        else
        {
            return index;
        }
    }

    /**
     * @brief
            0.5*x'Hx + f'*x = min
            subject to A*x <= b
                    Aeq*x = beq
                    lb <= x <= ub;
            其中
                x = [l1,l1_dl,l1_ddl,l2,l2_dl,l2_ddl ....ln,ln_dl,ln_ddl]^T 3n个参数

            QP求解条件
                qp_cost_l = 2;
                qp_cost_dl = 25000;
                qp_cost_ddl = 15;
                qp_cost_dddl = 15;
                qp_cost_ref = 15;
                qp_cost_end_l = 15;
                qp_cost_end_dl = 15;
                qp_cost_end_ddl = 15;
                dp_final_path dp结果
                l_min,l_max 凸边界
            QP求解结果
                qp_path_l
                qp_path_l_ds
                qp_path_l_d_ds
     */
    void EMPlanner::calc_qp_path(const std::vector<FrenetPoint> &dp_final_path,
                                 const Eigen::VectorXd &l_min,
                                 const Eigen::VectorXd &l_max)
    {
        int n = dp_final_path.size();
        double ds = path_ds;

        // double lf = 2.0;    // 质心到车辆前端距离
        // double lr = 2.0;    // 质心到车辆后端距离
        double width = 3.6; // 车宽

        // Hissen矩阵 H
        Eigen::SparseMatrix<double> H(3 * n, 3 * n);
        Eigen::SparseMatrix<double> H_L(3 * n, 3 * n);
        Eigen::SparseMatrix<double> H_DL(3 * n, 3 * n);
        Eigen::SparseMatrix<double> H_DDL(3 * n, 3 * n);
        Eigen::SparseMatrix<double> H_DDDL(n - 1, 3 * n);

        Eigen::SparseMatrix<double> H_CENTRE(3 * n, 3 * n);
        Eigen::SparseMatrix<double> H_L_END(3 * n, 3 * n);
        Eigen::SparseMatrix<double> H_DL_END(3 * n, 3 * n);
        Eigen::SparseMatrix<double> H_DDL_END(3 * n, 3 * n);

        // grident矩阵 f
        Eigen::VectorXd f = Eigen::VectorXd::Zero(3 * n);

        // // 不等式约束  A*x <= b，边界约束
        // Eigen::SparseMatrix<double> A(4 * n, 3 * n);
        // Eigen::VectorXd b = Eigen::VectorXd::Zero(4 * n);

        // 等式约束Aeq*x = beq,连续性约束
        Eigen::SparseMatrix<double> Aeq(2 * n - 2, 3 * n);
        Eigen::VectorXd beq = Eigen::VectorXd::Zero(2 * n - 2);

        // 边界约束 lb <= x <= ub ,dl ddl变化量约束
        Eigen::SparseMatrix<double> A_lu(3 * n, 3 * n);
        A_lu.setIdentity();
        Eigen::VectorXd lb = Eigen::VectorXd::Ones(3 * n) * (-DBL_MAX);
        Eigen::VectorXd ub = Eigen::VectorXd::Ones(3 * n) * DBL_MAX;

        // 约束整合矩阵 lb_merge <= A_merge*x <= ub_merge
        Eigen::SparseMatrix<double> A_merge(2 * n - 2 + 3 * n, 3 * n);
        Eigen::VectorXd lb_merge(2 * n - 2 + 3 * n);
        Eigen::VectorXd ub_merge(2 * n - 2 + 3 * n);

        /***********************************求H**************************************/
        for (int i = 0; i < n; i++)
        {
            H_L.insert(3 * i, 3 * i) = 1;
            H_DL.insert(3 * i + 1, 3 * i + 1) = 1;
            H_DDL.insert(3 * i + 2, 3 * i + 2) = 1;
        }
        H_CENTRE = H_L;

        for (int i = 0; i < n - 1; i++)
        {
            int row = i;
            int col = 3 * i;

            H_DDDL.insert(row, col + 2) = 1;
            H_DDDL.insert(row, col + 5) = -1;
        }

        H_L_END.insert(3 * n - 3, 3 * n - 3) = 0;
        H_DL_END.insert(3 * n - 2, 3 * n - 2) = 0;
        H_DDL_END.insert(3 * n - 1, 3 * n - 1) = 0;

        H = this->qp_cost_l * (H_L.transpose() * H_L) +
            this->qp_cost_dl * (H_DL.transpose() * H_DL) +
            this->qp_cost_ddl * (H_DDL.transpose() * H_DDL) +
            this->qp_cost_dddl * (H_DDDL.transpose() * H_DDDL) / pow(ds, 2) +
            this->qp_cost_ref * (H_CENTRE.transpose() * H_CENTRE) +
            this->qp_cost_end_l * (H_L_END.transpose() * H_L_END) +
            this->qp_cost_end_dl * (H_DL_END.transpose() * H_DL_END) +
            this->qp_cost_end_ddl * (H_DDL_END.transpose() * H_DDL_END);
        H = 2 * H;

        /***********************************求f**************************************/

        // auto centre_line = 0.5 * (l_min + l_max);
        for (int i = 0; i < n; i++)
        {
            f(3 * i) = (-2) * this->qp_cost_ref * dp_final_path[i].l;
            // f(3 * i) = (-2) * this->qp_cost_ref * 0.5 * (l_min(i) + l_max(i));
        }

        // 期望的终点状态
        // double end_l_desire = 0;
        // double end_dl_desire = 0;
        // double end_ddl_desire = 0;
        // f(3 * n - 3) = f(3 * n - 3) - 2 * end_l_desire * this->qp_cost_end_l;
        // f(3 * n - 2) = f(3 * n - 2) - 2 * end_dl_desire * this->qp_cost_end_dl;
        // f(3 * n - 1) = f(3 * n - 1) - 2 * end_ddl_desire * this->qp_cost_end_ddl;

        /*********************************** 边界约束 Ax<=b 8n个变量**************************************/
        // for (int i = 0; i < n - 1; i++)
        // {
        //     int row = 8 * i;
        //     int col = 3 * i;

        //     A_merge.insert(row + 0, col + 0) = 1;
        //     A_merge.insert(row + 1, col + 0) = 1;
        //     A_merge.insert(row + 2, col + 0) = 1;
        //     A_merge.insert(row + 3, col + 0) = 1;

        //     A_merge.insert(row + 4, col + 0) = -1;
        //     A_merge.insert(row + 5, col + 0) = -1;
        //     A_merge.insert(row + 6, col + 0) = -1;
        //     A_merge.insert(row + 7, col + 0) = -1;

        //     A_merge.insert(row + 0, col + 1) = lf;
        //     A_merge.insert(row + 1, col + 1) = lf;
        //     A_merge.insert(row + 2, col + 1) = -lr;
        //     A_merge.insert(row + 3, col + 1) = -lr;

        //     A_merge.insert(row + 4, col + 1) = -lf;
        //     A_merge.insert(row + 5, col + 1) = -lf;
        //     A_merge.insert(row + 6, col + 1) = lr;
        //     A_merge.insert(row + 7, col + 1) = lr;
        // }

        // // 生成b
        // int front_index = ceil(lf / ds);
        // int back_index = ceil(lr / ds);
        // for (int i = 0; i < n; i++)
        // {
        //     int index1 = min(i + front_index, n - 1); // 车头索引
        //     int index2 = max(i - back_index, 0);      // 车尾索引
        //     Eigen::VectorXd b_sub(8);
        //     b_sub << l_max(index1) - width / 2, l_max(index1) + width / 2, l_max(index2) - width / 2, l_max(index2) + width / 2,
        //         -l_min(index1) + width / 2, -l_min(index1) - width / 2, -l_min(index2) + width / 2, -l_min(index2) - width / 2;

        //     // b_sub << l_max(i) - width / 2, l_max(i) + width / 2, l_max(i) - width / 2, l_max(i) + width / 2,
        //     //     -l_min(i) + width / 2, -l_min(i) - width / 2, -l_min(i) + width / 2, -l_min(i) - width / 2;

        //     // cout << "index: " << i << " b_sub: " << b_sub.transpose() << endl;
        //     b.block(8 * i, 0, 8, 1) = b_sub;
        // }

        // cout << b << endl;
        /***********************************连续性约束 Aeq*x = beq 2n-2个变量**************************************/
        int row_index_start = 0;
        for (int i = 0; i < n - 1; i++)
        {
            int row = row_index_start + 2 * i;
            int col = 3 * i;
            A_merge.insert(row, col) = 1;
            A_merge.insert(row, col + 1) = ds;
            A_merge.insert(row, col + 2) = pow(ds, 2) / 3;
            A_merge.insert(row, col + 3) = -1;
            A_merge.insert(row, col + 5) = pow(ds, 2) / 6;

            A_merge.insert(row + 1, col + 1) = 1;
            A_merge.insert(row + 1, col + 2) = ds / 2;
            A_merge.insert(row + 1, col + 4) = -1;
            A_merge.insert(row + 1, col + 5) = ds / 2;
        }

        /***********************************起点约束，一阶，二阶导约束 lb<= A*x <=ub 3n个变量**************************************/
        row_index_start = 2 * n - 2;
        for (int i = 0; i < n; i++)
        {
            int row = row_index_start + 3 * i;
            int col = 3 * i;
            A_merge.insert(row, col) = 1;
            A_merge.insert(row + 1, col + 1) = 1;
            A_merge.insert(row + 2, col + 2) = 1;
            // A_merge.insert(row_index_start + i + 2, i + 2) = 1;
        }

        lb(0) = dp_final_path[0].l;
        lb(1) = dp_final_path[0].l_ds;
        lb(2) = dp_final_path[0].l_d_ds;
        ub(0) = lb(0);
        ub(1) = lb(1);
        ub(2) = lb(2);

        // 边界约束，l_ds, l_d_ds约束
        for (int i = 1; i < n; i++)
        {
            // lb(3 * i + 1) = -DBL_MAX;
            // lb(3 * i + 2) = -DBL_MAX;
            // ub(3 * i + 1) = DBL_MAX;
            // ub(3 * i + 2) = DBL_MAX;
            lb(3 * i) = l_min(i) + width / 2;
            lb(3 * i + 1) = -2.0;
            lb(3 * i + 2) = -0.1;
            ub(3 * i) = l_max(i) - width / 2;
            ub(3 * i + 1) = 2.0;
            ub(3 * i + 2) = 0.1;
        }

        /***********************************约束汇总**************************************/

        // // 边界约束 Ax<=b 8n个变量
        // lb_merge.block(0, 0, 8 * n, 1) = Eigen::MatrixXd::Ones(8 * n, 1) * (-DBL_MAX);
        // ub_merge.block(0, 0, 8 * n, 1) = b;

        // 连续性约束 Aeq*x = beq 2n-2个变量
        // lb_merge.block(8 * n, 0, 2 * n - 2, 1) = beq;
        // ub_merge.block(8 * n, 0, 2 * n - 2, 1) = beq;
        lb_merge.block(0, 0, 2 * n - 2, 1) = beq;
        ub_merge.block(0, 0, 2 * n - 2, 1) = beq;

        // 边界约束，起点约束，一阶，二阶导约束 lb<= A*x <=ub 3n个变量
        // lb_merge.block(8 * n + 2 * n - 2, 0, 3 * n, 1) = lb;
        // ub_merge.block(8 * n + 2 * n - 2, 0, 3 * n, 1) = ub;
        lb_merge.block(2 * n - 2, 0, 3 * n, 1) = lb;
        ub_merge.block(2 * n - 2, 0, 3 * n, 1) = ub;

        /***********************************调用osqp-eign**************************************/
        OsqpEigen::Solver solver;

        solver.settings()->setWarmStart(true);
        solver.settings()->setVerbosity(false);                   // 不打印osqp日志
        solver.data()->setNumberOfVariables(3 * n);               // A矩阵列数
        solver.data()->setNumberOfConstraints(2 * n - 2 + 3 * n); // A矩阵行数
        solver.data()->setHessianMatrix(H);
        solver.data()->setGradient(f);
        solver.data()->setLinearConstraintsMatrix(A_merge);
        solver.data()->setLowerBound(lb_merge);
        solver.data()->setUpperBound(ub_merge);

        if (!solver.initSolver())
        {
            ROS_INFO("QSOP init failed");
            return;
        }
        if (!solver.solve())
        {
            ROS_INFO("QSOP solve failed");
            return;
        }

        Eigen::VectorXd qp_solution(3 * n);
        qp_solution = solver.getSolution();

        qp_path = dp_final_path;
        for (int i = 0; i < n; ++i)
        {
            qp_path[i].l = qp_solution(3 * i);
            qp_path[i].l_ds = qp_solution(3 * i + 1);
            qp_path[i].l_d_ds = qp_solution(3 * i + 2);
        }
        ROS_INFO("get QP path successfully");
    }

    /**
     * @brief 计算frenet轨迹在全局坐标系下参数
     *
     * @param frenet_path frenet轨迹
     * @param ref_frenet 参考轨迹
     * @return int 轨迹点的个数
     */
    int EMPlanner::get_cartesian_paths(std::vector<FrenetPoint> &frenet_path, Spline2D &ref_frenet)
    {

        int size_ = 0;
        for (unsigned int i = 0; i < frenet_path.size(); ++i)
        {
            // 若轨迹比参考轨迹长，则及时截断
            if (frenet_path[i].s >= ref_frenet.s.back())
            {
                break;
            }
            size_++; // 有效点个数
            /*******************************求cartesian 坐标系中 x,y******************************************/
            // 投影点信息
            std::array<double, 2> poi = ref_frenet.calc_postion(frenet_path[i].s);
            double yawi = ref_frenet.calc_yaw(frenet_path[i].s);
            double curi = ref_frenet.calc_curvature(frenet_path[i].s);
            double li = frenet_path[i].l;

            frenet_path[i].x = poi[0] + li * cos(yawi + M_PI / 2.0);
            frenet_path[i].y = poi[1] + li * sin(yawi + M_PI / 2.0);
            /*********************************求cartesian 坐标系中 yaw****************************************/
            frenet_path[i].yaw = atan2(frenet_path[i].l_ds, (1 - curi * li)) + yawi;
            frenet_path[i].v = desired_speed;
        }

        // /*********************************求cartesian 坐标系中 yaw****************************************/
        // double dx = 0.0;
        // double dy = 0.0;
        // for (unsigned int i = 0; i < size_ - 1; i++)
        // {
        //     dx = frenet_path[i + 1].x - frenet_path[i].x;
        //     dy = frenet_path[i + 1].y - frenet_path[i].y;
        //     frenet_path[i].yaw = atan2(dy, dx);
        //     frenet_path[i].ds = sqrt(dx * dx + dy * dy);
        // }
        // // 补全缺失的航向角
        // frenet_path.back().yaw = atan2(dy, dx);
        // frenet_path.back().ds = sqrt(dx * dx + dy * dy);

        // /***********************************求cartesian 坐标系中曲率和速度**************************************/
        // double cur = 0.0;
        // for (unsigned int i = 0; i < size_ - 1; i++)
        // {
        //     // 求曲率
        //     double dyaw = frenet_path[i + 1].yaw - frenet_path[i].yaw;
        //     cur = (dyaw) / frenet_path[i].ds;
        //     frenet_path[i].cur = cur;
        //     frenet_path[i].v = desired_speed;
        // }
        // frenet_path.back().cur = cur;
        // frenet_path.back().v = desired_speed;
        return size_;
    }

    FrenetPath EMPlanner::planning(Spline2D &ref_frenet, const FrenetPoint &initial_point)
    {

        FrenetPath final_path;
        /***********************************DP path**************************************/
        // dp路径采样
        dp_sampling(initial_point);
        // ROS_INFO("sampling successuflly,the rows:%zu, cols:%zu",dp_sample_path.size(),dp_sample_path[0].size());

        // dp path动态规划
        calc_dp_path(initial_point);
        // ROS_INFO("get dp_path successuflly,the size is%zu",dp_final_path.size());
        // dp path插值
        dp_path_interpolation(dp_path);

        /***********************************QP path**************************************/
        // 生成凸空间
        calc_convex_space(this->dp_final_path);

        // QP 求解
        calc_qp_path(this->dp_final_path, this->l_min, this->l_max);

        // 转回全局坐标系
        // final_path.size_ = get_cartesian_paths(dp_final_path, ref_frenet);

        // final_path.frenet_path = dp_final_path;

        final_path.size_ = get_cartesian_paths(qp_path, ref_frenet);

        final_path.frenet_path = qp_path;

        return final_path;
    }

} // carla_pnc
