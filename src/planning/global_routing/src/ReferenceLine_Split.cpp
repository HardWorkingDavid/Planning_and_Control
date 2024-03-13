#include "../include/ReferenceLine_Split.h"

ReferenceLine_Split::ReferenceLine_Split() {
    ros::NodeHandle n;
    // 发布参考线，给局部规划用
    referenceline_pub_ = n.advertise<nav_msgs::Path>("/bigdavid/referenceline_centerPoint", 10);
}

void ReferenceLine_Split::referenceLine_split(Eigen::MatrixXd &hdmap_way_points) {
    average_interpolation(hdmap_way_points, path_point_after_interpolation, 0.2, 0.6); // 不能太密，也不能太稀疏
    Publish(path_point_after_interpolation);
}

// 线性插值
void ReferenceLine_Split::average_interpolation(Eigen::MatrixXd &input, Eigen::MatrixXd &output, double interval_dis,
                                                double distance)
{
  // 1.定义一个容器，类型为Point3D_s,即（x,y,z）
  std::vector<Point3D_s> vec_3d;
  std::vector<Point3D_s> n_vec;
  Point3D_s p;
  // 2.遍历
  for (int i = 0; i < input.rows() - 1; i++)
  {
    double dis = (input.row(i + 1) - input.row(i)).norm(); // 求两点的距离，前一行和这一行坐标的距离
    // 两点距离太长的话就进行插点
    if (dis >= distance)
    {
      // 计算(x,y)两点的距离
      double sqrt_val = sqrt((input(i + 1, 0) - input(i, 0)) * (input(i + 1, 0) - input(i, 0)) +
                             (input(i + 1, 1) - input(i, 1)) * (input(i + 1, 1) - input(i, 1)));
      // 计算角度
      double sin_a = (input(i + 1, 1) - input(i, 1)) / sqrt_val;
      double cos_a = (input(i + 1, 0) - input(i, 0)) / sqrt_val;
      // 两点之间要插值的插值点的数量
      int num = dis / interval_dis; // 分割了一下
      // 插入点
      for (int j = 0; j < num; j++)
      {
        // i=0,j=0的时候其实是插入起点
        p.x = input(i, 0) + j * interval_dis * cos_a;
        p.y = input(i, 1) + j * interval_dis * sin_a;
        p.z = input(i, 2);
        vec_3d.push_back(p);
      }
    }
    // 3.有些点原本比较近，不需要插点，但是也要补进去，不然会缺失,dis >= 1防止点太密集
    else if (dis < distance)
    {
      p.x = input(i, 0);
      p.y = input(i, 1);
      p.z = input(i, 2);
      vec_3d.push_back(p);
    }
  }
  // 4.漏了终点，需要加上
  p.x = input(input.rows() - 1, 0);
  p.y = input(input.rows() - 1, 1);
  p.z = input(input.rows() - 1, 2);
  vec_3d.push_back(p);

  // 传给输出矩阵output
  output = Eigen::MatrixXd::Zero(vec_3d.size(), 3);
  int j = 0;
  for (std::vector<Point3D_s>::iterator it = vec_3d.begin(); it != vec_3d.end(); it++)
  {
    output(j, 0) = (*it).x;
    output(j, 1) = (*it).y;
    output(j, 2) = (*it).z;
    j++;
  }
}

/************************参考线平滑算法*************************/
void ReferenceLine_Split::Publish(Eigen::MatrixXd &path_point_after_interpolation_) {
    // nav_msgs::Path类型的参考线发布referenceline_pub_
    referenceline.poses.clear();
    referenceline.header.frame_id = Frame_id;
    referenceline.header.stamp = ros::Time::now();

    geometry_msgs::PoseStamped pose_stamp;
    pose_stamp.header.frame_id = Frame_id;
    pose_stamp.header.stamp = ros::Time::now();
    // 优化参考路径点
    bool status = false;                                // 是否平滑成功标志
    std::vector<std::pair<double, double>> raw_point2d; // 参考路径点
    std::vector<double> box_bounds;                     // 边界条件
    // 获取原始参考路径点和边界值
    for (int i = 0; i < path_point_after_interpolation_.rows(); i++)
    {
        raw_point2d.emplace_back(path_point_after_interpolation_(i, 0), path_point_after_interpolation_(i, 1));
        box_bounds.emplace_back(0.25);
    }
    box_bounds.front() = 0.0;
    box_bounds.back() = 0.0;

    // box contraints on pos are used in cos theta smoother, thus shrink the
    // bounds by 1.0 / sqrt(2.0)
    // 裕度收缩
    std::vector<double> bounds = box_bounds;
    const double box_ratio = 1.0 / std::sqrt(2.0);
    for (auto &bound : bounds)
    {
        bound *= box_ratio;
    }

    //  * 二次规划QP
    //  * 0.5x'Hx + f'x = min
    //  * lb < Ax < ub
    std::vector<std::pair<double, double>> smoothed_point2d;
    std::vector<double> opt_x;
    std::vector<double> opt_y;
    // 使用FemPosSmooth
    FemPosDeviationSmoother smoother; // 优化曲线
    // 从其输入的参数来看，主要是AnchorPoint中的x，y和横纵向的裕度，输出是平滑后的point
    status = smoother.Solve(raw_point2d, bounds, &opt_x, &opt_y);
    // 求解失败
    if (status == false) {
        ROS_WARN("Fem Pos reference line smoothing failed!");
    } else {
        if (opt_x.size() < 2 || opt_y.size() < 2) {
            ROS_WARN("Return by fem pos smoother is wrong. Size smaller than 2 ");
        }
        for (size_t i = 0; i < opt_x.size(); i++)
        {
            smoothed_point2d.emplace_back(opt_x[i], opt_y[i]);
        }
        DeNormalizePoints(&smoothed_point2d);

        // 发布
        for (int i = 0; i < smoothed_point2d.size(); i++)
        {
            pose_stamp.pose.position.x = smoothed_point2d[i].first;
            pose_stamp.pose.position.y = smoothed_point2d[i].second;
            pose_stamp.pose.position.z = 0;
            referenceline.poses.push_back(pose_stamp);
        }
        referenceline_pub_.publish(referenceline); // 只能发布一次
    }

}

void ReferenceLine_Split::NormalizePoints(std::vector<std::pair<double, double>> *xy_points)
{
    zero_x_ = xy_points->front().first;
    zero_y_ = xy_points->front().second;
    std::for_each(xy_points->begin(), xy_points->end(), [this](std::pair<double, double> &point)
    {
        auto curr_x = point.first;
        auto curr_y = point.second;
        std::pair<double, double> xy(curr_x - zero_x_, curr_y - zero_y_);
        point = std::move(xy); 
    }
    );
}

void ReferenceLine_Split::DeNormalizePoints(std::vector<std::pair<double, double>> *xy_points)
{
    std::for_each(xy_points->begin(), xy_points->end(), [this](std::pair<double, double> &point)
    {
        auto curr_x = point.first;
        auto curr_y = point.second;
        std::pair<double, double> xy(curr_x + zero_x_, curr_y + zero_y_);
        point = std::move(xy); 
    }
    );
}