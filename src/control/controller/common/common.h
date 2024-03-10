#pragma once
#include <cmath>

namespace rviz_pnc {
// 车辆位置信息
struct VehicleState {
    double x;
    double y;
    double z;
    double heading;          // 车辆朝向
    double kappa;            // 曲率
    double velocity_x;       // x方向速度值
    double velocity_y;       // y方向速度值
    double velocity;         // 合速度
};

// 计算两点之间的距离
double cal_distance(const double &x1, const double &y1, const double &x2, const double &y2);
// 角度归一化
double normalize_angle(double &angle);
// 度数变为弧度
double degree_to_rad(const double &degree);
// 弧度转换为方向盘控制指令
double rad_to_steer(const double &steer_in_rad, const double &max_degree);

} // namespace rviz_pnc