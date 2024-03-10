#include "../common/common.h"
#include <iostream>

using namespace std;

namespace rviz_pnc {
// 计算两点之间的距离
double cal_distance(const double &x1, const double &y1, const double &x2, const double &y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return sqrt(dx * dx + dy * dy);
}
// 角度归一化
double normalize_angle(double &angle) {
    double a = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (a < 0.0) {
        a += (2.0 * M_PI);
    }
    return a - M_PI;
}
// 度数变为弧度
double degree_to_rad(const double &degree) {
    return degree * M_PI / 180;
}
// 弧度转换为方向盘控制指令
double rad_to_steer(const double &steer_in_rad, const double &max_degree) {
    return steer_in_rad * 180.0 / max_degree / M_PI;
}

}