#pragma once
#include "../common/common.h"
#include "../include/kinematic_model.h"
#include "../include/lqr_controller.h"

#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <unordered_map>
#include <string>

namespace rviz_pnc {
class Controller {
public:
    Controller() = default;
    // 车辆状态更新
    void update_vehicle_state(const VehicleState& current_pose, const double &timestamp);
    // 期望速度更新
    void update_desired_speed();
    // 更新路径点
    void update_waypoints(const std::vector<VehicleState> &local_waypoints);
    // 计算参考路径的横摆角,曲率
    void cal_heading(std::vector<VehicleState> &waypoints);

    // 获取控制命令
    std::vector<double> get_commands() { return commands_; }
    // 油门设置
    void set_throttle(const double &input_throttle);
    // 转向量设置
    void set_steer(const double &input_steer_in_rad);
    // 刹车设置
    void set_brake(const double &input_brake);
    // 查找最近点索引
    int search_closest_index(const VehicleState& current_pose, const std::vector<VehicleState>& waypoints);
    // 更新控制
    void update_controls(const double &frequency_update);
    // 初始化临时值
    void reset();

    /************************纵向控制*************************/
    double cal_longitudinal(const double& dt, const double& speed, const double& desired_speed);

    /************************横向控制*************************/
    // 求解离散LQR
    Eigen::MatrixXd cal_dlqr(const double& vx, const int& target_index);

    // 计算方向转角
    double cal_delta();


public:
    /********************整车参数********************/
    double wheel_base_;        // 轴距
    double cf_;                // 前轮侧偏刚度,左右轮之和
    double cr_;                // 后轮侧偏刚度, 左右轮之和
    double mass_;              // 车辆载荷
    double mass_front_;        // 前悬质量
    double mass_rear_;         // 后悬质量
    double lf_;                // 前轴中心到质心的距离
    double lr_;                // 后轴中心到质心的距离
    double Iz_;                // 车辆绕z轴转动的转动惯量
    double max_front_wheel_;   // 最大前轮转向角(度)
protected:
    VehicleState current_pose_;            // 车辆当前状态
    double current_time_;                  // 当前时间戳
    bool first_loop_;                      // 判断是否为第一次循环
    std::vector<VehicleState> waypoints_;  // 局部路径信息 x,y, x方向速度
    std::vector<double> commands_;         // throttle, steer, brake

    int closet_index_;                    // 最近匹配点下标
    double closet_distance_;              // 与最近匹配点的距离
    Eigen::MatrixXd Q_;                   // Q矩阵
    Eigen::MatrixXd R_;                   // R矩阵
    
    std::unordered_map<std::string, double> tmp; // 临时存储上一循环周期的变量

    double sum_pid_error_; // pid累计误差
    double kp_;
    double ki_;
    double kd_;
    double desired_speed_;                // 期望速度
};

} // namespace rviz_pnc