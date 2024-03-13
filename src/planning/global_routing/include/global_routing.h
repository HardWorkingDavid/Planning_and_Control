#pragma once

#include <iostream>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "std_msgs/String.h"
#include "../include/global_path_data.h"
#include "../include/ReferenceLine_Split.h"
#include <visualization_msgs/MarkerArray.h>

#include <waypoint_msgs/Waypoint.h>
#include <waypoint_msgs/WaypointArray.h>

#include "../../../control/controller/include/controller.h"
#include "tf/tf.h"
using namespace std;
using namespace rviz_pnc;

class GlobalRouting {
public:
    GlobalRouting();
    ~GlobalRouting();
    void thread_routing(void);

    void odom_call_back(const nav_msgs::Odometry &msg);
    void init_pose_call_back(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void goal_pose_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg);

    void Vehicle_Theta(const geometry_msgs::PoseArray &theta);
    void Vehicle_Kappa(const geometry_msgs::PoseArray &kappa);
    void Vehicle_Go(const std_msgs::String::ConstPtr &go);
    void Vehicle_Traj(const waypoint_msgs::WaypointArray::ConstPtr &msg);
    bool Vehicle_Stop(const geometry_msgs::Pose &goal_point, nav_msgs::Odometry &odom);

    void create_map();
    void publish_car_init_pose(const geometry_msgs::Pose &init_pose);
    Eigen::MatrixXd calc_dlqr(double vx, const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R);
public:
    /**************************整车参数****************************/
    double wheel_base_ = 2.947;
    double cf_ = -175016;
    double cr_ = -130634;
    double mass_ = 2020;
    double mass_front_ = 920;
    double mass_rear_ = 1100;
    double lf_ = 1.265;
    double lr_ = 2.947 - 1.265;
    double Iz_ = pow(this->lf_, 2) * this->mass_front_ + pow(this->lr_, 2) * this->mass_rear_;
    double max_front_wheel_ = 60.0;
protected:
    double Q_ex_k_, Q_ed_k_, Q_ephi_k_, R_value_k_;         // LQR_kinematics Q  R矩阵权重
    double kp_, ki_, kd_;                                // 纵向PID
private:
    Eigen::MatrixXd hdmap_way_points;           // 中心点坐标
    std::vector<double> speeds_, thetas_, kappas_; // 获取局部规划的速度与航向角,曲率
    std::vector<VehicleState> local_waypoints_;  // 局部规划路径点
    nav_msgs::Odometry car_odom_;
    VehicleState current_pose_;

    // 函数对象
    ReferenceLine_Split R_;                   // 参考线生成
    geometry_msgs::Vector3 msg_ros_;           // 发布控制数据
    visualization_msgs::MarkerArray obstacle_points_, map_points_;

    // 路径点
    nav_msgs::Path dynamic_points_;
    Eigen::MatrixXd hdmap_way_points_;

    // 点
    geometry_msgs::Pose init_pose_;
    geometry_msgs::Pose goal_pose_;

    // 可视化发布
    ros::Publisher obstacle_points_pub_;
    ros::Publisher goal_point_pub_;
    ros::Publisher map_points_pub_;
    ros::Publisher control_data_pub_;
    ros::Publisher vehicle_init_pose_pub_;
    // 可视化订阅
    ros::Subscriber odom_sub_;
    ros::Subscriber waypoints_sub_;
    ros::Subscriber start_pose_subscriber_;
    ros::Subscriber goal_pose_subscriber_;
    ros::Subscriber vehicle_theta;
    ros::Subscriber vehicle_kappa;
    ros::Subscriber vehicle_go;
    ros::Subscriber vehicle_traj;

    // 线程thread
    boost::thread *routing_thread_;
    // flag
    bool is_begin_reference;  // 控制参考线发布一次
    int obstacle_id;          // 手动标注障碍物位置坐标显示
    double vehicle_speed;     // 车速度
    double goal_distance;     // 目标距离
    bool is_vehicle_stop_set; // 判断车是否要停下来
    bool is_init_pose_set;   // 是否定义了起点
    bool is_goal_pose_set;    // 是否定义了终点
    string start_dynamic;     // 判断局部规划是否开始
};
