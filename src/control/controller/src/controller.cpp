#include "../include/controller.h"

using namespace std;

namespace rviz_pnc {
Controller::Controller() {
    this->first_loop_ = true;
    this->current_time_ = 0.0;
    this->commands_ = {0.0, 0.0, 0.0}; // throttle steer brake
    /*****************整车参数*****************/
    this->wheel_base_ = 2.947;
    this->cf_ = -175016;
    this->cr_ = -130634;
    this->mass_ = 2020;
    this->mass_front_ = 920;
    this->mass_rear_ = 1100;
    this->lf_ = 1.265;
    this->lr_ = 2.947 - 1.265;
    this->Iz_ = pow(this->lf_, 2) * this->mass_front_ + pow(this->lr_, 2) * this->mass_rear_;
    this->max_front_wheel_ = 60.0;
}
// 更新车辆当前状态
void Controller::update_vehicle_state(const VehicleState& current_pose, const double &timestamp) {
    this->current_pose_ = current_pose;
    this->current_time_ = timestamp;
}

// 更新期望速度值
void Controller::update_desired_speed() {
    if (this->closet_index_ < this->waypoints_.size() - 1) this->desired_speed_ = this->waypoints_[this->closet_index_].velocity;
    else this->desired_speed_ = this->waypoints_[this->waypoints_.size() - 1].velocity;
}

// 更新规划轨迹点
void Controller::update_waypoints(const std::vector<VehicleState> &local_waypoints) {
    this->waypoints_ = local_waypoints;
    cal_heading(this->waypoints_);
    this->closet_index_ = search_closest_index(this->current_pose_, this->waypoints_);
}

// 计算参考路径的横摆角,曲率
void Controller::cal_heading(std::vector<VehicleState> &waypoints) {
    double dx = 0.0;
    double ddx = 0.0;
    double dy = 0.0;
    double ddy = 0.0;
    for (int i = 0; i < this->waypoints_.size(); i++) {
        if (i == 0) {
            dx = waypoints_[i + 1].x - waypoints_[i].x;
            dy = waypoints_[i + 1].y - waypoints_[i].y;
            ddx = waypoints_[2].x + waypoints_[0].x - 2 * waypoints_[1].x;
            ddy = waypoints_[2].y + waypoints_[0].y - 2 * waypoints_[1].y;
        } else if (i == waypoints_.size() - 1) {
            dx = waypoints_[i].x - waypoints_[i - 1].x;
            dy = waypoints_[i].y - waypoints_[i - 1].y;
            ddx = waypoints_[i].x + waypoints_[i - 2].x - 2 * waypoints_[i - 1].x;
            ddy = waypoints_[i].y + waypoints_[i - 2].y - 2 * waypoints_[i - 1].y;
        } else {
            dx = waypoints_[i + 1].x - waypoints_[i].x;
            dy = waypoints_[i + 1].y - waypoints_[i].y;
            ddx = waypoints_[i + 1].x + waypoints_[i - 1].x - 2 * waypoints_[i].x;
            ddy = waypoints_[i + 1].y + waypoints_[i - 1].y - 2 * waypoints_[i].y;
        }
        waypoints_[i].heading = atan2(dy, dx);
        //计算曲率:设曲线r(t) =(x(t),y(t)),则曲率k=(x'y" - x"y')/((x')^2 + (y')^2)^(3/2).
        waypoints_[i].kappa = (ddy * dx - ddx * dy) / pow((dx * dx + dy * dy), 3.0 / 2);
    }
}

// 油门输入
void Controller::set_throttle(const double &input_throttle)
{
    double throttle = min(max(input_throttle, 0.0), 1.0); // 确保油门在[0.0 1.0]
    commands_[0] = throttle;
}

// 转向输入
void Controller::set_steer(const double &input_steer_in_rad)
{
    double steer = rad_to_steer(input_steer_in_rad, this->max_front_wheel_);
    commands_[1] = steer;
}

// 刹车输入
void Controller::set_brake(const double &input_brake)
{
    double brake = min(max(input_brake, 0.0), 1.0); // 确保刹车在[0.0 1.0]
    commands_[2] = brake;
}

// 查找匹配点索引
int Controller::search_closest_index(const VehicleState& current_pose, const std::vector<VehicleState>& waypoints) {
    double dist;
    for (int i = 0; i < waypoints_.size(); i++) {
        dist = cal_distance(waypoints_[i].x, waypoints_[i].y, current_pose_.x, current_pose_.y);
        if (dist < this->closet_distance_) {
            this->closet_distance_ = dist;
            this->closet_index_ = i;
        }
        return this->closet_index_;
    }
}

// 初始化临时值
void Controller::reset() {
    tmp["v"] = 0.0;
    tmp["t"] = 0.0;
    tmp["throttle"] = 0.0;
    tmp["v_error"] = 0.0;
    tmp["alpha"] = 0.0;
    tmp["delta"] = 0.0;
}

// 更新控制
void Controller::update_controls(const double &frequency_update) {
    update_desired_speed();
    double throttle_output = 0;
    double steer_output = 0;
    double brake_output = 0;

    if (first_loop_)
    {
      reset();
    } else {
        
    }
}

}