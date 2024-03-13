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
    /******************横向控制LQR参数****************/
    this->closet_index_ = 0;
    this->Q_ = Q_;
    this->R_ = R_;

    /************纵向PID控制参数**************/
    this->sum_pid_error_ = 0.0; // PID累计误差
    this->kp_ = kp_;
    this->ki_ = ki_;
    this->kd_ = kd_;
    this->desired_speed_ = 0.0;
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
    tmp["brake"] = 0.0;
}

/****************横向控制LQR****************/
// 求解k值
Eigen::MatrixXd Controller::cal_dlqr(const double& vx, const int& target_index) {
    double ref_yaw = this->waypoints_[target_index].heading;
    
    double ref_delta = atan2(wheel_base_ * this->waypoints_[target_index].kappa, 1);
    Eigen::MatrixXd Ad(3, 3), Bd(3, 2);
    // 离散化状态方程
    KinematicModel model;
    model.v = vx;
    Ad = model.stateSpace(ref_delta, ref_yaw)[0];
    Bd = model.stateSpace(ref_delta, ref_yaw)[1];
    // 设置迭代次数1000 预设精度EPS 1.0e-4
    LQR lqr(1000); 
    // 求解Riccati方程
    MatrixXd P = lqr.calRicatti(Ad, Bd, Q_, R_);

    MatrixXd K = (R_ + Bd.transpose() * P * Bd).inverse() * Bd.transpose() * P * Ad;
    return K;
}
// LQR_kinematics
double Controller::cal_delta() {
    double delta = 0.0;
    int target_index = search_closest_index(this->current_pose_, this->waypoints_);
    Matrix<double, 3, 1> error;
    double heading_error = this->current_pose_.heading - this->waypoints_[target_index].heading;

    error << this->current_pose_.x - this->waypoints_[target_index].x,
             this->current_pose_.x - this->waypoints_[target_index].x,
             normalize_angle(heading_error);
    MatrixXd K = cal_dlqr(this->desired_speed_, target_index);
    MatrixXd u = -K * error;
    delta = u(1, 0) + atan2(this->waypoints_[target_index].kappa, 1);
    if (isnan(delta))
    {
      delta = tmp["delta"];
    }
}

/*****************纵向控制*********************/
double Controller::cal_longitudinal(const double& dt, const double& speed, const double& desired_speed) {
    double v_err = desired_speed - speed;
    sum_pid_error_ += v_err * dt;
    double throttle = v_err * this->kp_ + sum_pid_error_ * this->ki_ + (v_err - tmp["v_error"]) * this->kd_ / dt;
    tmp["throttle"] = throttle;
    tmp["v_error"] = v_err;
    return throttle;
}


// 更新控制指令
void Controller::update_controls(const double &frequency_update) {
    update_desired_speed();
    double throttle_output = 0;
    double steer_output = 0;
    double brake_output = 0;

    if (first_loop_)
    {
      reset();
    } else {
        if (waypoints_.size() > 1) {
            double dt = 1 / frequency_update;
            // 纵向控制
            brake_output = 0.0;
            throttle_output = cal_longitudinal(dt, this->current_pose_.velocity, this->desired_speed_);
            // 横向控制
            steer_output = cal_delta();
            steer_output = normalize_angle(steer_output);
            // 限制前轮最大转角[-60度～60度]
            if (steer_output >= degree_to_rad(60.0)) steer_output = degree_to_rad(60.0);
            else if (steer_output <= -degree_to_rad(60.0)) steer_output = -degree_to_rad(60.0);
            } else {
                throttle_output = tmp["throttle"];
                steer_output = tmp["delta"];
                brake_output = tmp["brake"];
            }

            if (this->desired_speed_ < 0.1) {
                throttle_output = 0;
                steer_output = 0;
                brake_output = 1;
            }
            // 设置控制指令
            set_throttle(throttle_output); //  (0 to 1)
            set_steer(-steer_output);      // (-1 to 1)
            set_brake(brake_output);       // (0 to 1)
            // 存储上一个周期的值
            tmp["v"] = this->current_pose_.velocity_x;
            tmp["t"] = this->current_time_;
    }
    first_loop_ = false;
}
} // namespace rviz_pnc