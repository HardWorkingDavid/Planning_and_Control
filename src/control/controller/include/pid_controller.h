#pragma once
#include <iostream>

namespace control {

class PIDController {
 public:
  PIDController(const double kp, const double ki, const double kd);
  ~PIDController() = default;

  void Reset();

  /**
   * @brief 根据误差计算控制值
   * @param error 误差值，期望值和测量值之间的差值
   * @param dt 采样时间间隔
   * @return 控制量
   */
  double Control(const double error, const double dt);

 protected:
  double kp_ = 0.0;
  double ki_ = 0.0;
  double kd_ = 0.0;
  double previous_error_ = 0.0;
  double previous_output_ = 0.0;
  double integral_ = 0.0;
  bool first_hit_ = false;
};

}  // namespace control
