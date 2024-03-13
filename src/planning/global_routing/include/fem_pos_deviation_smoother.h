#pragma once

#include <utility>
#include <vector>
#include <coin/IpIpoptApplication.hpp>
#include <coin/IpSolveStatistics.hpp>

/*
 * @brief:
 * This class solve an optimization problem:
 * Y
 * |
 * |                       P(x1, y1)  P(x2, y2)
 * |            P(x0, y0)                       ... P(x(k-1), y(k-1))
 * |P(start)
 * |
 * |________________________________________________________ X
 *
 *
 * Given an initial set of points from 0 to k-1,  The goal is to find a set of
 * points which makes the line P(start), P0, P(1) ... P(k-1) "smooth".
 */

class FemPosDeviationSmoother
{
public:
  explicit FemPosDeviationSmoother();

  bool Solve(const std::vector<std::pair<double, double>>& raw_point2d, const std::vector<double>& bounds,
             std::vector<double>* opt_x, std::vector<double>* opt_y);

  bool QpWithOsqp(const std::vector<std::pair<double, double>>& raw_point2d, const std::vector<double>& bounds,
                  std::vector<double>* opt_x, std::vector<double>* opt_y);

  bool NlpWithIpopt(const std::vector<std::pair<double, double>>& raw_point2d, const std::vector<double>& bounds,
                    std::vector<double>* opt_x, std::vector<double>* opt_y);

  bool SqpWithOsqp(const std::vector<std::pair<double, double>>& raw_point2d, const std::vector<double>& bounds,
                   std::vector<double>* opt_x, std::vector<double>* opt_y);

private:
  /*
  如果考虑参考线的曲率约束，其优化问题是非线性的，
  可以使用ipopt非线性求解器求解（内点法），也可以使用osqp二次规划求解器来用SQP方法求解；
  如果不考虑曲率约束，则直接用osqp求解二次规划问题。
  */
  // 是否使用曲率约束
  bool apply_curvature_constraint = false;  
  // 是否使用SQP方法求解
  bool use_sqp = false;                     
};
