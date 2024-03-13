#pragma once

#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include "../common/common.h"

#define EPS 1.0e-4

using namespace std;
using namespace Eigen;

namespace rviz_pnc {

class LQR {
private:
    int N = 1000; // 默认迭代范围
public:
    LQR(int n);

    MatrixXd calRicatti(MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R);
    double LQRControl(vector<double> robot_state, vector<vector<double>> ref_path, double s0, MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R);
};
}
