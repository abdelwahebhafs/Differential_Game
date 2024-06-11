#include "Robot.h"
#include "Global.h"

#include <eigen3/Eigen/Dense>

using namespace Eigen;

const double I_robot = 0.0152;
const double m_robot = 1;
const double l_robot = 0.1;
const double D_robot = 0.5;


MatrixXd qr = (MatrixXd(n, n) << 50, 0, 0, 0.1).finished();
MatrixXd rr = (MatrixXd(m, m) << 1).finished();
MatrixXd rrh= (MatrixXd(m, m) << 1).finished();
VectorXd Ur = VectorXd::Zero(m);
vector<VectorXd>  Ur_arr;