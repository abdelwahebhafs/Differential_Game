#include "Human.h"
#include "Global.h"

#include <eigen3/Eigen/Dense>

using namespace Eigen;

const double I_human = 0.0497;
const double m_human = 0.427;
const double l_human = 0.3411;
const double D_human = 0;

MatrixXd qh = (MatrixXd(n, n) << 50, 0, 0, 0.1).finished();
MatrixXd rh = (MatrixXd(m, m) << 1).finished();
MatrixXd rhr= (MatrixXd(m, m) << 0).finished();
VectorXd Uh = VectorXd::Zero(m);
vector<VectorXd>  Uh_arr;

// void Update_Uh(vector<Vector2d>  &Uh_arr,VectorXd Uh){
//     Uh_arr.push_back(Uh);
// }