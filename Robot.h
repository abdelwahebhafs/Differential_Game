#ifndef ROBOT_H
#define ROBOT_H

#include "Global.h"

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

using namespace Eigen;
using namespace std;

extern const double m_robot, l_robot, I_robot, D_robot;

extern MatrixXd qr,rr,rrh;
extern vector<VectorXd> Ur_arr;
extern VectorXd Ur;


#endif