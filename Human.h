#ifndef HUMAN_H
#define HUMAN_H

#include "Global.h"

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

using namespace Eigen;
using namespace std;

extern const double m_human, l_human, I_human, D_human;

extern MatrixXd qh,rh,rhr;
extern vector<VectorXd>  Uh_arr;
extern VectorXd Uh;

// void Update_Uh(vector<Vector2d>  &Uh_arr,VectorXd a);
#endif