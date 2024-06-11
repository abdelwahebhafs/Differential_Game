#ifndef ESTIMATION_H
#define ESTIMATION_H

#include "State.h"

#include <eigen3/Eigen/Dense>

using namespace Eigen;

struct Estimation {
    MatrixXd A, Br, Bh, C, Qr, Qh, Rr, Rh, Rrh, Ph, Pr;
    VectorXd ar, ah, theta,xi,Uh,Ur;
    
};

void Construct_Estimation(Estimation &E,const  State &S);
double Estimation_Loop(Estimation &E,const State &S);

#endif