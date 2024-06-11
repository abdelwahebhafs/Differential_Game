#ifndef PLANNING_H
#define PLANNING_H

#include "State.h"

#include <eigen3/Eigen/Dense>

using namespace Eigen;

struct Planning {
    MatrixXd A,Br,Bh,C,Qr,Rr,Rrh,Pr,Qh,Rh,Ph;
    VectorXd ar,ah;
    
};

void Construct_Planning(Planning &P, const  State &S);
void Update_Planning(Planning &P, const  State &S);

#endif