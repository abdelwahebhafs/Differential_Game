#include "Global.h"
#include "State.h"
#include "Planning.h"
#include "Robot.h"
#include "Human.h"

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <chrono>

using namespace std;
using namespace Eigen;


void Construct_Planning(Planning &P, const  State &S){
    P.A.resize(n,n*np);
    P.Br.resize(n,np*m);
    P.Bh.resize(n,np*m);
    P.C.resize(n,np*m);
    P.A = S.A.block(0, 0, n, np*n);
    P.Br= S.B.block(0, 0, n, np*m);
    P.Bh= S.B.block(0, 0, n, np*m);
    P.C = S.C.block(0, 0, n, np);
    P.Pr.resize(n,n); P.Pr.setZero();
    P.Ph.resize(n,n); P.Ph.setZero();
    P.ar.resize(n); P.ar.setZero();
    P.ah.resize(n); P.ah.setZero();
    P.Rrh = rrh;
    P.Rh = rh;
    P.Rr = rr;
    P.Qh = qh;
    P.Qr = qr;
    

    cout <<"Planning Structure Constructed.\n";

}

void Update_Planning(Planning &P, const  State &S){   
    P.A = S.A.block(0, nc*n, n, np*n);
    P.Br= S.B.block(0,nc*m, n, np*m);
    P.Bh= S.B.block(0,nc*m, n, np*m);
    P.C = S.C.block(0, nc, n, np);
    
    P.ah.setZero(); P.ar.setZero(); P.Ph.setZero(); P.Pr.setZero();
}

