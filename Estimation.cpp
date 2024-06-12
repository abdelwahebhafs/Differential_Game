#include "Global.h"
#include "State.h"
#include "Estimation.h"
#include "Robot.h"
#include "Human.h"

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <chrono>
#include <nlopt.hpp>
#include <iomanip>

using namespace std;
using namespace Eigen;
using namespace nlopt;

Estimation E;
vector<double> x,lb,ub;
// Create NLopt object and set parameters
opt opt0(LN_BOBYQA, 2);/* algorithm and dimensionality */

void Construct_Estimation(Estimation &E, const  State &S){
    E.theta=qh.diagonal();
    E.A.resize(n,n*(ne+np));
    E.Br.resize(n,(ne+np)*m);
    E.Bh.resize(n,(ne+np)*m);
    E.C.resize(n,(ne+np)*m);
    E.A = S.A.block(0, 0, n, (ne+np)*n);
    E.Br= S.B.block(0, 0, n, (ne+np)*m);
    E.Bh= S.B.block(0, 0, n, (ne+np)*m);
    E.C = S.C.block(0, 0, n, (ne+np));
    E.Pr.resize(n,n); E.Pr.setZero();
    E.Ph.resize(n,n); E.Ph.setZero();
    E.ar.resize(n); E.ar.setZero();
    E.ah.resize(n); E.ah.setZero();
    E.Rrh = rrh;
    E.Rr = rr;
    E.Rh = rh;
    E.Qr = qr;
    E.Qh = E.theta.asDiagonal();
    E.xi = S.xi;
    
    x.push_back(10);
    x.push_back(0.1);
    lb.push_back(-200);
    lb.push_back(-200);
    ub.push_back(1000);
    ub.push_back(1000);
    opt0.set_lower_bounds(lb);
    opt0.set_upper_bounds(ub);
    opt0.set_maxeval(20);
    // opt0.set_initial_step({1,0.01});
    opt0.set_min_objective(Estimation_fxn,nullptr);

    cout <<"Estimation Structure Constructed.\n";

}

void Update_Estimation(Estimation &E, const  State &S){   int i = nc - ne ;
    E.A = S.A.block(0, i*n, n, (ne+np)*n);
    E.Br= S.B.block(0,i*m, n, (ne+np)*m);
    E.Bh= S.B.block(0,i*m, n, (ne+np)*m);
    E.C = S.C.block(0, i, n, (ne+np));
    
    E.Qh=E.theta.asDiagonal();
    E.xi = S.xi_arr[i];
    E.Uh_arr = Uh_arr;

    E.ah.setZero(); E.ar.setZero(); E.Ph.setZero(); E.Pr.setZero();
}

double Estimation_fxn(const vector<double>& x, vector<double>& grad,void* data){
    MatrixXd Brt_i,Brht_i,Bht_i,cht_i,crt_i,Qh(n,n);
    MatrixXd Aht_i,Art_i,Frt_i,Fht_i;
    MatrixXd Rr_inv = E.Rr.inverse(),Rh_inv = E.Rh.inverse();
    VectorXd error_vectors(m),sero(m),XV(2),XB(2),r(2);  error_vectors.setZero();  sero.setZero();
    double error=0;
    
    Qh <<x[0],0,0,x[1];

    for (int i = 0; i <ne; i++) {  
        E.ah.setZero(); E.ar.setZero(); E.Ph.setZero(); E.Pr.setZero();
        for (int j = np-1; j > 0; j--) {   
            //////////////////////////////////////////////////////////////////////
    // XV<<x[0],x[1];
    // XB<<1,M_PI;
    // r=Qh*XV-XB;
    // return r.sum();
//////////////////////////////////////////////////////////////////////
            Brt_i.noalias() = E.Br.block(0,(j+i),n,m) * Rr_inv * (E.Br.block(0,(j+i),n,m)).transpose();
            Bht_i.noalias() = E.Bh.block(0,(j+i),n,m) * Rh_inv * (E.Bh.block(0,(j+i),n,m)).transpose();
            Brht_i.noalias()= E.Br.block(0,(j+i),n,m) * Rr_inv * E.Rrh * Rr_inv * (E.Br.block(0,(j+i),n,m)).transpose();
            Art_i.noalias() = E.A.block(0,(j+i)*n,n,n) - Bht_i * E.Ph;
            Aht_i.noalias() = E.A.block(0,(j+i)*n,n,n) - Brt_i * E.Pr;
            crt_i.noalias() = E.C.block(0,(j+i),n,m) - Bht_i * E.ah;
            cht_i.noalias() = E.C.block(0,(j+i),n,m) - Brt_i * E.ar;

            Frt_i.noalias() = E.Pr * Art_i;
            Fht_i.noalias() = E.Ph * Aht_i;

            // Update equations
            E.Pr.noalias() += integration_step * (Frt_i + Frt_i.transpose() + E.Qr - E.Pr * Brt_i * E.Pr + E.Ph * Brht_i * E.Ph);
            E.ar.noalias() += integration_step * ((Art_i - Brt_i * E.Pr).transpose() * E.ar + E.Pr * crt_i + E.Ph * Brht_i * E.ah);
            E.Ph.noalias() += integration_step * (Fht_i + Fht_i.transpose() + Qh - E.Ph * Bht_i * E.Ph);
            E.ah.noalias() += integration_step * ((Aht_i - Bht_i * E.Ph).transpose() * E.ah + E.Ph * cht_i);
        
        }

        E.Uh = -E.Rh.inverse() * E.Bh.block(0,i,n,m).transpose() * (E.Ph * E.xi + E.ah);
        E.Ur = -E.Rr.inverse() * E.Br.block(0,i,n,m).transpose() * (E.Pr * E.xi + E.ar);
        E.xi += time_step * (E.A.block(0, i*n, n, n) * E.xi + E.Br.block(0,i,n,m) * E.Ur + E.Bh.block(0,i,n,m) * E.Uh + E.C.block(0, i, n, 1));
        error_vectors += (E.Uh-E.Uh_arr[nc-ne+i])*(E.Uh-E.Uh_arr[nc-ne+i]);
    }
    
    error = error_vectors.sum();
    return error;
}

double Estimation_Loop(Estimation &EE, const  State &S){
    E=EE;
    Update_Estimation(E,S);
    double minf;
    cout<< x[0]<<"  "<< x[1]<<"          ";
    result result0 = opt0.optimize(x, minf);
    cout << "found minimum at f(" << x[0] << "," << x[1] << ") = " << minf <<"  "<<result0 << endl;
    return minf;
}

