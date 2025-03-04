#include "Global.h"
#include "State.h"
#include "Estimation.h"
#include "Robot.h"
#include "Human.h"

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <chrono>

using namespace std;
using namespace Eigen;


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

double Estimation_fxn(Estimation &E){
    MatrixXd Brt_i,Brht_i,Bht_i,cht_i,crt_i;
    MatrixXd Aht_i,Art_i,Frt_i,Fht_i;
    MatrixXd Rr_inv = E.Rr.inverse(),Rh_inv = E.Rh.inverse();
    VectorXd error_vectors(m),sero(m);  error_vectors.setZero();  sero.setZero();
    double error=0;

    for (int i = 0; i <ne; i++) {  
        E.ah.setZero(); E.ar.setZero(); E.Ph.setZero(); E.Pr.setZero();
        for (int j = np-1; j > 0; j--) {   
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
            E.Ph.noalias() += integration_step * (Fht_i + Fht_i.transpose() + E.Qh - E.Ph * Bht_i * E.Ph);
            E.ah.noalias() += integration_step * ((Aht_i - Bht_i * E.Ph).transpose() * E.ah + E.Ph * cht_i);
        
        }

        E.Uh = -E.Rh.inverse() * E.Bh.block(0,i,n,m).transpose() * (E.Ph * E.xi + E.ah);
        E.Ur = -E.Rr.inverse() * E.Br.block(0,i,n,m).transpose() * (E.Pr * E.xi + E.ar);
        E.xi += time_step * (E.A.block(0, i*n, n, n) * E.xi + E.Br.block(0,i,n,m) * E.Ur + E.Bh.block(0,i,n,m) * E.Uh + E.C.block(0, i, n, 1));
        error_vectors += (E.Uh-E.Uh_arr[nc-ne+i])*(E.Uh-E.Uh_arr[nc-ne+i]);
        if (error_vectors!=sero){
            cout <<i<<" ";
        }
    }
    
    error = error_vectors.sum();
    return error;
}

double Estimation_Loop(Estimation &E, const  State &S){
    
    Update_Estimation(E,S);
    return Estimation_fxn(E);
}

