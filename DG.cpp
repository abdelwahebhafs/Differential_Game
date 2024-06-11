#include "Planning.h"
#include "Global.h"
#include "DG.h"

#include <iostream>
#include <eigen3/Eigen/Dense>

using namespace Eigen;
using namespace std;

void DG(Planning &P){ 
    MatrixXd Brt_i,Brht_i,Bht_i,cht_i,crt_i;
    MatrixXd Aht_i,Art_i,Frt_i,Fht_i;
    MatrixXd Rr_inv = P.Rr.inverse(),Rh_inv = P.Rh.inverse();
    int j;

    for (int j = np-1; j > 0; j--) {   
        Brt_i.noalias() = P.Br.block(0,j,n,m) * Rr_inv * (P.Br.block(0,j,n,m)).transpose();
        Bht_i.noalias() = P.Bh.block(0,j,n,m) * Rh_inv * (P.Bh.block(0,j,n,m)).transpose();
        Brht_i.noalias() = P.Br.block(0,j,n,m) * Rr_inv * P.Rrh * Rr_inv * (P.Br.block(0,j,n,m)).transpose();
        Art_i.noalias() = P.A.block(0,j*n,n,n) - Bht_i * P.Ph;
        Aht_i.noalias() = P.A.block(0,j*n,n,n) - Brt_i * P.Pr;
        crt_i.noalias() = P.C.block(0,j,n,m) - Bht_i * P.ah;
        cht_i.noalias() = P.C.block(0,j,n,m) - Brt_i * P.ar;

        Frt_i.noalias() = P.Pr * Art_i;
        Fht_i.noalias() = P.Ph * Aht_i;

        // Update equations
        P.Pr.noalias() += integration_step * (Frt_i + Frt_i.transpose() + P.Qr - P.Pr * Brt_i * P.Pr + P.Ph * Brht_i * P.Ph);
        P.ar.noalias() += integration_step * ((Art_i - Brt_i * P.Pr).transpose() * P.ar + P.Pr * crt_i + P.Ph * Brht_i * P.ah);
        P.Ph.noalias() += integration_step * (Fht_i + Fht_i.transpose() + P.Qh - P.Ph * Bht_i * P.Ph);
        P.ah.noalias() += integration_step * ((Aht_i - Bht_i * P.Ph).transpose() * P.ah + P.Ph * cht_i);
        
    }
    
}