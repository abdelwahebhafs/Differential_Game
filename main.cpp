#include "Global.h"
#include "State.h"
#include "Planning.h"
#include "DG.h"
#include "Robot.h"
#include "Human.h"
#include "Print.h"
#include "Estimation.h"


#include <iostream>
#include <eigen3/Eigen/Dense>
#include <chrono>
#include <vector>

using namespace std;
using namespace Eigen;
using namespace chrono;


int main(){
  State S; 
  Planning P;     
  Estimation E;  
  Construct_State(S);   //Constructing matirxes A B,C according to the Trajectory.csv
  Construct_Planning(P,S);
  Construct_Estimation(E,S);

  vector<MatrixXd>PH, PR;
  vector<VectorXd> AR, AH, EUh, EUr;

  Uh_arr.clear();
  Ur_arr.clear();
  double error;
  auto start = high_resolution_clock::now();

  //Fist for loop 
  for(current_time; current_time<estimation_horizon+time_step;current_time+=time_step){     nc++;
    Update_Planning(P,S);
    DG(P);

    // Updating Ur, Uh, and xi
    Ur = -P.Rr.inverse() * (P.Br.block(0,0,n,m)).transpose() * (P.Pr * S.xi + P.ar);
    Uh = -P.Rh.inverse() * (P.Bh.block(0,0,n,m)).transpose() * (P.Ph * S.xi + P.ah);
    S.xi += time_step * ((P.A.block(0,0,n,n)* S.xi )+ ((P.Br.block(0,0,n,m))* Ur )+ ((P.Bh.block(0,0,n,m)) * Uh) + (P.C.block(0,0,n,1)));
    
    // Store them in their matirxes
    S.xi_arr.push_back(S.xi); Ur_arr.push_back(Ur); Uh_arr.push_back(Uh);
    PH.push_back(P.Ph); PR.push_back(P.Pr); AR.push_back(P.ar); AH.push_back(P.ah);

    cout <<current_time<<endl;
  }


  // Include Estimation in our loop
  for(current_time; current_time<task_duration;current_time+=time_step){                  nc++;
    Update_Planning(P,S);
    DG(P);

    // Updating Ur, Uh, and xi
    Ur = -P.Rr.inverse() * (P.Br.block(0,0,n,m)).transpose() * (P.Pr * S.xi + P.ar);
    Uh = -P.Rh.inverse() * (P.Bh.block(0,0,n,m)).transpose() * (P.Ph * S.xi + P.ah);
    S.xi += integration_step * ((P.A.block(0,0,n,n)* S.xi)+ ((P.Br.block(0,0,n,m))* Ur)+ ((P.Bh.block(0,0,n,m)) * Uh) + (P.C.block(0,0,n,1)));

    // Store them in their matirxes
    S.xi_arr.push_back(S.xi); Ur_arr.push_back(Ur); Uh_arr.push_back(Uh);
    PH.push_back(P.Ph); PR.push_back(P.Pr); AR.push_back(P.ar); AH.push_back(P.ah);


    //While(1) eventually in its own thread
    error = Estimation_Loop(E,S); // for nlopt another input theta and use that input for nlopt inside estimation to construct Qh and will retuan error and theta estimation


    EUh.push_back(E.Uh); EUr.push_back(E.Ur);
    cout <<current_time<<", "<<error<<endl;
  }

  auto end = high_resolution_clock::now();
  auto duration = duration_cast<microseconds>(end - start);

  print_to_csv(Ur_arr, Uh_arr, EUh, EUr);
  printToCsv(PH, PR, AH, AR);
  
  cout << "Time taken: " << duration.count() << " microseconds" << endl; 


  return 0;
}