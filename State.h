#ifndef STATE_H
#define STATE_H

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

using namespace Eigen;
using namespace std;

struct State {

  MatrixXd A;
  MatrixXd B;
  MatrixXd C;
  VectorXd xi;
  vector<VectorXd> xi_arr;
  double I,D,M,l,g=9.81;

};

vector<string> split(const string &s, char delimiter);
void readCSV(const string &filename,State &S);
void Construct_State(State &a);

#endif 