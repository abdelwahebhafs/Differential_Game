#ifndef PRINT_H
#define PRINT_H

#include <eigen3/Eigen/Dense>
#include <vector>
#include <fstream>
#include <iostream>

using namespace Eigen;
using namespace std;

void print_to_csv(const vector<VectorXd>& vec1, const vector<VectorXd>& vec2, const vector<VectorXd>& vec3, const vector<VectorXd>& vec4);
void printToCsv(const std::vector<MatrixXd>& PH, const std::vector<MatrixXd>& PR, const std::vector<VectorXd>& AH, const std::vector<VectorXd>& AR);

#endif