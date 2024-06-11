#include "Print.h"
#include "Global.h"

#include <eigen3/Eigen/Dense>
#include <vector>
#include <fstream>
#include <iostream>

using namespace Eigen;
using namespace std;

void printToCsv(const std::vector<MatrixXd>& PH, const std::vector<MatrixXd>& PR, const std::vector<VectorXd>& AH, const std::vector<VectorXd>& AR) {
  std::ofstream csvFile("csv/P.csv");

  if (csvFile.is_open()) {
    // Write the header row for each column
    csvFile << " I    | PH(0 0), PH(0 1), PH(1 0), PH(1 1)  | PR(0 0), PR(0 1), PR(1 0), PR(1 1)  | AH(0), AH(1)  | AR(0), AR(1)\n";
    size_t min_size = min({PH.size(), PR.size()});
    // Loop through each element and write to the CSV file
    for (size_t i = 0; i < min_size; ++i){
      // Write matrix elements separated by commas
      csvFile <<  i<<"|"<<PH[i](0, 0) << ","<<PH[i](0, 1)<<","<<PH[i](1, 0)<<","<<PH[i](1, 1) <<" | "<<PR[i](0, 0) << ","<<PR[i](0, 1)<<","<<PR[i](1, 0)<<","<<PR[i](1, 1) <<" | "<<AH[i](0) <<","<<AH[i](1)<<" | "<<AR[i](0) <<","<<AR[i](1)<<endl;
    }

    csvFile.close();
  } else {
    std::cerr << "Error: Could not open file for writing!" << std::endl;
  }
}

void print_to_csv(const vector<VectorXd>& vec1, const vector<VectorXd>& vec2, const vector<VectorXd>& vec3, const vector<VectorXd>& vec4) {
  // Open the CSV file for writing
  ofstream csv_file("csv/data.csv");

  // Check if the file is open for writing
  if (!csv_file.is_open()) {
    cerr << "Error: Could not open file '" << "csv/data.csv" << "'" << endl;
    return;
  }

  // Write the header row
  csv_file << "i , Uh , Ur , E.Uh , E.Ur\n";

  // Make sure all vectors have the same size
  size_t min_size = min({vec1.size(), vec2.size()});

  // Write data rows
  for (size_t i = 0; i < min_size; ++i) {
    if(i<estimation_horizon/time_step) csv_file <<i<<","<< vec2[i] << "," << vec1[i] << endl;
    else{
      csv_file <<i<<","<< vec2[i] << "," << vec1[i]<<","<< vec3[i-estimation_horizon/time_step] << "," << vec4[i-estimation_horizon/time_step] << endl;
    }
  }

  csv_file.close();
  
}
