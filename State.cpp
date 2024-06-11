#include "Global.h"
#include "State.h"
#include "Planning.h"
#include "Human.h"
#include "Robot.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

vector<string> split(const string &s, char delimiter) {
    vector<string> tokens;
    string token;
    istringstream tokenStream(s);
    while (getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

// Function to read the CSV file and divide data into vectors
void readCSV(const string &filename,State &S) {
    ifstream file(filename);
    string line;
    int t=ntotal;

    if (!file.is_open()) {
        cerr << "Error opening file: " << filename << endl;
        return;
    }
    MatrixXd mA(n,n), mB(n,m),mC(n,1);
    mB << 0,1/S.I;
    mA << 0,1,
        0,-S.D/S.I;
    mC<<0,0;
    vector<string> parts;
    int i=0, lne=0;
        while (getline(file, line) && i <t) {
        parts = split(line, ',');
        if ((lne % int(integration_step/0.001))==0) {

            mA(1, 0) = S.M * S.g * S.l * sin(stod(parts[0]))/S.I;
            mC(1,0)=-(1/S.I)*(S.I*stod(parts[2])+S.D*stod(parts[1])+S.M*S.g*S.l*cos(stod(parts[0])));
            
            // if (i==100){ cout<< mA<<endl<<mC<<endl;}

            S.A.block(0, n*i, n, n) = mA;
            S.B.block(0, i*m, n, m) = mB;
            S.C.block(0, i, n, 1) = mC;
            i++;
    
        }
        lne++;

    }

    file.close();
}
void writeMatricesToCSV(const MatrixXd& A, const MatrixXd& B, const MatrixXd& C) {
    ofstream afile;
    int t=ntotal;
    afile.open("csv/A.csv", ios::out);
    if (afile.is_open()) {
        // Write elements of matrix A
        for (int i = 0; i < t; i++) {
            afile<<i<<" /  "<< A.block(1,i*2,1,2)<<","<<endl;
        }

        afile.close();
    } else {
        cerr << "Unable to open file: " << "A" << endl;
    }
    ofstream bfile;
    bfile.open("csv/B.csv",ios::out);
    if(bfile.is_open()){
        for(int i=0;i<t;i++){
            bfile<<i<<" /  "<<B.block(1,i,1,1)<<","<<endl;
        }
    } else {
        cerr << "Unable to open file: " << "B" << endl;
    }
    ofstream cfile;
    cfile.open("csv/C.csv",ios::out);
    if(cfile.is_open()){
        for(int i=0;i<t;i++){
            cfile<<i<<" /  "<<C.block(1,i,1,1)<<","<<endl;
        }
    } else {
        cerr << "Unable to open file: " << "C" << endl;
    }
}

void Construct_State(State &S) {
    
    S.M = m_human + m_robot;
    S.l = ((m_human*l_human)+(m_robot*l_robot))/S.M;
    S.I = I_human + I_robot;
    S.D = D_robot + D_human;

    S.xi = VectorXd::Ones(n);
    S.xi_arr.push_back(S.xi);
    
    string filename = "Trajectory.csv"; 
    S.A.resize(n,n*(ntotal+2*ne)),S.B.resize(n,(ntotal+2*ne)),S.C.resize(n,(ntotal+2*ne));

    readCSV(filename, S);
    writeMatricesToCSV(S.A,S.B,S.C);

    cout<<"State Matricies Constructed\n";
}