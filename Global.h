#ifndef GLOBAL_H
#define GLOBAL_H

#include <eigen3/Eigen/Dense>

using namespace Eigen;

extern const int m; 
extern const int n; 
extern const double time_step ;
extern const double task_duration ;
extern double current_time ; // Renamed to avoid conflict with <chrono>
extern const double integration_time_step ;
extern const double planning_horizon ;
extern const double estimation_horizon ;
extern const double integration_step;
extern int ne, np, nc, ntotal;

#endif


