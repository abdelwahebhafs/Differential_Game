#include "Global.h"

#include <eigen3/Eigen/Dense>

using namespace Eigen;

const int m = 1;
const int n = 2 * m; 
const double time_step = 0.001;
const double task_duration = 1.01;
double current_time = 0; // Renamed to avoid conflict with <chrono>
const double integration_time_step = 0.001;
const double planning_horizon = 1;
const double estimation_horizon = 1;
const double integration_step = 0.001;
int ne = estimation_horizon/time_step;
int np = planning_horizon/time_step;
int nc = -1;
int ntotal = task_duration/time_step;

