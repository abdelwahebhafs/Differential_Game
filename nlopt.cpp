#include <iomanip>
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <cmath>

#include "../nlopt-2.7.1/build/nlopt.hpp"

// Define your objective function here
double myfunc(const std::vector<double>& x, std::vector<double>& grad, void* my_func_data) {
  
  return (x[1]-M_PI)*(x[1]-M_PI)+x[0]*x[0] + 1;
}

// Define your constraint function here
struct my_constraint_data {
  double a;
  double b;
};

double myconstraint(const std::vector<double>& x, std::vector<double>& grad, void* data) {
  // Implement your constraint logic here
  // This example defines a constraint based on data structure
  my_constraint_data* d = reinterpret_cast<my_constraint_data*>(data);
  double a = d->a;
  double b = d->b;
 
  return pow(a * x[0] + b, 3) - x[1]; // Constraint function based on data
}

int main() {
  // Create NLopt object and set parameters
  nlopt::opt opt(nlopt::LN_BOBYQA, 2);
  std::vector<double> lb(2);
  lb[0] = -HUGE_VAL;
  lb[1] = 0;
  opt.set_lower_bounds(lb);
  opt.set_min_objective(myfunc, nullptr); // Set user-defined objective function

  // Define constraint data
  my_constraint_data data[2] = {{2, 0}, {-1, 1}};


  opt.set_xtol_rel(1e-4); // Set tolerance for solution

  // Initial guess for variables
  std::vector<double> x(2);
  x[0] = 1.234;
  x[1] = 5.678;
  double minf;

  try {
    nlopt::result result = opt.optimize(x, minf);
    std::cout << "found minimum at f(" << x[0] << "," << x[1] << ") = "
              << std::setprecision(10) << minf << std::endl;
  } catch (std::exception& e) {
    std::cout << "nlopt failed: " << e.what() << std::endl;
  }

  return 0;
}


/////// To Run  g++ nlopt.cpp -o out -lnlopt -lm