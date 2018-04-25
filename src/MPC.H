#ifndef MPC_H
#define MPC_H

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include <vector>

typedef CPPAD_TESTVECTOR(double) Dvector;

const int N = 10; 
const double dt = 0.1; 

const double Lf = 2.67; // this is the length from front of vehicle to Center-of-Gravity
const double VELOCITY_MAX = 50.0; 

const int num_state = 6; 
const int num_actuations = 2; 

const int NX =  N * num_state + (N - 1) * num_actuations; 
const int NG = N * num_state;

// Cost variables
const double W_cte = 1500.0;
const double W_epsi = 1500.0;
const double W_v = 1.0;
const double W_delta = 10.0;
const double W_a = 10.0;
const double W_ddelta = 150.0;
const double W_da = 15.0; 

// where the first element of each state variable
const int initial_px = 0;
const int initial_py = initial_px + N;
const int initial_psi = initial_py + N;
const int initial_v = initial_psi + N;
const int initial_cte = initial_v + N;
const int initial_epsi = initial_cte + N;
const int initial_delta = initial_epsi + N;
const int initial_a = initial_delta + N - 1;


class MPC {

 public:

  double steer;
  double throttle;

  Dvector x; 
  Dvector x_lowerbound; //lower limit for corresponding variable in x
  Dvector x_upperbound; //upper limit for corresponding variable in x
  Dvector g_lowerbound; // value constraint for corresponding constraint expression
  Dvector g_upperbound; // value constraint for corresponding constraint expression

  std::vector<double> future_xs;
  std::vector<double> future_ys;

  MPC();
  virtual ~MPC();

  // Solve the model given the current state and curve coefficients
  void solve(Eigen::VectorXd state, Eigen::VectorXd K);
};

#endif /* MPC_H */