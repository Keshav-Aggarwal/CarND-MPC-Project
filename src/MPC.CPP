#include "MPC.h"

using CppAD::AD;
using namespace std;

class FG_eval {

  public:

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    Eigen::VectorXd K; // Fitted road curve polynomial coefficients

    FG_eval(Eigen::VectorXd Kin) : K(Kin) {}

    void operator()(ADvector& fg, const ADvector& x) {
      // fg a vector containing the cost and all constraints
      // x is a vector containing all states and actuations for N "lookahead" states and actuations.

      //*********************************************************
      //* COST DEFINED HERE
      //*********************************************************

      fg[0] = 0.0;

      for (int i = 0; i < N; ++i) {

        const auto cte = x[initial_cte + i];
        const auto epsi = x[initial_epsi + i];
        const auto v = x[initial_v + i] - VELOCITY_MAX;

        fg[0] += (W_cte * cte * cte + W_epsi * epsi * epsi + W_v * v * v);
      }

      for (int i = 0; i < N - 1; ++i) {

        const auto delta = x[initial_delta + i];
        const auto a = x[initial_a + i];

        fg[0] += (W_delta * delta * delta + W_a * a * a);
      }

      for (int i = 0; i < N - 2; ++i) {

        const auto ddelta = x[initial_delta + i + 1] - x[initial_delta + i];
        const auto da = x[initial_a + i + 1] - x[initial_a + i];

        fg[0] += (W_ddelta * ddelta * ddelta + W_da * da * da);
      }

      //*********************************************************
      //* CONSTRAINTS DEFINED HERE
      //*********************************************************

      // given state does not vary
      fg[initial_px + 1] = x[initial_px];
      fg[initial_py + 1] = x[initial_py];
      fg[initial_psi + 1] = x[initial_psi];
      fg[initial_v + 1] = x[initial_v];
      fg[initial_cte + 1] = x[initial_cte];
      fg[initial_epsi + 1] = x[initial_epsi];

      // constraints based on our kinematic model
      for (int i = 0; i < N - 1; ++i) {

        // where the current state variables of interest are stored
        // stored for readability
        const int curr_px = initial_px + i;
        const int curr_py = initial_py + i;
        const int curr_psi = initial_psi + i;
        const int curr_v = initial_v + i;
        const int curr_cte = initial_cte + i;
        const int curr_epsi = initial_epsi + i;
        const int curr_delta = initial_delta + i;
        const int curr_a = initial_a + i;

        //current state and actuations
        const auto px0 = x[curr_px];
        const auto py0 = x[curr_py];
        const auto psi0 = x[curr_psi];
        const auto v0 = x[curr_v];
        const auto cte0 = x[curr_cte];
        const auto epsi0 = x[curr_epsi];
        const auto delta0 = x[curr_delta];
        const auto a0 = x[curr_a];

        // next state
        const auto px1 = x[curr_px + 1];
        const auto py1 = x[curr_py + 1];
        const auto psi1 = x[curr_psi + 1];
        const auto v1 = x[curr_v + 1];
        const auto cte1 = x[curr_cte + 1];
        const auto epsi1 = x[curr_epsi + 1];

        // desired py and psi
        const auto py_desired = K[3] * px0 * px0 * px0 + K[2] * px0 * px0 + K[1] * px0 + K[0];
        const auto psi_desired = CppAD::atan(3.0 * K[3] * px0 * px0 + 2.0 * K[2] * px0 + K[1]);

        // relationship of current state + actuations and next state
        // based on our kinematic model
        const auto px1_f = px0 + v0 * CppAD::cos(psi0) * dt;
        const auto py1_f = py0 + v0 * CppAD::sin(psi0) * dt;
        const auto psi1_f = psi0 + v0 * (-delta0) / Lf * dt;
        const auto v1_f = v0 + a0 * dt;
        const auto cte1_f = py_desired - py0 + v0 * CppAD::sin(epsi0) * dt;
        const auto epsi1_f = psi0 - psi_desired + v0 * (-delta0) / Lf * dt;

        // store the constraint expression of two consecutive states
        fg[curr_px + 2] = px1 - px1_f;
        fg[curr_py + 2] = py1 - py1_f;
        fg[curr_psi + 2] = psi1 - psi1_f;
        fg[curr_v + 2] = v1 - v1_f;
        fg[curr_cte + 2] = cte1 - cte1_f;
        fg[curr_epsi + 2] = epsi1 - epsi1_f;
      }
    }
};

MPC::MPC() {

  this->x.resize(NX);

  for (int i = 0; i < NX; ++i) {
    this->x[i] = 0.0;
  }

  // SET UPPER AND LOWER LIMITS OF VARIABLES

  this->x_lowerbound.resize(NX);
  this->x_upperbound.resize(NX);

  // all other values large values the computer can handle
  for (int i = 0; i < initial_delta; ++i) {
    this->x_lowerbound[i] = -1.0e10;
    this->x_upperbound[i] = 1.0e10;
  }

  // all actuation inputs (steering, acceleration) should have values between [-1, 1]
  for (int i = initial_delta; i < initial_a; ++i) {
    this->x_lowerbound[i] = -0.75;
    this->x_upperbound[i] = 0.75;
  }

  for (int i = initial_a; i < NX; ++i) {
    this->x_lowerbound[i] = -0.5;
    this->x_upperbound[i] = 1.0;
  }

  this->g_lowerbound.resize(NG);
  this->g_upperbound.resize(NG);

  for (int i = 0; i < NG; ++i) {
    this->g_lowerbound[i] = 0.0;
    this->g_upperbound[i] = 0.0;
  }
}

MPC::~MPC() {}

void MPC::solve(Eigen::VectorXd state, Eigen::VectorXd K) {

  const double px = state[0];
  const double py = state[1];
  const double psi = state[2];
  const double v = state[3];
  const double cte = state[4];
  const double epsi = state[5];

  this->g_lowerbound[initial_px] = px;
  this->g_lowerbound[initial_py] = py;
  this->g_lowerbound[initial_psi] = psi;
  this->g_lowerbound[initial_v] = v;
  this->g_lowerbound[initial_cte] = cte;
  this->g_lowerbound[initial_epsi] = epsi;

  this->g_upperbound[initial_px] = px;
  this->g_upperbound[initial_py] = py;
  this->g_upperbound[initial_psi] = psi;
  this->g_upperbound[initial_v] = v;
  this->g_upperbound[initial_cte] = cte;
  this->g_upperbound[initial_epsi] = epsi;

  this->x[initial_px] = px;
  this->x[initial_py] = py;
  this->x[initial_psi] = psi;
  this->x[initial_v] = v;
  this->x[initial_cte] = cte;
  this->x[initial_epsi] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(K);

  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options,
      x,
      x_lowerbound,
      x_upperbound,
      g_lowerbound,
      g_upperbound,
      fg_eval,
      solution);

  this->steer = solution.x[initial_delta];
  this->throttle = solution.x[initial_a];

  this->future_xs = {};
  this->future_ys = {};

  for (int i = 0; i < N; ++i) {

    const double px = solution.x[initial_px + i];
    const double py = solution.x[initial_py + i];

    this->future_xs.emplace_back(px);
    this->future_ys.emplace_back(py);
  }
}