#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

const int PORT = 4567;

// for convenience
using json = nlohmann::json;
using namespace std;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Fit a polynomial.
// Adapted from https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); ++i) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); ++j) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {

          std::vector<double> points_xs = j[1]["ptsx"];
          std::vector<double> points_ys = j[1]["ptsy"];

          const double px = j[1]["x"];
          const double py = j[1]["y"];
          const double psi = j[1]["psi"];
          const double v = j[1]["speed"];
          const double steer = j[1]["steering_angle"];
          const double a = j[1]["throttle"];

          const int num_points = points_xs.size();
          Eigen::VectorXd wp_xs(num_points);
          Eigen::VectorXd wp_ys(num_points);

          for(int i = 0; i < num_points; ++i) {

            const double dx = points_xs[i] - px;
            const double dy = points_ys[i] - py;

            wp_xs[i] = dx * cos(-psi) - dy * sin(-psi);
            wp_ys[i] = dy * cos(-psi) + dx * sin(-psi);
          }

          std::vector<double> next_xs(N);
          std::vector<double> next_ys(N);

          const int poly_order = 3;
          auto K = polyfit(wp_xs, wp_ys, poly_order);

          
          const double D = 5.0;

          for (int i = 0; i < N; ++i) {

            const double dx = D * i;
            const double dy = K[3] * dx * dx * dx + K[2] * dx * dx + K[1] * dx + K[0];

            next_xs[i] = dx;
            next_ys[i] = dy;
          }

          const double cte = K[0];
          const double epsi = -atan(K[1]);

          const double dt = 0.1;
          const double Lf = 2.67;

          const double c_px = 0.0 + v * dt;
          const double c_py = 0.0;
          const double c_psi = 0.0 + v * (-steer) / Lf * dt;
          const double c_v = v + a * dt;
          const double c_cte = cte + v * sin(epsi) * dt;
          const double c_epsi = epsi + v * (-steer) / Lf * dt;

          const int total_states = 6;
          Eigen::VectorXd state(total_states);
          state << c_px, c_py, c_psi, c_v, c_cte, c_epsi;

          mpc.solve(state, K);
          
          json msgJson;
          msgJson["steering_angle"] = mpc.steer;
          msgJson["throttle"] = mpc.throttle;

          msgJson["mpc_x"] = mpc.future_xs;
          msgJson["mpc_y"] = mpc.future_ys;

          msgJson["next_x"] = next_xs;
          msgJson["next_y"] = next_ys;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          
          // Latency
          this_thread::sleep_for(chrono::milliseconds(100));

          // sent do simulator
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  if (h.listen(PORT)) {
    std::cout << "Listening to port " << PORT << std::endl;
  } else {
    std::cerr << "Failed to listen to port " << PORT << std::endl;
    return -1;
  }
  h.run();
}