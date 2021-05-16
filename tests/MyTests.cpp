// This tells Catch to provide a main() - only do this in one cpp file
#define CATCH_CONFIG_MAIN
#include <iostream>
#include <sciplot/sciplot.hpp>
#include "catch.hpp"
#include "kalman_filter.hpp"
using namespace sciplot;
#include <chrono>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <thread>
#include <type_traits>
using namespace std::chrono;
#include <vector>
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::ifstream;
using std::istringstream;
using std::string;
using std::vector;

double RMSE(const vector<double>& x1, const vector<double>& y1, const vector<double>& x2,
            const vector<double>& y2) {
  assert(x1.size() == x2.size());
  assert(y1.size() == y2.size());
  assert(x1.size() == y2.size());
  double sum = 0;
  for (size_t i = 0; i < x1.size(); i++) {
    double c1 = (x1[i] - x2[i]) * (x1[i] - x2[i]);
    double c2 = (y1[i] - y2[i]) * (y1[i] - y2[i]);
    sum += sqrt(c1 + c2);
  }
  return sum / x1.size();
}

TEST_CASE("KalmanFilter", "StraightLine") {
  KalmanFilter kf(0.1, 0.1, 1, 1);
  double x = 0;
  double y = 0;
  double vx = 2;
  double vy = 2;
  VectorXd Z(2, 1);
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<> d(0, 1);

  std::vector<double> xv, yv, xf, yf, xa, ya;

  long t = 0;

  for (size_t i = 0; i < 1000; i++) {
    double dt = 0.1;
    t += dt * pow(10, 9);
    x += (vx)*dt;
    y += (vy)*dt;
    xa.push_back(x);
    ya.push_back(y);
    double x_noised = x + d(gen);
    double y_noised = y + d(gen);
    Z << x_noised, y_noised;
    kf.Update(Z, t);
    // std::cout << t << " Measurement: X: " << x << " Y: " << y << std::endl;
    // std::cout << t << " Filtered: X: " << kf.GetX()(0) << " Y: " << kf.GetX()(1)
    //           << " Vx: " << kf.GetX()(2) << std::endl;
    // std::cout << "dt: " << dt << std::endl;
    xv.push_back(x_noised);
    yv.push_back(y_noised);
    xf.push_back(kf.GetX()(0));
    yf.push_back(kf.GetX()(1));
  }
  std::cout << "RMSE: " << RMSE(xa, ya, xf, yf) << std::endl;
  REQUIRE(RMSE(xa, ya, xf, yf) < 1);
  // Create a Plot object
  Plot plot;
  // Set color palette
  plot.palette("set1");
  plot.size(800, 400);
  plot.drawCurve(xa, ya).label("actual").lineWidth(2);
  // Draw a sine graph putting x on the x-axis and sin(x) on the y-axis
  plot.drawDots(xv, yv).label("measured").lineWidth(4);
  // Draw a cosine graph putting x on the x-axis and cos(x) on the y-axis
  plot.drawDots(xf, yf).label("filtered").lineWidth(4);
  // Show the plot in a pop-up window
  plot.show();
  // Save the plot to a PDF file
  plot.save("straight_line.pdf");
}

TEST_CASE("KalmanFilter2", "8Shape") {
  KalmanFilter kf(3, 3, 0.2, 0.2);
  std::vector<double> xf, yf, xa, ya;

  // hardcoded input file with laser and radar measurements
  string in_file_name_ = "../obj_pose-laser-radar-synthetic-input.txt";
  ifstream in_file(in_file_name_.c_str(), ifstream::in);

  if (!in_file.is_open()) {
    cout << "Cannot open input file: " << in_file_name_ << endl;
  }

  string line;
  // set i to get only first 3 measurments
  int i = 0;
  while (getline(in_file, line) && (i <= 250)) {
    istringstream iss(line);
    string sensor_type;
    iss >> sensor_type;  // reads first element from the current line
    int64_t timestamp;
    if (sensor_type.compare("L") == 0) {  // laser measurement
      // read measurements
      VectorXd z = VectorXd(2);
      double x;
      double y;
      iss >> x;
      iss >> y;
      z << x, y;
      xa.push_back(x);
      ya.push_back(y);
      iss >> timestamp;

      kf.Update(z, timestamp, false, true);
      xf.push_back(kf.GetX()(0));
      yf.push_back(kf.GetX()(1));
      // std::cout << "X: " << kf.GetX()(0) << " Y: " << kf.GetX()(1) << std::endl;

    } else if (sensor_type.compare("R") == 0) {
      // Skip Radar measurements
      continue;
    }
    ++i;
  }

  if (in_file.is_open()) {
    in_file.close();
  }
  std::cout << "RMSE: " << RMSE(xa, ya, xf, yf) << std::endl;
  REQUIRE(RMSE(xa, ya, xf, yf) < 1);
  // Create a Plot object
  Plot plot;
  // Set color palette
  plot.palette("set2");
  plot.size(800, 400);
  plot.drawDots(xa, ya).label("actual");
  // Draw a cosine graph putting x on the x-axis and cos(x) on the y-axis
  plot.drawDots(xf, yf).label("filtered");
  // Show the plot in a pop-up window
  plot.show();
  // Save the plot to a PDF file
  plot.save("8shape.pdf");
}