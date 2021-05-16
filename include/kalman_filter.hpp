#ifndef __KALMAN_FILTER_H__
#define __KALMAN_FILTER_H__
#include "Eigen/Dense"
#include <cmath>
#include <iostream>
using namespace Eigen;
class KalmanFilter {
public:
  /**
   * @brief Construct a new Kalman Filter object
   *
   * @param noise_ax the variance of ax squared.
   * @param noise_ay the variance of ay squared.
   * @param z_cov_x measurement covariance in x.
   * @param z_cov_y measurement covariance in y.
   */
  KalmanFilter(double noise_ax, double noise_ay, double z_cov_x,
               double z_cov_y);
  /**
   * @brief Measurements update function.
   *
   * @param z Measurements vector (2x1).
   * @param timestamp Measurements Timestamp.
   * @param bearing If True we will use EKF and the measurement should be
   * (range,angle).
   * @param us True if the timestamp in microseconds, false if it's in
   * nanoseconds.
   */
  void Update(const VectorXd &z, long timestamp, bool bearing = false,
              bool us = false);

  /**
   * @brief Get the states vector.
   *
   * @return const VectorXd
   */
  const VectorXd GetX() const { return x_; }

private:
  VectorXd x_;
  MatrixXd F_;
  MatrixXd Q_;
  MatrixXd P_;
  MatrixXd H_;
  MatrixXd R_;
  MatrixXd I_;
  double noise_ax_;
  double noise_ay_;
  long previous_ts_ = 0;
  bool initialized_ = false;
  void Predict_();
};

#endif // __KALMAN_FILTER_H__