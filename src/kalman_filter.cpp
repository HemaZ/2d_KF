#include "kalman_filter.hpp"
KalmanFilter::KalmanFilter(double noise_ax, double noise_ay, double z_cov_x, double z_cov_y)
    : x_(4),
      F_(4, 4),
      Q_(4, 4),
      P_(4, 4),
      H_(2, 4),
      R_(2, 2),
      I_(MatrixXd::Identity(4, 4)),
      noise_ax_(noise_ax),
      noise_ay_(noise_ay) {
  x_.setZero();
  F_.setIdentity();
  // initialize covariance matrix P we are more confident about location than
  // velocity.
  P_.setIdentity();
  P_(2, 2) = 1000;
  P_(3, 3) = 1000;

  Q_.setZero();

  // measurement covariance
  R_ << z_cov_x, 0, 0, z_cov_y;

  // measurement matrix Z_pred = H*X
  H_ << 1, 0, 0, 0, 0, 1, 0, 0;
}

void KalmanFilter::Predict_() {
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z, long timestamp, bool bearing, bool us) {
  if (!initialized_) {
    x_ << z[0], z[1], 0, 0;
    initialized_ = true;
    previous_ts_ = timestamp;
    return;
  }
  double dt = (timestamp - previous_ts_);
  if (us)
    dt *= pow(10, -6);
  else
    dt *= pow(10, -9);
  // Update F
  F_(0, 2) = dt;
  F_(1, 3) = dt;
  // Update Q
  double dt2 = pow(dt, 2);
  double dt3 = pow(dt, 3);
  double dt4 = pow(dt, 4);
  Q_ << (noise_ax_ / 4) * dt4, 0, (noise_ax_ / 2) * dt3, 0, 0, (noise_ay_ / 4) * dt4, 0,
      (noise_ay_ / 2) * dt3, (noise_ax_ / 2) * dt3, 0, noise_ax_ * dt2, 0, 0, noise_ay_ * dt3 / 2,
      0, noise_ay_ * dt2;
  // Call The Predict function
  Predict_();
  // Update using the new measurements
  VectorXd z_pred(2, 1);
  if (bearing) {
    double c1 = x_(0) * x_(0) + x_(1) * x_(1);
    double c2 = sqrt(c1);
    if (fabs(c1) < 0.0001) {
      std::cout << "Calculating Jacobian - Error - Division by Zero" << std::endl;
      return;
    }
    H_ << x_(0) / c2, x_(1) / c2, 0, 0, -x_(1) / c1, x_(0) / c1, 0, 0;  // Jacobian Matrix
    z_pred << c2, std::atan2(x_(1), x_(0));  // h(x)-> (sqrt(x^2+y^2), atan(y,x))
  } else {
    z_pred = H_ * x_;
  }
  VectorXd y = z - z_pred;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  x_ = x_ + (K * y);
  P_ = (I_ - K * H_) * P_;
  previous_ts_ = timestamp;
}