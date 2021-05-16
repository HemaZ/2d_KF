# 2D Position and Velocity Kalman Filter

![screenshot](screenshot.svg)
 2D Kalman filter for position and velocity estimation.

 ## How to use

```c++
/**
 * @brief Construct a new Kalman Filter object
 *
 * @param noise_ax the variance of ax squared.
 * @param noise_ay the variance of ay squared.
 * @param z_cov_x measurement covariance in x.
 * @param z_cov_y measurement covariance in y.
 */
KalmanFilter kf(3, 3, 0.2, 0.2);
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
kf.Update(z, timestamp, false, true);
/**
* @brief Get the states vector.
*
* @return const VectorXd
*/
auto x = kf.GetX();

 ```
