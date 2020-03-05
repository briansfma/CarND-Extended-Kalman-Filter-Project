#include "kalman_filter.h"
#include <cmath>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::atan2;

using std::cout;
using std::endl;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

/**
 * Initialize a KalmanFilter object with specific matrices.
 */
void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;

}

/**
 * Execute a prediction step from the KalmanFilter object.
 */
void KalmanFilter::Predict() {

  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

/**
 * Execute a measurement update step from the KalmanFilter object.
 * For use with cartesian-coordinate X/Y data only
 */
void KalmanFilter::Update(const VectorXd &z) {
  VectorXd y = z - (H_ * x_);
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K = P_ * Ht * S.inverse();

  //new estimate
  x_ = x_ + (K * y);
  P_ = (I_ - K * H_) * P_;
}

/**
 * Execute a measurement update step from the KalmanFilter object,
 * using Extended Kalman Filter equations.
 * For use with polar ro/theta/ro* data only
 */
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // Manually calculate h(x)
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  float ro = sqrt(px * px + py * py);
  float theta = atan2(py, px);
  float ro_dot = (px * vx + py * vy) / ro;
  VectorXd h_x = VectorXd(3);
  h_x << ro, theta, ro_dot;

  // Apply h(x) and Jacobian H (Hj) back to the Kalman Filter equations
  VectorXd y = z - h_x;

  // Adjust numerical value of theta in y after subtraction; it may have
  // fallen outside the expected range (-pi, pi)
  if (y(1) > M_PI) { y(1) -= 2 * M_PI; }
  if (y(1) < -M_PI) { y(1) += 2 * M_PI; }

  // Continue calculating new estimate
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K = P_ * Ht * S.inverse();

  x_ = x_ + (K * y);
  P_ = (I_ - K * H_) * P_;
}
