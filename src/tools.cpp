#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

/**
 * Calculate the root-mean-square error between filter estimates and
 * ground-truth data.
 */
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  if (!estimations.size()) {
    cout << "CalculateRMSE () - Error - Estimations vector size 0" << endl;
  }
  //  * the estimation vector size should equal ground truth vector size
  else if (estimations.size() != ground_truth.size()) {
    cout << "CalculateRMSE () - Error - Estimations and Ground_Truth different size"
         << endl;
  }
  // implement RMSE formula
  else {
    // accumulate residuals
    for (int i=0; i < estimations.size(); i++) {
      VectorXd residuals = estimations[i] - ground_truth[i];
      residuals = residuals.array() * residuals.array();
      rmse += residuals;
    }

    // calculate the mean
    rmse = rmse / estimations.size();

    // calculate the squared root
    rmse = rmse.array().sqrt();

  }

  // return the result
  return rmse;
}

/**
 * Calculate the Jacobian matrix for polar-coordinate updates.
 */
MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // TODO: YOUR CODE HERE
  // check division by zero
  float rho_sq = px * px + py * py;
  float rho = sqrt(rho_sq);
  float vxy_pxy = vx*py - vy*px;
  float vyx_pyx = vy*px - vx*py;

  if (rho_sq > 0) {
    // compute the Jacobian matrix
    Hj << (px / rho), (py / rho), 0, 0,
          -(py / rho_sq), (px / rho_sq), 0, 0,
          (py*vxy_pxy)/(rho_sq*rho), (py*vyx_pyx)/(rho_sq*rho), (px / rho), (py / rho);
  }
  else {
      cout << "CalculateJacobian () - Error - Division by Zero" << endl;
  }


  return Hj;
}
