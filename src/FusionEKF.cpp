#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;
using std::sin;
using std::cos;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  // Hj_ = MatrixXd(3, 4);			// no longer needed

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  // measurement matrix - laser
  H_laser_ << 1, 0, 0, 0,
          	  0, 1, 0, 0;

  // measurement matrix - radar
  // Hj = we will calculate per update of the Extended Kalman Filter

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    // Initialize Kalman Filter matrices

	// the measurement matrix (H) will not be initialized here, as we'll need
	// to switch between H_laser and the Jacobian depending on measurement type.

	// measurement covariance (R) will not be initialized here, as we'll need
	// to switch between R_laser and R_radar depending on measurement type.

    // initialize the state covariance matrix (P)
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
			   0, 1, 0, 0,
			   0, 0, 1000, 0,
			   0, 0, 0, 1000;

    // initialize the transition matrix (F) and process covariance matrix (Q)
    // as well. we will update it with each measurement when we have dt values.
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ << 1, 0, 1, 0,
			   0, 1, 0, 1,
			   0, 0, 1, 0,
			   0, 0, 0, 1;

    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ << 0, 0, 0, 0,
			   0, 0, 0, 0,
			   0, 0, 0, 0,
			   0, 0, 0, 0;

    // initialize Identity matrix now, it is static so this will be faster
    // than doing it each time we call Update or UpdateEKF
    ekf_.I_ = MatrixXd(4, 4);
    ekf_.I_ << 1, 0, 0, 0,
			   0, 1, 0, 0,
			   0, 0, 1, 0,
			   0, 0, 0, 1;

    // initialize the measurement vector
    ekf_.x_ = VectorXd(4);

    // first measurement
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // convert radar from polar to cartesian coordinates
      float ro = measurement_pack.raw_measurements_(0);
      float theta = measurement_pack.raw_measurements_(1);
      float ro_dot = measurement_pack.raw_measurements_(2);

      // initialize state. we have no info on tangential velocity, so
      // vx and vy initially will be zero.
      ekf_.x_(0) = ro * cos(theta);							// px
      ekf_.x_(1) = ro * sin(theta);							// py
      ekf_.x_(2) = 0;										// vx
      ekf_.x_(3) = 0;										// vy
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // initialize state. we have no info on velocity, period, so
      // vx and vy initially will be zero.
      ekf_.x_(0) = measurement_pack.raw_measurements_(0);	// px
      ekf_.x_(1) = measurement_pack.raw_measurements_(1);	// py
      ekf_.x_(2) = 0;										// vx
      ekf_.x_(3) = 0;										// vy
    }

    // Update "previous timestamp" to be able to calculate next dt
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    cout << "EKF: " << endl;

    return;
  }

  /**
   * Prediction
   */
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1e6;

  // Update the F matrix with dt as needed
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;

  // Update the process covariance matrix Q as needed
  // noise_ax = 9 and noise_ay = 9
  float dt_sq = dt * dt;
  float dt_cu = dt_sq * dt;
  float dt_qu = dt_cu * dt;
  float noise_ax = 9;
  float noise_ay = 9;
  ekf_.Q_(0,0) = dt_qu/4 * noise_ax;
  ekf_.Q_(0,2) = dt_cu/2 * noise_ax;
  ekf_.Q_(1,1) = dt_qu/4 * noise_ay;
  ekf_.Q_(1,3) = dt_cu/2 * noise_ay;
  ekf_.Q_(2,0) = dt_cu/2 * noise_ax;
  ekf_.Q_(2,2) = dt_sq * noise_ax;
  ekf_.Q_(3,1) = dt_cu/2 * noise_ay;
  ekf_.Q_(3,3) = dt_sq * noise_ay;

  // Run the prediction equations
  ekf_.Predict();

  /**
   * Update
   */
  // Check measurement type before update (Radar will need polar
  // coordinate conversion, Laser just uses cartesian X/Y)
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
	// Calculate Jacobian matrix given current state
	ekf_.H_ = tools.CalculateJacobian(ekf_.x_);

	// Set measurement noise, then update x_ and P_
	ekf_.R_ = R_radar_;
	ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
	// Set measurement matrix
	ekf_.H_ = H_laser_;

	// Set measurement noise, then update x_ and P_
	ekf_.R_ = R_laser_;
	ekf_.Update(measurement_pack.raw_measurements_);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;

  // Update "previous timestamp" to be able to calculate next dt
  previous_timestamp_ = measurement_pack.timestamp_;
}
