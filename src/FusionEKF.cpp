#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

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
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */

   // Laser measurement - only X and Y, no velocity
   H_laser_ << 1, 0, 0, 0,
               0, 1, 0, 0;

   // F - next state function
   MatrixXd F = MatrixXd(4,4);

   F << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;

   // X - initial state and location (4D - 2 position, 2 velocity)
   VectorXd X = VectorXd(4);

   X << 0, 0, 0, 0;

   // H_Jacobian
   //Hj_ = CalculateJacobian(X);

   // P - initial uncertainty
   MatrixXd P = MatrixXd(4,4);

   P << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

   // u - external motion (assume zero)
   MatrixXd u = MatrixXd(1,2);
   u << 0, 0;

   // Empty matrices to initialise EKF objects
   MatrixXd Q = MatrixXd::Zero(4, 4);

   // Initialise EFK object
   ekf_.Init(X, P, F, H_laser_, R_laser_, Q);
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
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates
      //         and initialise state.
      float x = measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]);
      float y = measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]);
      float vx = measurement_pack.raw_measurements_[2] * cos(measurement_pack.raw_measurements_[1]);
      float vy = measurement_pack.raw_measurements_[2] * sin(measurement_pack.raw_measurements_[1]);

      ekf_.x_ << x, y, vx, vy;

      // Speed is probably quite accurate
      ekf_.P_ << 1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 1, 0,
                 0, 0, 0, 1;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      ekf_.x_ << measurement_pack.raw_measurements_[0],
                 measurement_pack.raw_measurements_[1],
                 0,
                 0;

       // Speed is probably less accurate
       ekf_.P_ << 1, 0, 0, 0,
                  0, 1, 0, 0,
                  0, 0, 1000, 0,
                  0, 0, 0, 1000;
    }

    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
   // Get time step
   float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
   previous_timestamp_ = measurement_pack.timestamp_;

   //Update F
   ekf_.F_(0,2) = dt;
   ekf_.F_(1,3) = dt;
   //ekf_.F_ << 1, 0, dt, 0,
  //            0, 1, 0,  dt,
  //            0, 0, 1,  0,
  //            0, 0, 0,  1;


   // Q
   double n_ax = 9;
   double n_ay = 9;

   ekf_.Q_ << (n_ax*n_ax)*(dt*dt*dt*dt)/4, 0, (n_ax*n_ax)*((dt*dt*dt)/2),  0,
              0, (n_ay*n_ay)*((dt*dt*dt*dt)/4), 0, (n_ay*n_ay)*((dt*dt*dt)/2),
              (n_ax*n_ax)*((dt*dt*dt)/2), 0, (dt*dt)*(n_ax*n_ax), 0,
              0, ((dt*dt*dt)/2)*(n_ay*n_ay), 0, (dt*dt)*(n_ay*n_ay);


  ekf_.Predict();

  /**
   * Update
   */

  /**
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates

    //calculate Jacobian
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    // Extended KF update
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    // Normal KF update
    ekf_.Update(measurement_pack.raw_measurements_);
  }


  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
