#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * predict the state
   */
   x_ = F_ * x_;
   MatrixXd Ft = F_.transpose();
   P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */

   VectorXd z_pred = H_ * x_;
   VectorXd y = z - z_pred;
   UpdateEqns(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * Update the state by using Extended Kalman Filter equations
   */
   float px = x_(0);
   float py = x_(1);
   float vx = x_(2);
   float vy = x_(3);
  
   // Convert Prediction to polar
   double rho = sqrt(px*px+py*py);
   double theta = atan2(py, px);

   // Need to protect again div/0
   if (rho < .00001){
     px += 0.001;
     py += 0.001;
     rho = sqrt(px*px+py*py);
   }
   double rho_dot = (px*vx+py*vy) / rho;

   VectorXd h = VectorXd(3);
   h << rho, theta, rho_dot;

   VectorXd y = z - h;

   // Normalise angle
   for (; y(1) < -M_PI; y(1) += 2*M_PI) {}
   for (; y(1) > M_PI;  y(1) -= 2*M_PI) {}

   UpdateEqns(y);
}

void KalmanFilter::UpdateEqns(const VectorXd &y) {
  // Standard updates for KF and EKF
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
