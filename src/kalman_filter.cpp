#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {

  // Set identity matrix I, for uncertainty calculation
	I = MatrixXd::Identity(4, 4);

}

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
  TODO:
    * predict the state
  */
  
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_; // lesson 5.8

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

    // Lesson 5.7
  	VectorXd y = z - H_ * x_; // error calculation for prediction vs actual
    MatrixXd Ht = H_.transpose();
		MatrixXd S = H_ * P_ *  Ht + R_;
		MatrixXd K =  P_ * Ht *  S.inverse();

		// Set the new state/uncertainty
		x_ = x_ + (K * y);
		P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  // Lesson 5.14, convert cartesian coordinates to polar for radar
  float x = x_[0];
  float y = x_[1];
  float vx = x_[2];
  float vy = x_[3];

  float rho = sqrt(x * x + y * y);
  float phi = atan2(y, x);

  float ro_dot = (x * vx + y * vy) / rho;
  VectorXd z_predict = VectorXd(3);
  z_predict << rho, phi, ro_dot;

  // Error calculation for prediction vs actual 
  // z_predict is the converted cordinates from the prediction step (see above)
  VectorXd y_error = z - z_predict;

  // Normalize phi
  while (y_error[1] > M_PI) {
     y_error[1] -= 2 * M_PI;
  }
  while (y_error[1] < -M_PI) {
    y_error[1] += 2 * M_PI;
  }

  // Lesson 5.7
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  // Set the new state/uncertainty
  x_ = x_ + (K * y_error);
  P_ = (I - K * H_) * P_;

}
