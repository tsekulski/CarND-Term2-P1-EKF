#include "kalman_filter.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  TODO:
    * predict the state
  */

	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
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

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

	//prep: calculate h(x)
	VectorXd hx = CalculateHx(x_);

	//EKF measurement update - same equations as above
	VectorXd z_pred = hx;
	VectorXd y = z - z_pred;

	//Normalizing phi
	y(1) = atan2(sin(y(1)),cos(y(1)));

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

VectorXd KalmanFilter::CalculateHx(const VectorXd& x_state) {


	VectorXd Hx(3);
	Hx << 0,0,0;

	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//pre-compute a set of terms
	float c1 = px*px + py*py;

	//check division by zero
	if(c1 < 0.0001){
		//cout << "CalculateHx () - Error - Division by Zero" << endl;
		px += 0.001;
		py += 0.001;
		c1 = px*px + py*py;
	}

	float rho = sqrt(c1);

	float phi = 0.0; // default if px is 0
	if (fabs(px) > 0.0001) {
		phi = atan2(py,px);
	}

	float rho_dot = 0.0; // default if rho is 0
		if (fabs(rho) > 0.0001) {
			rho_dot = (px*vx+py*vy)/rho;
		}

	//update Hx vector
	Hx << rho, phi, rho_dot;

	return Hx;

}
