#include "kalman_filter.h"

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

	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {

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

	float px = x_(0);
	float py = x_(1);
	float vx = x_(2);
	float vy = x_(3);
	
	float rho = sqrt(pow(px, 2) + pow(py, 2));
	float phi = atan2(py, px);
	float dot_rho;
	if (rho >=0.0001)
		dot_rho = (px*vx + py*vy) / rho;
	else
		dot_rho = (px*vx + py*vy) / 0.0001;

	VectorXd radar_hx = VectorXd(3);
	radar_hx << rho, phi, dot_rho;

	VectorXd y = z - radar_hx;
	
	// the resulting angle phi in the y vector should be adjusted so that it is between -pi and pi
	while (y(1) < -M_PI || y(1) > M_PI) {
		if (y(1) < -M_PI) y(1) += 2 * M_PI;
		else y(1) -= 2 * M_PI;
	}

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
