#include "kalman_filter.h"
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
	MatrixXd y = z - H_ * x_;
	MatrixXd H_t = H_.transpose();
	MatrixXd S = H_ * P_ * H_t + R_;
	MatrixXd SInv = S.inverse();
	MatrixXd K = P_ * H_t * SInv;

	x_ = x_ + K * y;
	long xSize = x_.size();
	MatrixXd I = MatrixXd::Identity(xSize, xSize);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
	/**
  TODO:
	 * update the state by using Extended Kalman Filter equations
	 */

	VectorXd Hx = VectorXd(3);

	float px = x_(0);
	float py = x_(1);
	float vx = x_(2);
	float vy = x_(3);

	float c1 = px*px+py*py;
	float c2 = sqrt(c1);

	Hx(0) = c2;
	float phi = atan2(py,px);

	float min = -M_PI;
	float max = M_PI;

	Hx(1) = phi;
	Hx(2) = (px * vx + py * vy)/c2;

	VectorXd y = z - Hx;

	y[1] = min + fmod( (max-min) + fmod( (static_cast <float> (y[1])-min), (max-min)), (max-min) );
	MatrixXd H_t = H_.transpose();
	MatrixXd S = H_ * P_ * H_t + R_;
	MatrixXd SInv = S.inverse();
	MatrixXd K = P_ * H_t * SInv;

	x_ = x_ + K * y;
	long xSize = x_.size();
	MatrixXd I = MatrixXd::Identity(xSize, xSize);
	P_ = (I - K * H_) * P_;
}
