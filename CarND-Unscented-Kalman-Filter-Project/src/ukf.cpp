#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);
  x_.fill(0.0);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1;

  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

  /**
  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  P_ << 1,0,0,0,0,
        0,1,0,0,0,
        0,0,1,0,0,
        0,0,0,1,0,
        0,0,0,0,1;

  n_x_ = 5;
  n_aug_ = 7;
  n_radar_ = 3;
  n_laser_ = 2;
  lambda_ = 3 - n_aug_;

  weights_ = VectorXd(2*n_aug_+1);

  // Calculate the weights
  weights_(0) = lambda_ / (lambda_+n_aug_);
  for(int i = 1; i < 2 * n_aug_ + 1; i++)
  {
      weights_(i) = 0.5 / (lambda_+n_aug_);
  }
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  bool isLaser = use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER;
  bool isRadar = use_radar_ && meas_package.sensor_type_ ==  MeasurementPackage::RADAR;

  if(!is_initialized_)
  {
    if(isLaser)
    {
      x_[0] = meas_package.raw_measurements_[0];
      x_[1] = meas_package.raw_measurements_[1];
    }
    else if(isRadar)
    {
        double rho = meas_package.raw_measurements_[0];
        double phi = meas_package.raw_measurements_[1];
        x_[0] = rho * cos(phi);
        x_[1] = rho * sin(phi);
    }

    time_us_ =  meas_package.timestamp_;
    is_initialized_ = isLaser || isRadar;
    // cout << "UKF Initialization Done: " <<  endl;
    // cout << x_ << endl;

    return;
  }

  // Predict
  double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;;
  time_us_ = meas_package.timestamp_;

  Prediction(delta_t);
  cout << "Prediction Done: " <<  endl;
  cout << x_ << endl;

  //Update
  if(isLaser)
  {
      UpdateLidar(meas_package);
      cout << "Lidar update done: " <<  endl;
  }
  else if(isRadar)
  {
      UpdateRadar(meas_package);
      cout << "Radar update done: " <<  endl;
  }

  cout << x_ << endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
   Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  // Generate augmented sigma points
  VectorXd x_aug_ = VectorXd(n_aug_);
  x_aug_.fill(0.0);

  MatrixXd P_aug_ = MatrixXd(n_aug_, n_aug_);
  P_aug_.fill(0.0);

  MatrixXd xAugSigPts = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  // x_aug_ and P_aug_
  x_aug_.head(n_x_) = x_;
  P_aug_.topLeftCorner(n_x_, n_x_) = P_;
  P_aug_(5,5) = std_a_ * std_a_;
  P_aug_(6,6) = std_yawdd_ * std_yawdd_;

  // Square root of P_aug_
  MatrixXd A = P_aug_.llt().matrixL();

  xAugSigPts.col(0) = x_aug_;

  for(int i = 0; i < n_aug_; i++)
  {
      xAugSigPts.col(i+1)     = x_aug_ + sqrt(lambda_+n_aug_) * A.col(i);
      xAugSigPts.col(i+1+n_aug_) = x_aug_ - sqrt(lambda_+n_aug_) * A.col(i);
  }

  // Predict sigma points

  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);

  for(int i = 0; i < 2*n_aug_+1; i++)
  {
      double p_x =  xAugSigPts(0,i);
      double p_y = xAugSigPts(1,i);
      double v = xAugSigPts(2,i);
      double yaw = xAugSigPts(3,i);
      double yawd = xAugSigPts(4,i);
      double nu_a = xAugSigPts(5,i);
      double nu_yawdd = xAugSigPts(6,i);

      double pxP, pyP;

      if (fabs(yawd) > 0.001)
      {
          pxP = p_x + (v/yawd) * (sin(yaw+yawd*delta_t) - sin(yaw) );
          pyP = p_y + (v/yawd) * (cos(yaw) -cos(yaw+yawd*delta_t));
      }
      else
      {
          pxP = p_x + v * cos(yaw) * delta_t;
          pyP = p_y + v * sin(yaw) * delta_t;
      }
      pxP = pxP + 0.5 * (delta_t * delta_t) * cos(yaw) * nu_a;
      pyP = pyP + 0.5 * (delta_t * delta_t) * sin(yaw) * nu_a;

      double vP = v + 0 +  nu_a * delta_t;
      double yawP = yaw + yawd * delta_t + nu_yawdd * (delta_t * delta_t) * 0.5;
      double yawdP = yawd + 0 + nu_yawdd * delta_t;

      Xsig_pred_(0,i) = pxP;
      Xsig_pred_(1,i) = pyP;
      Xsig_pred_(2,i) = vP;
      Xsig_pred_(3,i) = yawP;
      Xsig_pred_(4,i) = yawdP;
  }

  // Predict State (mean)
  x_.fill(0.0);
  for(int i = 0; i < 2*n_aug_+1 ; i++)
  {
      x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  // Predict covariance matrix
  P_.fill(0.0);
  for(int i = 0 ; i < 2*n_aug_+1; i++)
  {
      VectorXd xDiff = Xsig_pred_.col(i) - x_;

      xDiff(3) = normalizeAngles(xDiff(3));

      P_ = P_ + weights_(i) * xDiff * xDiff.transpose();
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

  VectorXd z = VectorXd(n_laser_);
  z(0) = meas_package.raw_measurements_[0];
  z(1) = meas_package.raw_measurements_[1];

  MatrixXd zSig = MatrixXd(n_laser_, 2 * n_aug_+1);

  for(int i = 0; i < 2 * n_aug_ + 1; i++)
  {
      double pX = Xsig_pred_(0,i);
      double pY = Xsig_pred_(1,i);

      zSig(0,i) = pX;
      zSig(1,i) = pY;
  }

  VectorXd zPred = VectorXd(n_laser_);
  zPred.fill(0.0);
  for(int i = 0; i < 2 * n_aug_+1; i++)
  {
      zPred = zPred + weights_(i) * zSig.col(i);
  }

  MatrixXd S = MatrixXd(n_laser_, n_laser_);
  S.fill(0.0);

  for(int i = 0; i < 2 * n_aug_ + 1; i++)
  {
      VectorXd zDiff = zSig.col(i) - zPred;
      S = S + weights_(i) * zDiff * zDiff.transpose();
  }

  MatrixXd R = MatrixXd(n_laser_, n_laser_);

  R << std_laspx_ * std_laspx_, 0,
        0, std_laspy_ * std_laspy_;

  S = S + R;

  MatrixXd T = MatrixXd(n_x_, n_laser_);
  T.fill(0.0);

  for(int i = 0; i < 2 * n_aug_+1; i++)
  {
      VectorXd zDiff = zSig.col(i) - zPred;
      VectorXd xDiff = Xsig_pred_.col(i) - x_;
      xDiff(3) = normalizeAngles(xDiff(3));
      T = T + weights_(i) * xDiff * zDiff.transpose();
  }

  MatrixXd K = T * S.inverse();
  VectorXd residual = z - zPred;

  x_ = x_ + K * residual;
  P_ = P_ - K * S * K.transpose();

  double nisLidar = (z - zPred).transpose() * S.inverse() * (z - zPred);
  cout << "NIS Lidar" << nisLidar << endl;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  VectorXd z = VectorXd(n_radar_);
  z(0) = meas_package.raw_measurements_[0];
  z(1) = meas_package.raw_measurements_[1];
  z(2) = meas_package.raw_measurements_[2];

  MatrixXd zSig = MatrixXd(n_radar_, 2 * n_aug_ + 1);

  for( int i = 0; i < 2 * n_aug_ + 1; i++ )
  {
      double pX = Xsig_pred_(0,i);
      double pY = Xsig_pred_(1,i);
      double v = Xsig_pred_(2,i);
      double yaw = Xsig_pred_(3,i);

      zSig(0,i) = sqrt(pX * pX + pY * pY);
      zSig(1,i) = atan2(pY, pX);
      zSig(2,i) = ((pX * cos(yaw) * v) + (pY * sin(yaw) * v)) / (sqrt(pX * pX + pY * pY));
  }

  VectorXd zPred = VectorXd(n_radar_);
  zPred.fill(0.0);
  for(int i = 0; i < 2 * n_aug_+1; i++)
  {
      zPred = zPred + weights_(i) * zSig.col(i);
  }

  MatrixXd S = MatrixXd(n_radar_, n_radar_);
  S.fill(0.0);

  for( int i = 0; i <  2 * n_aug_+1; i++)
  {
      VectorXd zDiff = zSig.col(i) - zPred;

      zDiff(1) = normalizeAngles(zDiff(1));

      S = S + weights_(i) * zDiff * zDiff.transpose();
  }

  MatrixXd R = MatrixXd(n_radar_, n_radar_);

  R <<  std_radr_ * std_radr_ , 0, 0,
        0, std_radphi_ * std_radphi_, 0,
        0, 0, std_radrd_ * std_radrd_;

  S = S + R;

  MatrixXd T = MatrixXd(n_x_, n_radar_);
  T.fill(0.0);

  for( int i = 0; i < 2 * n_aug_ + 1 ; i++)
  {
      VectorXd zDiff = zSig.col(i) - zPred;

      zDiff(1) = normalizeAngles(zDiff(1));

      VectorXd xDiff = Xsig_pred_.col(i) - x_;

      xDiff(3) = normalizeAngles(xDiff(3));

      T = T + weights_(i) * xDiff * zDiff.transpose();
  }

  MatrixXd K = T * S.inverse();

  VectorXd residual = z - zPred;

  residual(1) = normalizeAngles(residual(1));

  x_ = x_ + K * residual;
  P_ = P_ - K * S * K.transpose();

  double nisRadar = (z - zPred).transpose() * S.inverse() * (z - zPred);
  cout << "NIS Radar" << nisRadar << endl;
}

double UKF::normalizeAngles(double input)
{
    float min = -M_PI;
	float max = M_PI;

    double output;

    output = min + fmod( (max-min) + fmod( (static_cast <float> (input)-min), (max-min)), (max-min) );
    return output;
}
