#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in)
{
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict()
{
  /**
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z)
{
  /**
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
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z)
{
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  static float last_z_pred_2 = 0;
  float px, py, vx, vy;
  px = x_[0];
  py = x_[1];
  vx = x_[2];
  vy = x_[3];

  VectorXd z_pred(3);
  z_pred[0] = sqrt(px * px + py * py);
  if (abs(px) < 1e-6)
  { // Near-0 X, snap to PI/2 depending on y sign
    if (py > 0)
    {
      z_pred[1] = M_PI / 2;
    }
    else
    {
      z_pred[1] = -M_PI / 2;
    }
  }
  else
  {
    z_pred[1] = atan2(py, px);
  }
  if (px * px + py * py > 1e-6)
  {
    z_pred[2] = (px * vx + py * vy) / sqrt(px * px + py * py);
    last_z_pred_2 = z_pred[2];
  }
  else
  { // Use previous value
    z_pred[2] = last_z_pred_2;
  }

  VectorXd y = z - z_pred;

  // Correct angles
  while (y[1] > M_PI)
  {
    y[1] -= 2 * M_PI;
  }

  while (y[1] < -M_PI)
  {
    y[1] += 2 * M_PI;
  }

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
