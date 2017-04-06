#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in)
{
    x_ = x_in;
    P_ = P_in;
    stateIdentity_ = MatrixXd::Identity(x_.size(), x_.size());
}

void KalmanFilter::Predict(const MatrixXd &F, const MatrixXd &Q)
{
    // predict the new mean
    x_ = F * x_;
    
    // predict the new covariance
    // Q is the process noise - It represents the idea/feature that the state of the system changes over time but
    // we do not know the exact details of when/how those changes occur, and thus we need to model them as a random
    // process. In this case, acceleration of the object being tracked is inluded in this.
    P_ = F * P_ * F.transpose() + Q;
}

void KalmanFilter::Update(const VectorXd &z, const MatrixXd &H, const MatrixXd &Ht, const MatrixXd &R)
{
    // calculate the Kalman gain
    MatrixXd S = H * P_ * Ht + R;
    MatrixXd K = P_ * Ht * S.inverse();

    // project the current prediction into the measurement space, calculate the error term and use it to update the state
    VectorXd y = z - H * x_;
    x_ = x_ + (K * y);
    
    // update the state covariance
    P_ = (stateIdentity_ - K * H) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z, MatrixXd (*HjoX)(const VectorXd& in), VectorXd (*HoX)(const VectorXd &in), const MatrixXd &R)
{
    // calculate the Kalman gain using the Jacobian provided by HjoX
    MatrixXd Hj = HjoX(x_);
    MatrixXd Hjt = Hj.transpose();
    MatrixXd S = Hj * P_ * Hjt + R;
    MatrixXd K = P_ * Hjt * S.inverse();
    
    // project the current prediction into the measurement space, calculate the error term and use it to update the state
    VectorXd y = z - HoX(x_);
    x_ = x_ + K * y;
    
    // update the state covariance
    P_ = (stateIdentity_ - K * Hj) * P_;
}
