#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

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
  //cout << "x_: " << x_ <<endl;
  MatrixXd Ft = F_.transpose();
  //cout << "Ft: " << Ft <<endl;
  P_ = F_ * P_ * Ft + Q_;
  //cout << "P_: " << P_ <<endl;
  /**
  TODO:
    * predict the state
  */
}

void KalmanFilter::Update(const VectorXd &z) {

  //cout <<"KF: Update function"<<endl;
  //cout << "H_" << H_ << endl;
  //cout << "x_" << x_ << endl;
  //cout << "R_" << R_ << endl;
  VectorXd z_pred = H_ * x_;
  //cout << "z_pred "<<z_pred <<endl;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  //cout <<"R_" << R_<<endl;
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  //cout << "Kalman_filter: K: " << K << endl;

    //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);

  //cout << "Kalman_filter: I: " << I << endl;
  P_ = (I - K * H_) * P_;

  //cout << "Kalman_filter: P: " << P_ << endl;
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

   /* if (x_[0] < epsilon && x_[1] < epsilon) {
      x_[0] = epsilon;
      x_[1] = epsilon;
    } else if (x_[0] < epsilon) {

      x_[0] = epsilon;
    }
    */
    Tools tools;
    VectorXd h_(3);
    float px, px2, py, py2, sps, vx, vy;
    px = x_[0];
    px2 = px * px;
    py = x_[1];
    py2 = py * py;
    sps = pow((px2 + py2), 0.5);
    vx = x_[2];
    vy = x_[3];
    h_ << sps, atan2(py, px), (px * vx + py * vy) / sps;
    //cout <<"h"<<h_<<endl;
    VectorXd y = z - h_;

    while (y(1) < -M_PI) y(1) += 2. * M_PI;
    while (y(1) > M_PI) y(1) -= 2. * M_PI;
    //cout <<"y" << y <<endl;

    MatrixXd Hj = tools.CalculateJacobian(x_);
    //cout <<"Hj" << Hj <<endl;
    MatrixXd Ht = Hj.transpose();
    MatrixXd PHt = P_ * Ht;
    //cout <<"PHt" << PHt <<endl;
    //cout <<"R_" << R_<<endl;
    //cout << "Hj*PHT" << endl<<Hj*PHt << endl;
    //MatrixXd S = Hj * PHt + R_;
    MatrixXd S = Hj * P_ * Ht + R_;
    //cout <<"S" << S <<endl;

    MatrixXd K = PHt * S.inverse();
    //cout << "K: " << K << endl;

    x_ = x_ + K * y;
    long x_size = x_.size();
    MatrixXd I_ = MatrixXd::Identity(x_size, x_size);
    P_ = (I_ - K * Hj) * P_;

}
