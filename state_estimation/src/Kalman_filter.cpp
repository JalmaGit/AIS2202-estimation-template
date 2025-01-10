#include "Kalman_filter.hpp"
#include <iostream>
#include <utility>

estimation2::Kalman_filter::Kalman_filter(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                                         const Eigen::MatrixXd& P, const Eigen::MatrixXd& Q,
                                         const Eigen::MatrixXd& R, const Eigen::MatrixXd& H):
    A_(A),
    B_(B),
    P_(P),
    Q_(Q),
    R_(R),
    H_(H){
    P0_ = P_;
    I_ = Eigen::MatrixXd::Identity(A.rows(),A.cols());
}

void estimation2::Kalman_filter::init(const Eigen::VectorXd& x){
    x_ = x;
    P_ = P0_;
}

void estimation2::Kalman_filter::set_Q(Eigen::MatrixXd& Q){
    Q_= Q;
}

void estimation2::Kalman_filter::set_R(Eigen::MatrixXd& R){
    R_ = R;
}

void estimation2::Kalman_filter::set_H(Eigen::MatrixXd& H){
    H_ = H;
}

void estimation2::Kalman_filter::predict(const Eigen::VectorXd& u){
    x_ = A_*x_ + B_*u;
    P_ = A_*P_*A_.transpose() + Q_;
}

void estimation2::Kalman_filter::correct(const Eigen::VectorXd& z){
    Eigen::MatrixXd K = P_*H_.transpose()*(H_*P_*H_.transpose() + R_).inverse();
    x_ = x_ + K*(z-H_*x_);
    P_ = (I_ - K*H_)*P_;
}

Eigen::VectorXd estimation2::Kalman_filter::get_x(){
    return x_;
}
