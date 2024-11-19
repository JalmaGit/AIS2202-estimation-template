#include "Kalman_filter.hpp"

#include <Eigen/Dense>
#include <iostream>

void testing_eigen() {
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3, 3);

    Eigen::Vector3d v(1.0, 2.0, 3.0);

    A <<    1.0, 10.0, 100.0,
            1.0, 10.0, 100.0,
            1.0, 10.0, 100.0;

    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(3, 3);

    B <<    1.0, 1.0, 1.0,
            1.0, 1.0, 1.0,
            1.0, 1.0, 1.0;

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(A.rows(), A.cols());

    I.setIdentity();

    A = A * B;
    A = A + B;

    std::cout << "Vector:\n" << v << "\n\n";
    std::cout << "Skew-symmetric matrix using cross:\n" << A << "\n";
}

int main() {

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(9, 9);
    A << Eigen::Matrix3d::Identity(), Eigen::MatrixXd::Zero(3, 6),
    Eigen::MatrixXd::Zero(3,3), 3*Eigen::MatrixXd::Identity(3, 3), Eigen::MatrixXd::Zero(3,3),
    Eigen::MatrixXd::Zero(3, 6), Eigen::MatrixXd::Identity(3, 3);

    Eigen::Vector3d B = Eigen::Vector3d(3.0, 4.0, 0.0);

    auto abs = B.norm();

    std::cout << "Skew-symmetric matrix using cross:\n" << A << std::endl;

    return 0;
}

