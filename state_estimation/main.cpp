#include "kalman_filter.hpp"

#include <Eigen/Dense>
#include <iostream>

// Function to create a skew-symmetric matrix from a Vector3d using Eigen's cross
Eigen::Matrix3d skewSymmetricUsingCross(const Eigen::Vector3d &v) {
    // The skew-symmetric matrix can be constructed as the mapping for cross product
    Eigen::Matrix3d skewMat = Eigen::Matrix3d::Zero();
    skewMat.col(0) = v.cross(Eigen::Vector3d::UnitX());
    skewMat.col(1) = v.cross(Eigen::Vector3d::UnitY());
    skewMat.col(2) = v.cross(Eigen::Vector3d::UnitZ());
    return skewMat;
}

int main() {
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

    return 0;
}

