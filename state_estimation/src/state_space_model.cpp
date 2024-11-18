//
// Created by jalma on 11/15/24.
//

#include "state_space_model.hpp"

state_space_model::state_space_model(float m_hat, Eigen::VectorXd &r_hat){
    m_identity = Eigen::MatrixXd::Identity(9, 9);
    m_A_k = Eigen::MatrixXd::Identity(9, 9);
    m_B_k = Eigen::MatrixXd::Zero(9, 3);

    m_B_k << Eigen::MatrixXd::Identity(3, 3),
            Eigen::MatrixXd::Identity(3, 3)*m_hat,
            skewSymmetric(r_hat)*m_hat;

}

Eigen::Matrix3d state_space_model::skewSymmetric(const Eigen::Vector3d &v){
    Eigen::Matrix3d skewMat = Eigen::Matrix3d::Zero();
    skewMat.col(0) = v.cross(Eigen::Vector3d::UnitX());
    skewMat.col(1) = v.cross(Eigen::Vector3d::UnitY());
    skewMat.col(2) = v.cross(Eigen::Vector3d::UnitZ());
    return skewMat;
}