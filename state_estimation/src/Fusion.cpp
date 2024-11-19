#include "Fusion.hpp"

Fusion::Fusion(float m_hat, Eigen::VectorXd &r_hat, Eigen::VectorXd &force_var, Eigen::VectorXd &torqu_var,
               Eigen::VectorXd &accel_var) {
    m_r_hat = r_hat;
    m_m_hat = m_hat;

    //TODO: Check that all Matrixes are set correctly
    m_identity = Eigen::MatrixXd::Identity(9, 9);
    m_A_k = Eigen::MatrixXd::Identity(9, 9);
    m_B_k = Eigen::MatrixXd::Zero(9, 3);

    m_B_k << Eigen::MatrixXd::Identity(3, 3),
            Eigen::MatrixXd::Identity(3, 3) * m_hat,
            skewSymmetric(r_hat) * m_hat;

    m_Q_k = Eigen::MatrixXd::Zero(9, 9);

    m_force_var = force_var;
    m_torque_var = torqu_var;
    m_accel_var = accel_var;

    m_H_f = Eigen::MatrixXd::Zero(6, 9);
    m_H_f << Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(),
    Eigen::MatrixXd::Zero(3,6), Eigen::Matrix3d::Identity();

    m_H_a = Eigen::MatrixXd::Zero(3, 9);
    m_H_a << Eigen::Matrix3d::Identity(), Eigen::MatrixXd::Zero(3, 6);
}

void Fusion::init(float s_a, float s_t, float s_f, float sigma_k) {
    m_s_a = s_a;
    m_s_t = s_t;
    m_s_f = s_f;
    m_sigma_k = sigma_k;

    //TODO: Calculate m_R_a and m_R_f, a.k.a do diagonal.
    m_R_a = Eigen::MatrixXd::Zero(6, 6);

    m_R_f = Eigen::MatrixXd::Zero(6, 3);
}

void Fusion::update(float d_t) {
    Calc_Q(d_t);
}

void Fusion::Calc_Q(float d_t) {
    m_Q_k << Eigen::MatrixXd::Identity(3, 3), Eigen::MatrixXd::Zero(3, 6),
            Eigen::MatrixXd::Zero(3, 3), m_m_hat * Eigen::MatrixXd::Identity(3, 3), Eigen::MatrixXd::Zero(3, 3),
            Eigen::MatrixXd::Zero(3, 6), m_m_hat * m_r_hat.norm() * Eigen::MatrixXd::Identity(3, 3);

    m_Q_k = d_t * m_Q_k * m_sigma_k;
}

Eigen::Matrix3d Fusion::skewSymmetric(const Eigen::Vector3d &v) {
    Eigen::Matrix3d skewMat = Eigen::Matrix3d::Zero();
    skewMat.col(0) = v.cross(Eigen::Vector3d::UnitX());
    skewMat.col(1) = v.cross(Eigen::Vector3d::UnitY());
    skewMat.col(2) = v.cross(Eigen::Vector3d::UnitZ());
    return skewMat;
}
