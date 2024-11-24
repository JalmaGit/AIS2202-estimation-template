#include "Fusion.hpp"

Fusion::Fusion(float m_hat, Eigen::VectorXd &r_hat, Eigen::VectorXd &force_var, Eigen::VectorXd &torque_var,
               Eigen::VectorXd &accel_var, Eigen::VectorXd &V_b_hat) {
    m_r_hat = r_hat;
    m_m_hat = m_hat;

    m_identity = Eigen::MatrixXd::Identity(9, 9);
    m_A_k = Eigen::MatrixXd::Identity(9, 9);
    m_B_k = Eigen::MatrixXd::Zero(9, 3);

    m_B_k << Eigen::MatrixXd::Identity(3, 3),
            Eigen::MatrixXd::Identity(3, 3) * m_hat,
            skewSymmetric(r_hat) * m_hat;

    m_Q_k = Eigen::MatrixXd::Zero(9, 9);

    m_force_var = force_var;
    m_torque_var = torque_var;
    m_accel_var = accel_var;

    m_H_f = Eigen::MatrixXd::Zero(6, 9);
    m_H_f << Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(),
    Eigen::MatrixXd::Zero(3,6), Eigen::Matrix3d::Identity();

    m_H_a = Eigen::MatrixXd::Zero(3, 9);
    m_H_a << Eigen::Matrix3d::Identity(), Eigen::MatrixXd::Zero(3, 6);

    m_V_b_hat = V_b_hat;
}

void Fusion::init(float s_a, float s_t, float s_f, float sigma_k) {
    m_s_a = s_a;
    m_s_t = s_t;
    m_s_f = s_f;
    m_sigma_k = sigma_k;

    m_Q_k << Eigen::MatrixXd::Identity(3, 3), Eigen::MatrixXd::Zero(3, 6),
        Eigen::MatrixXd::Zero(3, 3), m_m_hat * Eigen::MatrixXd::Identity(3, 3), Eigen::MatrixXd::Zero(3, 3),
        Eigen::MatrixXd::Zero(3, 6), m_m_hat * m_r_hat.norm() * Eigen::MatrixXd::Identity(3, 3);

    m_Q_k = m_Q_k * m_sigma_k;

    m_R_a = Eigen::MatrixXd::Zero(6, 6);
    m_R_a << m_force_var.diagonal()*s_f, Eigen::Matrix3d::Identity(),
    Eigen::Matrix3d::Identity(), m_torque_var.diagonal()*s_t;

    m_R_f = Eigen::MatrixXd::Zero(3, 3);
    m_R_f << m_accel_var.diagonal()*s_a;

    m_P_k = Eigen::MatrixXd::Zero(9, 9);

    comb_avg_freq = m_f_r/(m_f_f + m_f_a);

    m_kalman_filter = estimation::Kalman_filter(m_A_k,m_B_k,m_H_a,m_P_k,m_R_a,m_Q_k);
}

void Fusion::update(float d_t, Eigen::VectorXd &V_s, Eigen::Vector3d &a, Eigen::Vector3d &g_w, Eigen::Matrix3d &R_ws) {
    Eigen::VectorXd V = V_s - m_V_b_hat;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(9);
    x << a, V;

    Eigen::VectorXd u_k = (R_ws*g_w + m_g_s)*comb_avg_freq;

    Eigen::VectorXd z_k = m_H_a * x;
    m_kalman_filter->update(m_H_a, d_t);
    m_kalman_filter->prediction_update(u_k);
    m_kalman_filter->correction_update(z_k);
}

Eigen::Matrix3d Fusion::skewSymmetric(const Eigen::Vector3d &v) {
    Eigen::Matrix3d skewMat = Eigen::Matrix3d::Zero();
    skewMat.col(0) = v.cross(Eigen::Vector3d::UnitX());
    skewMat.col(1) = v.cross(Eigen::Vector3d::UnitY());
    skewMat.col(2) = v.cross(Eigen::Vector3d::UnitZ());
    return skewMat;
}

//TODO: Calc u_k
//TODO: Calc frequencies
//TODO: Load Datasett
//TODO: Kalman Filter If statements. Check input pattern.