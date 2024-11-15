#include "kalman_filter.hpp"

using namespace estimation;

kalman_filter::kalman_filter(const Eigen::MatrixXd &A_k, const Eigen::MatrixXd &B_k, const Eigen::MatrixXd &H_k,
                             const Eigen::MatrixXd &P_k, const Eigen::MatrixXd &R_k, const Eigen::MatrixXd &Q_k):
            m_A_k(A_k),
            m_B_k(B_k),
            m_H_k(H_k),
            m_P_k(P_k),
            m_R_k(R_k),
            m_Q_k(Q_k) {
    m_I = Eigen::MatrixXd::Identity(m_A_k.rows(),m_A_k.rows());
}

void kalman_filter::init(Eigen::VectorXd &x_hat_k_1, Eigen::MatrixXd &P_k_1){
    m_x_hat_k = x_hat_k_1;
    m_P_k = P_k_1;
}

void kalman_filter::prediction_update(Eigen::VectorXd &u_k){
    m_x_hat_k = m_A_k * m_x_hat_k + m_B_k * u_k;
    m_P_k = m_A_k * m_P_k * m_A_k.transpose() + m_Q_k;
}


void kalman_filter::correction_update(Eigen::VectorXd &z_k){
    Eigen::MatrixXd K = m_P_k * m_H_k.transpose()*(m_H_k*m_P_k*m_H_k.transpose() + m_R_k).inverse();
    m_x_hat_k = m_x_hat_k + K*(z_k - m_H_k*m_x_hat_k);
    m_P_k = (m_I - K*m_H_k)*m_P_k;
}