#include "Kalman_filter.hpp"

using namespace estimation;

Kalman_filter::Kalman_filter(const Eigen::MatrixXd &A_k,
                             const Eigen::MatrixXd &B_k,
                             const Eigen::MatrixXd &H_k,
                             const Eigen::MatrixXd &P_k,
                             const Eigen::MatrixXd &R_k,
                             const Eigen::MatrixXd &Q_k)
    : m_A_k(A_k), m_B_k(B_k), m_H_k(H_k), m_P_k(P_k), m_R_k(R_k), m_Q_k(Q_k) {
  m_I = Eigen::MatrixXd::Identity(m_A_k.rows(), m_A_k.rows());
}

void Kalman_filter::init(Eigen::VectorXd &x_hat_k_1, Eigen::MatrixXd &P_k_1) {
  m_x_hat_k = x_hat_k_1;
  m_P_k = P_k_1;
}

void Kalman_filter::update(Eigen::MatrixXd& H_k, Eigen::MatrixXd& R_k, double d_t)
{
  m_H_k = H_k;
  m_dt = d_t;
  m_R_k = R_k;
}

void Kalman_filter::prediction_update(Eigen::VectorXd &u_k) {
  m_x_hat_k = m_A_k * m_x_hat_k + m_B_k * u_k;
  m_P_k = m_A_k * m_P_k * m_A_k.transpose() + m_Q_k * m_dt;
}

void Kalman_filter::correction_update(Eigen::VectorXd &z_k) {
  Eigen::MatrixXd K = m_P_k * m_H_k.transpose() *
                      (m_H_k * m_P_k * m_H_k.transpose() + m_R_k).inverse();
  m_x_hat_k = m_x_hat_k + K * (z_k - m_H_k * m_x_hat_k);
  m_P_k = (m_I - K * m_H_k) * m_P_k;
}

Eigen::VectorXd Kalman_filter::get_kalman_state() { return m_x_hat_k; }
