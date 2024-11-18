#include "Fusion.hpp"

Fusion::Fusion(float m_hat, Eigen::VectorXd &r_hat, Eigen::VectorXd &force_var,
               Eigen::VectorXd &torqu_var, Eigen::VectorXd &accel_var) {
  m_r_hat = r_hat;
  m_m_hat = m_hat;

  m_identity = Eigen::MatrixXd::Identity(9, 9);
  m_A_k = Eigen::MatrixXd::Identity(9, 9);
  m_B_k = Eigen::MatrixXd::Zero(9, 3);

  m_B_k << Eigen::MatrixXd::Identity(3, 3),
      Eigen::MatrixXd::Identity(3, 3) * m_hat, skewSymmetric(r_hat) * m_hat;

  m_Q_k = Eigen::MatrixXd::Zero(9, 9);

  m_force_var = force_var;
  m_torque_var = torqu_var;
  m_accel_var = accel_var;
}

void Fusion::init(float s_a, float s_t, float s_f, float sigma_k) {
  m_s_a = s_a;
  m_s_t = s_t;
  m_s_f = s_f;
  m_sigma_k = sigma_k;
}

void Fusion::update(float d_t) { Calc_Q(d_t); }

Eigen::Matrix3d Fusion::skewSymmetric(const Eigen::Vector3d &v) {
  Eigen::Matrix3d skewMat = Eigen::Matrix3d::Zero();
  skewMat.col(0) = v.cross(Eigen::Vector3d::UnitX());
  skewMat.col(1) = v.cross(Eigen::Vector3d::UnitY());
  skewMat.col(2) = v.cross(Eigen::Vector3d::UnitZ());
  return skewMat;
}

void Fusion::Calc_Q(float d_t) {
  m_Q_k << Eigen::MatrixXd::Identity(3, 3), Eigen::MatrixXd::Zero(3, 6),
      Eigen::MatrixXd::Zero(3, 3), m_m_hat * Eigen::MatrixXd::Identity(3, 3),
      Eigen::MatrixXd::Zero(3, 3), Eigen::MatrixXd::Zero(3, 6),
      m_m_hat * m_r_hat.norm() * Eigen::MatrixXd::Identity(3, 3);

  m_Q_k = d_t * m_Q_k * m_sigma_k;
}
