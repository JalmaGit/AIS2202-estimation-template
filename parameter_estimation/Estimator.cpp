#include "Estimator.hpp"

Estimator::Estimator(rapidcsv::Document &data) {
  populate_vectors(data);
  calculate_m();
}

void Estimator::populate_vectors(rapidcsv::Document &data) {
  m_force_all.resize(3 * data.GetRowCount());
  m_gravity_all.resize(3 * data.GetRowCount());
  m_torque_all.resize(3 * data.GetRowCount());
  for (int i = 0; i < data.GetRowCount(); i++) {
    auto row = data.GetRow<float>(i);
    Eigen::Vector3d force(row[0], row[1], row[2]);
    Eigen::Vector3d torque(row[3], row[4], row[5]);
    Eigen::Vector3d accel(row[6], row[7], row[8]);
    Eigen::Vector3d gravity_frame(row[9], row[10], row[11]);
    m_force_all.block<3, 1>(i * 3, 0) = force;
    m_torque_all.block<3, 1>(i * 3, 0) = torque;
    m_gravity_all.block<3, 1>(i * 3, 0) = gravity_frame;
  }
}

void Estimator::calculate_m() {
  auto numerator = m_gravity_all.transpose().dot(m_force_all);
  auto denominator = m_gravity_all.transpose().dot(m_gravity_all);
  m_m = numerator / denominator;
}

float Estimator::get_m() { return m_m; }
