#include "Estimator.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "rapidcsv.h"
#include "util.hpp"

Estimator::Estimator(std::string &file_path) {
  rapidcsv::Document data(file_path);

  populate(data);
  calculate_biases();
  calculate_m();
  calculate_r();
}

void Estimator::populate(rapidcsv::Document &data) {
  force_all_.resize(3 * data.GetRowCount());
  accel_all_.resize(3 * data.GetRowCount());
  gravity_all_.resize(3 * data.GetRowCount());
  torque_all_.resize(3 * data.GetRowCount());
  a_all_.resize(3 * data.GetRowCount(), 3);

  for (int i = 0; i < data.GetRowCount(); i++) {
    auto row = data.GetRow<float>(i);
    Eigen::Vector3d force(row[0], row[1], row[2]);
    Eigen::Vector3d torque(row[3], row[4], row[5]);
    Eigen::Vector3d accel(row[6], row[7], row[8]);
    Eigen::Vector3d gravity_frame(row[9], row[10], row[11]);
    force_all_.block<3, 1>(i * 3, 0) = force;
    torque_all_.block<3, 1>(i * 3, 0) = torque;
    accel_all_.block<3, 1>(i * 3, 0) = accel;
    gravity_all_.block<3, 1>(i * 3, 0) = gravity_frame;

    Eigen::Matrix3d A;
    A << 0, gravity_frame[2], -gravity_frame[1], -gravity_frame[2], 0,
        gravity_frame[0], gravity_frame[1], -gravity_frame[0], 0;
    a_all_.block<3, 3>(i * 3, 0) = A;
  }
}

void Estimator::calculate_biases() {
  Eigen::VectorXd force_sum = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd accel_sum = Eigen::VectorXd::Zero(3);

  int num_rows = force_all_.size() / 3;

  for (int i = 0; i < num_rows; i++) {
    force_sum += force_all_.segment<3>(i * 3);
    accel_sum += gravity_all_.segment<3>(i * 3);
  }
  double tx = 0;
  double ty = 0;
  double tz = 0;
  for (int i = 0; i < 8; i++) {
    tx += torque_all_[i*3];
    ty += torque_all_[i*3 + 8*3 + 1];
    tz += torque_all_[i*3 + 2*8*3 + 2];
  }
  Eigen::Vector3d torque_sum(tx,ty,tz);

  force_bias_ = force_sum / num_rows;
  torque_bias_ = torque_sum / 8;
  accel_bias_ = accel_sum / num_rows;
}
Eigen::VectorXd Estimator::get_force_bias()
{

  return force_bias_;
}
Eigen::VectorXd Estimator::get_torque_bias()
{

  return torque_bias_;
}
Eigen::VectorXd Estimator::get_accel_bias()
{
  return accel_bias_;
}
Eigen::VectorXd Estimator::get_fts_bias() {
  Eigen::VectorXd fts_bias = Eigen::VectorXd::Zero(6);
  fts_bias << force_bias_, torque_bias_;
  return fts_bias;
}

void Estimator::print_data()
{
  std::cout << "mass estimate: " << get_m() << std::endl;
  std::cout << "mass center estimate: " << get_r() << std::endl;
  std::cout << "bias accel: " << get_accel_bias().transpose() << std::endl;
  std::cout << "bias force: " << get_force_bias().transpose() << std::endl;
  std::cout << "bias torque: " << get_torque_bias().transpose() << std::endl;
}

void Estimator::calculate_m() {
  auto numerator = gravity_all_.transpose().dot(force_all_);
  auto denominator = gravity_all_.transpose().dot(gravity_all_);
  m_est_ = numerator / denominator;
}

void Estimator::calculate_r() {
  r_est_ = (1 / m_est_) * (pseudo_inverse(a_all_) * (torque_all_));
}

float Estimator::get_m() { return m_est_; }

Eigen::VectorXd Estimator::get_r()
{
  return r_est_;
}


