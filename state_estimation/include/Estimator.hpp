#ifndef ESTIMATOR
#define ESTIMATOR

#include <Eigen/Dense>
#include <rapidcsv.h>
class Estimator {
public:
  Estimator(std::string &file_path);
  float get_m();
  Eigen::VectorXd get_r();
  Eigen::VectorXd get_force_bias();
  Eigen::VectorXd get_torque_bias();
  Eigen::VectorXd get_accel_bias();
  Eigen::VectorXd get_fts_bias();
  void print_data();

private:
  void populate(rapidcsv::Document &data);
  void calculate_m();
  void calculate_r();
  void calculate_biases();
  void calculate_variance();

  Eigen::VectorXd force_all_;
  Eigen::VectorXd torque_all_;
  Eigen::VectorXd accel_all_;
  Eigen::VectorXd gravity_all_;
  Eigen::MatrixXd a_all_;
  Eigen::Vector3d force_bias_;
  Eigen::Vector3d torque_bias_;
  Eigen::Vector3d accel_bias_;
  Eigen::VectorXd r_est_;

  float m_est_{};
};

#endif
