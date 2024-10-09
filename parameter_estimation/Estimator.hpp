#ifndef ESTIMATOR
#define ESTIMATOR

#include <Eigen/Dense>
#include <rapidcsv.h>
class Estimator {
public:
  Estimator(rapidcsv::Document &data);
  float get_m();
  Eigen::VectorXd get_r();

private:
  void populate(rapidcsv::Document &data);
  void calculate_m();
  void calculate_r();

private:
  Eigen::VectorXd m_force_all;
  Eigen::VectorXd m_torque_all;
  Eigen::VectorXd m_gravity_all;
  Eigen::MatrixXd m_A_all;
  Eigen::VectorXd m_r;
  float m_m;
};

#endif
