#ifndef ESTIMATOR
#define ESTIMATOR

#include <Eigen/Dense>
#include <rapidcsv.h>

class Estimator {
public:
  Estimator(rapidcsv::Document &data);
  float get_m();

private:
  void populate_vectors(rapidcsv::Document &data);
  void calculate_m();

private:
  Eigen::VectorXd m_force_all;
  Eigen::VectorXd m_torque_all;
  Eigen::VectorXd m_gravity_all;
  float m_m;
};

#endif
