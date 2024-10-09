#include "Estimator.hpp"
#include <Eigen/Dense>
#include <rapidcsv.h>

Eigen::MatrixXd pseudo_inverse(const Eigen::MatrixXd &A) {
  return A.completeOrthogonalDecomposition().pseudoInverse();
}

int main() {
  rapidcsv::Document data("data/0-calibration_fts-accel.csv");

  Estimator estimator(data);
  std::cout << estimator.get_m() << std::endl;
  return 0;
}
