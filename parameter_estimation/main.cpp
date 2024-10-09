#include "Estimator.hpp"
#include <Eigen/Dense>
#include <rapidcsv.h>

int main() {
  rapidcsv::Document data("data/0-calibration_fts-accel.csv");

  Estimator estimator(data);
  std::cout << estimator.get_m() << std::endl;
  std::cout << estimator.get_r() << std::endl;
  return 0;
}
