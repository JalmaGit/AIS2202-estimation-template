#include <Eigen/Dense>
#include <rapidcsv.h>

Eigen::MatrixXd pseudo_inverse(const Eigen::MatrixXd &A) {
  return A.completeOrthogonalDecomposition().pseudoInverse();
}

int main() {
  rapidcsv::Document accel("data/1-baseline_accel.csv");
  rapidcsv::Document wrench("data/1-baseline_wrench.csv");
  rapidcsv::Document orientations("data/1-baseline_orientations.csv");

  Eigen::Vector3f gravity(0, 0 - 9.81);

  return 0;
}
