#ifndef UTIL
#define UTIL
#include <Eigen/Dense>
Eigen::MatrixXd pseudo_inverse(const Eigen::MatrixXd &A) {
  return A.completeOrthogonalDecomposition().pseudoInverse();
}
#endif // UTIL
