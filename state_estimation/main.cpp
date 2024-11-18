#include "kalman_filter.hpp"

#include <Eigen/Dense>
#include <iostream>
#include <rapidcsv.h>
#include <vector>

// Function to create a skew-symmetric matrix from a Vector3d using Eigen's
// cross
Eigen::Matrix3d skewSymmetricUsingCross(const Eigen::Vector3d &v) {
  // The skew-symmetric matrix can be constructed as the mapping for cross
  // product
  Eigen::Matrix3d skewMat = Eigen::Matrix3d::Zero();
  skewMat.col(0) = v.cross(Eigen::Vector3d::UnitX());
  skewMat.col(1) = v.cross(Eigen::Vector3d::UnitY());
  skewMat.col(2) = v.cross(Eigen::Vector3d::UnitZ());
  return skewMat;
}

double calculate_variance(Eigen::VectorXd data) {
  double mean = data.mean();
  double sum = 0;
  for (double measurement : data) {
    sum += (measurement - mean) * (measurement - mean);
  }
  return sum / (data.size() - 1);
}

Eigen::MatrixXd csv_to_mat(rapidcsv::Document doc) {
  int row_count = doc.GetRowCount();
  int col_count = doc.GetColumnCount();
  Eigen::MatrixXd data(row_count, col_count);

  for (int i = 0; i < col_count; i++) {
    std::vector<double> column = doc.GetColumn<double>(i);
    Eigen::VectorXd col =
        Eigen::Map<Eigen::VectorXd>(column.data(), column.size());
    data.block(0, i, row_count, 1) = col;
  }
  return data;
}

double g_to_mps2(double g) { return g * 9.81; }
Eigen::VectorXd g_to_mps2(Eigen::VectorXd g) { return g * 9.81; }
int main() {

  rapidcsv::Document stdy_accel_data("data/0-steady-state_accel.csv");
  rapidcsv::Document stdy_wrench_data("data/0-steady-state_wrench.csv");

  Eigen::MatrixXd force_torque = csv_to_mat(stdy_wrench_data);
  Eigen::MatrixXd accel = csv_to_mat(stdy_accel_data);

  Eigen::RowVector3d sigma_f(calculate_variance(force_torque.col(0)),
                             calculate_variance(force_torque.col(1)),
                             calculate_variance(force_torque.col(2)));

  Eigen::RowVector3d sigma_t(calculate_variance(force_torque.col(3)),
                             calculate_variance(force_torque.col(4)),
                             calculate_variance(force_torque.col(5)));

  Eigen::RowVector3d sigma_a(calculate_variance(g_to_mps2(accel.col(0))),
                             calculate_variance(g_to_mps2(accel.col(1))),
                             calculate_variance(g_to_mps2(accel.col(2))));

  std::cout << "[ " << sigma_f * 250 << " ]" << std::endl;
  std::cout << "[ " << sigma_t * 5000 << " ]" << std::endl;
  std::cout << "[ " << sigma_a * 100 << " ]" << std::endl;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3, 3);

  Eigen::Vector3d v(1.0, 2.0, 3.0);

  A << 1.0, 10.0, 100.0, 1.0, 10.0, 100.0, 1.0, 10.0, 100.0;

  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(3, 3);

  B << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(A.rows(), A.cols());

  I.setIdentity();

  A = A * B;
  A = A + B;

  std::cout << "Vector:\n" << v << "\n\n";
  std::cout << "Skew-symmetric matrix using cross:\n" << A << "\n";

  return 0;
}
