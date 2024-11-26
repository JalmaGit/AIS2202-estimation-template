#include <Fusion.hpp>

#include "Kalman_filter.hpp"

#include <Eigen/Dense>
#include <iostream>
#include <rapidcsv.h>
#include <vector>

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

int main() {

  rapidcsv::Document stdy_accel_data("data/0-steady-state_accel.csv");
  rapidcsv::Document stdy_wrench_data("data/0-steady-state_wrench.csv");

  Eigen::MatrixXd force_torque = csv_to_mat(stdy_wrench_data);
  Eigen::MatrixXd accel = csv_to_mat(stdy_accel_data);

  Eigen::VectorXd var_f(3);

  var_f << calculate_variance(force_torque.col(0)),
                             calculate_variance(force_torque.col(1)),
                             calculate_variance(force_torque.col(2));

  Eigen::VectorXd var_t(3);
  var_t << calculate_variance(force_torque.col(3)),
                             calculate_variance(force_torque.col(4)),
                             calculate_variance(force_torque.col(5));

  Eigen::VectorXd var_a(3);

  var_a << calculate_variance(accel.col(1) * 9.81),
                             calculate_variance(accel.col(2) * 9.81),
                             calculate_variance(accel.col(0) * 9.81);

  //std::cout << "[ " << var_f * 250 << " ]" << std::endl;
  //std::cout << "[ " << var_t * 5000 << " ]" << std::endl;
  //std::cout << "[ " << var_a * 100 << " ]" << std::endl;

  double m_hat = 0.932296;
  Eigen::VectorXd r_hat(3);
  Eigen::VectorXd V_b_hat(6);

  r_hat << 0.000279925, 5.43937e-05,0.0438988;
  V_b_hat << 9.07633, -1.01814,  9.98482, 0.432449, -0.692162, -0.156746;

  std::cout << "Running Fusion" << std::endl;
  Fusion fusion{m_hat, r_hat, var_f, var_t, var_a,V_b_hat};
  fusion.load_data_sets("data/1-baseline_accel.csv","data/1-baseline_wrench.csv", "data/1-baseline_orientations.csv");
  fusion.init(100,5000  ,250, 0.5);
  fusion.constants_verify();
  fusion.run();



  return 0;
}
