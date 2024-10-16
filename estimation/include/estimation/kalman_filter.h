#ifndef ESTIMATION_KALMAN_FILTER_H
#define ESTIMATION_KALMAN_FILTER_H

#include <Eigen/Dense>

namespace estimation {
class kalman_filter {
public:
  kalman_filter();

private:
  void calculate_variance();
  float m_var_imu;
  float m_var_fts;
};
} // namespace estimation

#endif
