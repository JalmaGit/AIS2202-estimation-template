#include <iostream>
#include <string>

#include "Fusion.hpp"
#include "Estimator.hpp"
#include "Variance.hpp"

int main() {

  std::cout << "\n ________Getting Bias________" << std::endl;
  std::string data = "data/0-calibration_fts-accel.csv";
  Estimator estimator(data);
  //estimator.print_data();

  std::cout << "\n ________Getting Variance________" << std::endl;
  std::string accel_file = "data/0-steady-state_accel.csv";
  std::string wrench_file = "data/0-steady-state_wrench.csv";
  Variance variance(accel_file, wrench_file);
  //variance.print_data();

  std::cout << "\n __________SENSOR FUSION__________" << std::endl;
  constexpr double s_a{100}, s_f{5000}, s_t{250}, sigma_k{0.5};
  Fusion fusion = Fusion(estimator.get_m(), estimator.get_r(),variance.get_a(),variance.get_f(), variance.get_t(), estimator.get_fts_bias(), estimator.get_accel_bias());
  fusion.load_data_sets("data/1-baseline_accel.csv","data/1-baseline_wrench.csv", "data/1-baseline_orientations.csv");
  fusion.init(s_a,s_f,s_t, sigma_k);
  fusion.run("baseline");

  fusion.load_data_sets("data/2-vibrations_accel.csv","data/2-vibrations_wrench.csv", "data/2-vibrations_orientations.csv");
  fusion.init(s_a,s_f,s_t, sigma_k);
  fusion.run("vibrations");

  fusion.load_data_sets("data/3-vibrations-contact_accel.csv","data/3-vibrations-contact_wrench.csv", "data/3-vibrations-contact_orientations.csv");
  fusion.init(s_a,s_f,s_t, sigma_k);
  fusion.run("contact");

  return 0;
}
