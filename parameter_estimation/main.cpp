#include <rapidcsv.h>
#include <Eigen/Dense>
#include <iostream>
#include "Estimator.hpp"

int main() {
    rapidcsv::Document data("data/0-calibration_fts-accel.csv");

    Estimator estimator(data);
    std::cout << estimator.get_m() << std::endl;
    std::cout << estimator.get_r() << std::endl;
    std::cout << estimator.get_force_bias().transpose() << std::endl;
    std::cout << estimator.get_torque_bias().transpose() << std::endl;
    std::cout << estimator.get_accel_bias().transpose() << std::endl;

    return 0;
}
