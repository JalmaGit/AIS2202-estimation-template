//
// Created by jalma on 11/28/24.
//

#ifndef VARIANCE_HPP
#define VARIANCE_HPP
#include <Eigen/Dense>
#include <rapidcsv.h>
#include <vector>

class Variance {
    public: Variance(std::string accel_file, std::string wrench_file);

    Eigen::VectorXd get_a();
    Eigen::VectorXd get_f();
    Eigen::VectorXd get_t();
    void print_data();

private:
    static double calculate_variance(const Eigen::VectorXd& data);

    static Eigen::MatrixXd csv_to_mat(const rapidcsv::Document& doc);

    Eigen::VectorXd var_a_ = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd var_f_ = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd var_t_ = Eigen::VectorXd::Zero(3);
};



#endif //VARIANCE_HPP
