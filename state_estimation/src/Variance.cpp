//
// Created by jalma on 11/28/24.
//

#include "Variance.hpp"

Variance::Variance(std::string accel_file, std::string wrench_file)
{
    rapidcsv::Document stdy_accel_data(accel_file);
    rapidcsv::Document stdy_wrench_data(wrench_file);

    Eigen::MatrixXd accel = csv_to_mat(stdy_accel_data);
    Eigen::MatrixXd force_torque = csv_to_mat(stdy_wrench_data);

    var_a_ << calculate_variance(accel.col(0) * -9.81),
        calculate_variance(accel.col(1) * -9.81),
        calculate_variance(accel.col(2) * -9.81);

    var_f_ << calculate_variance(force_torque.col(0)),
        calculate_variance(force_torque.col(1)),
        calculate_variance(force_torque.col(2));

    var_t_ << calculate_variance(force_torque.col(3)),
        calculate_variance(force_torque.col(4)),
        calculate_variance(force_torque.col(5));
}

Eigen::VectorXd Variance::get_a()
{return var_a_;}

Eigen::VectorXd Variance::get_f()
{return var_f_;}

Eigen::VectorXd Variance::get_t()
{return var_t_;}

void Variance::print_data()
{
    std::cout << get_a() <<std::endl;
    std::cout << get_f() <<std::endl;
    std::cout << get_t() <<std::endl;
}

double Variance::calculate_variance(const Eigen::VectorXd& data)
{
    double mean = data.mean();
    double sum = 0;
    for (double measurement : data) {
        sum += (measurement - mean) * (measurement - mean);
    }
    return sum / (data.size() - 1);
}

Eigen::MatrixXd Variance::csv_to_mat(const rapidcsv::Document& doc)
{
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
