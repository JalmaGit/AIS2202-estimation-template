//
// Created by jalma on 11/28/24.
//

#include "Variance.hpp"

Variance::Variance(std::string accel_file, std::string wrench_file)
{
    Rfa_ << 0, -1, 0,
        0, 0, 1,
        -1, 0, 0;

    rapidcsv::Document stdy_accel_data(accel_file);
    rapidcsv::Document stdy_wrench_data(wrench_file);

    Eigen::MatrixXd accel = csv_to_mat(stdy_accel_data);
    Eigen::MatrixXd force_torque = csv_to_mat(stdy_wrench_data);

    accel.col(0) *= 9.81;
    accel.col(1) *= 9.81;
    accel.col(2) *= 9.81;

    for (int i = 0; i < accel.rows(); i++)
    {
        Eigen::VectorXd a = Eigen::VectorXd::Zero(3);
        a << accel.row(i)[0], accel.row(i)[1], accel.row(i)[2];

        a << Rfa_*a;

        accel.row(i)[0] = a.col(0)(0);
        accel.row(i)[1] = a.col(0)(1);
        accel.row(i)[2] = a.col(0)(2);
    }

    var_a_ << calculate_variance(accel.col(0)),
        calculate_variance(accel.col(1)),
        calculate_variance(accel.col(2));

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
    std::cout << "accel variance: \n"  << get_a() <<std::endl;
    std::cout << "force variance: \n"  << get_f() <<std::endl;
    std::cout << "torque variance: \n"  << get_t() <<std::endl;
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
