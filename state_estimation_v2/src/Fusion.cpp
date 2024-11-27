#include "Fusion.hpp"

#include <filesystem>

Eigen::Matrix3d Fusion_v2::skew_symmetric(const Eigen::Vector3d& vec)
{
    Eigen::Matrix3d skew_mat = Eigen::Matrix3d::Zero();
    skew_mat.col(0) = vec.cross(Eigen::Vector3d::UnitX());
    skew_mat.col(1) = vec.cross(Eigen::Vector3d::UnitY());
    skew_mat.col(2) = vec.cross(Eigen::Vector3d::UnitZ());
    return skew_mat;
}

Eigen::MatrixXd Fusion_v2::diagonal_matrix(const Eigen::Vector3d& v)
{
    Eigen::MatrixXd diag = Eigen::MatrixXd::Zero(3, 3);

    diag << v[0], 0, 0,
        0, v[1], 0,
        0, 0, v[2];

    return diag;
}

double Fusion_v2::calc_freq(const Eigen::MatrixXd& data)
{
    double total = (data.block(1, 0, data.rows() - 1, 1)
                       - data.block(0, 0, data.rows() - 1, 1))
                   .array()
                   .sum() / 1000000.0;

    double freq = 1/(total / data.rows());

    return freq;
}

Eigen::MatrixXd Fusion_v2::csv_to_mat(rapidcsv::Document& doc)
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

void Fusion_v2::init_A()
{
    A_ = Eigen::MatrixXd::Identity(9, 9);
}

void Fusion_v2::init_B(const double m_est, const Eigen::Vector3d& r_est)
{
    B_ = Eigen::MatrixXd::Zero(9, 3);

    B_ << Eigen::MatrixXd::Identity(3, 3),
        Eigen::MatrixXd::Identity(3, 3) * m_est,
        skew_symmetric(r_est) * m_est;
}

void Fusion_v2::init_P()
{
    P0_ = Eigen::MatrixXd::Zero(9, 9);
}

void Fusion_v2::init_Q(const double m_est, const Eigen::Vector3d& r_est)
{
    Q_ << Eigen::MatrixXd::Identity(3, 3), Eigen::MatrixXd::Zero(3, 6),
        Eigen::MatrixXd::Zero(3, 3), m_est * Eigen::MatrixXd::Identity(3, 3), Eigen::MatrixXd::Zero(3, 3),
        Eigen::MatrixXd::Zero(3, 6), m_est * r_est.norm() * Eigen::MatrixXd::Identity(3, 3);
}

void Fusion_v2::init_Hf()
{
    Hf_ = Eigen::MatrixXd::Zero(6, 9);
    Hf_ << Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(),
        Eigen::MatrixXd::Zero(3, 6), Eigen::Matrix3d::Identity();
}

void Fusion_v2::init_Ha()
{
    Ha_ = Eigen::MatrixXd::Zero(3, 9);
    Ha_ << Eigen::Matrix3d::Identity(), Eigen::MatrixXd::Zero(3, 6);
}

void Fusion_v2::init_Rf(const double s_f, const double s_t)
{
    Rf_ = Eigen::MatrixXd::Zero(6, 6);
    Rf_ << diagonal_matrix(var_f_)*s_f, Eigen::Matrix3d::Zero(),
        Eigen::Matrix3d::Zero(), diagonal_matrix(var_t_)*s_t;
}

void Fusion_v2::init_Ra(const double s_a)
{
    Ra_ = Eigen::MatrixXd::Zero(3, 3);
    Ra_ << diagonal_matrix(var_a_)*s_a;
}
