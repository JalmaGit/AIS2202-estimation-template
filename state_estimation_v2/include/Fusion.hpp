#ifndef FUSION_HPP
#define FUSION_HPP

#include <Eigen/Dense>
#include <optional>
#include "Kalman_filter.hpp"
#include <rapidcsv.h>


class Fusion_v2
{
public:
    Fusion_v2(double m_est, const Eigen::VectorXd& r_est, const Eigen::VectorXd& var_a, const Eigen::VectorXd& var_f,
              const Eigen::VectorXd& var_t, const Eigen::VectorXd& fts_bias)
    {
        init_A();
        init_B(m_est, r_est);

        init_P();
        init_Q(m_est, r_est);

        init_Hf();
        init_Ha();

        Rfa_ = Eigen::MatrixXd::Zero(3, 3);
        Rfa_ << 0, -1, 0,
            0, 0, 1,
            -1, 0, 0;

        var_a_ = var_a;
        var_f_ = var_f;
        var_t_ = var_t;
        fts_bias_ = fts_bias;
    };

    void init(double s_a, double s_f, double s_t, double sigma_k)
    {
        init_Rf(s_f, s_t);
        init_Ra(s_a);

        fa_ = calc_freq(data_accel_);
        ff_ = calc_freq(data_wrench_);
        fr_ = calc_freq(data_rotation_);

        Eigen::VectorXd x_init = Eigen::VectorXd::Zero(9);

        kalman_filter_ = estimation::Kalman_filter(A_, B_, P0_, Q_, Ra_, Ha_);
        kalman_filter_->init(x_init);

        sigma_k_ = sigma_k;
    };

private:
    static Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d& vec);
    static Eigen::MatrixXd diagonal_matrix(const Eigen::Vector3d& v);
    static double calc_freq(const Eigen::MatrixXd& data);
    static Eigen::MatrixXd csv_to_mat(rapidcsv::Document& doc);

    void init_A();
    void init_B(double m_est, const Eigen::Vector3d& r_est);
    void init_P();
    void init_Q(double m_est, const Eigen::Vector3d& r_est);
    void init_Hf();
    void init_Ha();

    void init_Rf(double s_f, double s_t);
    void init_Ra(double s_a);

    Eigen::MatrixXd A_, B_, P0_, Q_, Hf_, Ha_, Rf_, Ra_;
    Eigen::VectorXd fts_bias_, var_a_, var_f_, var_t_;
    double sigma_k_, fa_, ff_, fr_;

    Eigen::MatrixXd Rfa_;

    Eigen::MatrixXd data_accel_, data_wrench_, data_rotation_;

    std::optional<estimation::Kalman_filter> kalman_filter_;
};


#endif //FUSION_HPP
