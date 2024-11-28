#ifndef FUSION_HPP
#define FUSION_HPP

#include <Eigen/Dense>
#include <optional>
#include "Kalman_filter.hpp"
#include <rapidcsv.h>

class Fusion
{
public:
    Fusion(double m_est, const Eigen::VectorXd& r_est, const Eigen::VectorXd& var_a, const Eigen::VectorXd& var_f,
              const Eigen::VectorXd& var_t, const Eigen::VectorXd& fts_bias, const Eigen::VectorXd& accel_bias);;
    void init(double s_a, double s_f, double s_t, double sigma_k);;
    void load_data_sets(const std::string& accel_file, const std::string& wrench_file, const std::string& rotation_file);
    void run(const std::string& experiment);

private:
    static Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d& vec);
    static Eigen::MatrixXd diagonal_matrix(const Eigen::Vector3d& v);
    static double calc_freq(const Eigen::MatrixXd& data);
    static Eigen::MatrixXd csv_to_mat(rapidcsv::Document& doc);
    static void csv_writer(const std::vector<double>& time, std::vector<Eigen::VectorXd> data, std::string filename, std::vector<std::string>& labels);

    void init_A();
    void init_B(double m_est, const Eigen::Vector3d& r_est);
    void init_P();
    void init_Q(double m_est, const Eigen::Vector3d& r_est);
    void init_Hf();
    void init_Ha();
    void init_Hc(double m_est, const Eigen::Vector3d& r_est);

    void init_Rf(double s_f, double s_t);
    void init_Ra(double s_a);

    Eigen::MatrixXd A_, B_, P0_, Q_, Hf_, Ha_, Rf_, Ra_, Hc_;
    Eigen::VectorXd fts_bias_, accel_bias_, var_a_, var_f_, var_t_;
    double sigma_k_{}, fa_{}, ff_{}, fr_{};

    Eigen::Vector3d gw_{0,0,-9.81};
    Eigen::Vector3d prev_gs_{0,0,0};

    Eigen::MatrixXd Rfa_;

    Eigen::MatrixXd data_accel_, data_wrench_, data_rotation_;

    std::optional<estimation2::Kalman_filter> kf_;
};


#endif //FUSION_HPP
