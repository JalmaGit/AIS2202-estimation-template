#ifndef FUSION_HPP
#define FUSION_HPP

#include <Eigen/Dense>
#include <optional>
#include "Kalman_filter.hpp"
#include <rapidcsv.h>


class Fusion {
    public:
        Fusion(double m_hat, Eigen::VectorXd &r_hat, Eigen::VectorXd &force_var, Eigen::VectorXd &torque_var, Eigen::VectorXd &accel_var, Eigen::VectorXd &V_b_hat);

        void load_data_sets(std::string accel_file, std::string wrench_file, std::string rotation_file);
        void init(double s_a, double s_t, double s_f, double sigma_k);
        void constants_verify();
        void run();



    private:
        Eigen::MatrixXd diagonal_matrix(const Eigen::Vector3d &v);
        static Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d &v);
        Eigen::MatrixXd csv_to_mat(rapidcsv::Document &doc);
        double calc_freq(const Eigen::MatrixXd &data);

        std::optional<estimation::Kalman_filter> m_kalman_filter;

        Eigen::MatrixXd m_data_accel;
        Eigen::MatrixXd m_data_wrench;
        Eigen::MatrixXd m_data_rotation;

        Eigen::MatrixXd m_identity;
        Eigen::MatrixXd m_A_k;
        Eigen::MatrixXd m_B_k;
        Eigen::MatrixXd m_Q_k;
        Eigen::MatrixXd m_P_k;

        Eigen::MatrixXd m_H_f;
        Eigen::MatrixXd m_H_a;

        Eigen::MatrixXd m_R_f;
        Eigen::MatrixXd m_R_a;

        Eigen::Vector3d m_force_var;
        Eigen::Vector3d m_torque_var;
        Eigen::Vector3d m_accel_var;

        Eigen::VectorXd m_V_b_hat;

        Eigen::Vector3d m_r_hat;

        Eigen::Vector3d m_g_s;
        Eigen::Vector3d m_g_w{0,0,-9.81};

        double comb_avg_freq{};

        double m_m_hat;

        double m_s_a{};
        double m_s_t{};
        double m_s_f{};
        double m_sigma_k{};
};



#endif //FUSION_HPP
