#ifndef FUSION_HPP
#define FUSION_HPP

#include <Eigen/Dense>
#include <optional>
#include "Kalman_filter.hpp"

class Fusion {
    public:
        Fusion(float m_hat, Eigen::VectorXd &r_hat, Eigen::VectorXd &force_var, Eigen::VectorXd &torque_var, Eigen::VectorXd &accel_var, Eigen::VectorXd &V_b_hat);

        void init(float s_a, float s_t, float s_f, float sigma_k);
        void update(float d_t, Eigen::VectorXd &V_s, Eigen::Vector3d &a, Eigen::Vector3d &G_w, Eigen::Matrix3d &R_ws);

        void load_data_sets();

    private:

        static Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d &v);

        std::optional<estimation::Kalman_filter> m_kalman_filter;

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
        Eigen::VectorXd m_x;

        Eigen::Vector3d m_r_hat;

        Eigen::Vector3d m_g_s;

        float comb_avg_freq{};

        float m_f_a{};
        float m_f_r{};
        float m_f_f{};

        float m_m_hat;

        float m_s_a{};
        float m_s_t{};
        float m_s_f{};
        float m_sigma_k{};
};



#endif //FUSION_HPP
