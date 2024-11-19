#ifndef FUSION_HPP
#define FUSION_HPP

#include <Eigen/Dense>

class Fusion {
    public:
        Fusion(float m_hat, Eigen::VectorXd &r_hat, Eigen::VectorXd &force_var, Eigen::VectorXd &torqu_var, Eigen::VectorXd &accel_var);

        void init(float s_a, float s_t, float s_f, float sigma_k);
        void update(float d_t);


    private:
        void Calc_Q(float d_t);

        static Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d &v);


        Eigen::MatrixXd m_identity;
        Eigen::MatrixXd m_A_k;
        Eigen::MatrixXd m_B_k;
        Eigen::MatrixXd m_Q_k;

        Eigen::MatrixXd m_H_f;
        Eigen::MatrixXd m_H_a;

        Eigen::MatrixXd m_R_f;
        Eigen::MatrixXd m_R_a;

        Eigen::Vector3d m_force_var;
        Eigen::Vector3d m_torque_var;
        Eigen::Vector3d m_accel_var;

        Eigen::Vector3d m_r_hat;
        float m_m_hat;

        float m_s_a;
        float m_s_t;
        float m_s_f;
        float m_sigma_k;
};



#endif //FUSION_HPP
