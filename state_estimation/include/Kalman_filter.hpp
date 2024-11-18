#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <Eigen/Dense>

namespace estimation {
    class Kalman_filter {
    public:
        explicit Kalman_filter(const Eigen::MatrixXd &A_k, const Eigen::MatrixXd &B_k,
                               const Eigen::MatrixXd& $H_k, const Eigen::MatrixXd &P_k,
                               const Eigen::MatrixXd &R_k, const Eigen::MatrixXd &Q_k);

        void init(Eigen::VectorXd &x_hat_k_1, Eigen::MatrixXd &P_k_1);

        void prediction_update(Eigen::VectorXd &u_k);
        void correction_update(Eigen::VectorXd &z_k);
        Eigen::VectorXd get_kalman_state();

    private:
        Eigen::MatrixXd m_A_k; //System Matrix
        Eigen::MatrixXd m_B_k; //Input Matrix
        Eigen::MatrixXd m_H_k; //Output Matrix
        Eigen::MatrixXd m_P_k; //Covariance Error Estimate
        Eigen::MatrixXd m_R_k; //White Gaussian Measurement Noise Variance
        Eigen::MatrixXd m_Q_k; //White Gaussian Process Noise Variance
        Eigen::MatrixXd m_I; //Identity Matrix, Must be the same size as A_k

        Eigen::VectorXd m_x_hat_k; //Estimated Kalman State
    };
} // namespace estimation


#endif //KALMAN_FILTER_HPP
