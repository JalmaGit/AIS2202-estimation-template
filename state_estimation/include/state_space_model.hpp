//
// Created by jalma on 11/15/24.
//

#ifndef TEMP_HPP
#define TEMP_HPP

#include <Eigen/Dense>

class state_space_model {
    public:
    state_space_model(float m_hat, Eigen::VectorXd &r_hat);

    private:
    static Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d &v);

    void calculate_variance();
    Eigen::MatrixXd m_identity;
    Eigen::MatrixXd m_A_k;
    Eigen::MatrixXd m_B_k;

};



#endif //TEMP_HPP
