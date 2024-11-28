#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <Eigen/Dense>
#include <utility>

namespace estimation2 {
    class Kalman_filter
    {
    public:
        Kalman_filter(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& P, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, const Eigen::MatrixXd& H);

        void init(const Eigen::VectorXd& x);;

        void set_Q (Eigen::MatrixXd& Q);
        void set_R (Eigen::MatrixXd& R);
        void set_H (Eigen::MatrixXd& H);

        void predict(const Eigen::VectorXd &u);;
        void correct(const Eigen::VectorXd &z);;

        Eigen::VectorXd get_x();

    private:
        Eigen::MatrixXd A_, B_, P_, P0_, Q_, R_, H_, I_;

        Eigen::VectorXd x_;


    };
} // namespace estimation


#endif //KALMAN_FILTER_HPP
