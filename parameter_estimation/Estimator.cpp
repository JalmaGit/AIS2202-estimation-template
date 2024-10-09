#include "Estimator.hpp"
#include "util.hpp"
Estimator::Estimator(rapidcsv::Document &data) {
    populate(data);
    calculate_biases();
    calculate_m();
    calculate_r();
}

void Estimator::populate(rapidcsv::Document &data) {
    m_force_all.resize(3 * data.GetRowCount());
    m_gravity_all.resize(3 * data.GetRowCount());
    m_torque_all.resize(3 * data.GetRowCount());
    m_A_all.resize(3 * data.GetRowCount(), 3);

    for (int i = 0; i < data.GetRowCount(); i++) {
        auto row = data.GetRow<float>(i);
        Eigen::Vector3d force(row[0], row[1], row[2]);
        Eigen::Vector3d torque(row[3], row[4], row[5]);
        Eigen::Vector3d accel(row[6], row[7], row[8]);
        Eigen::Vector3d gravity_frame(row[9], row[10], row[11]);
        m_force_all.block<3, 1>(i * 3, 0) = force;
        m_torque_all.block<3, 1>(i * 3, 0) = torque;
        m_gravity_all.block<3, 1>(i * 3, 0) = gravity_frame;

        Eigen::Matrix3d A;
        A << 0, gravity_frame[2], -gravity_frame[1], -gravity_frame[2], 0,
                gravity_frame[0], gravity_frame[1], -gravity_frame[0], 0;
        m_A_all.block<3, 3>(i * 3, 0) = A;
    }
}

void Estimator::calculate_biases() {
    Eigen::VectorXd force_sum = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd torque_sum = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd accel_sum = Eigen::VectorXd::Zero(3);

    int num_rows = m_force_all.size() / 3;

    for (int i = 0; i < num_rows; i++) {
        force_sum += m_force_all.segment<3>(i * 3);
        torque_sum += m_torque_all.segment<3>(i * 3);
        accel_sum += m_gravity_all.segment<3>(i * 3);
    }

    m_force_bias = force_sum / num_rows;
    m_torque_bias = torque_sum / num_rows;
    m_accel_bias = accel_sum / num_rows;
}
Eigen::Vector3d Estimator::get_force_bias() { return m_force_bias; }
Eigen::Vector3d Estimator::get_torque_bias() { return m_torque_bias; }
Eigen::Vector3d Estimator::get_accel_bias() { return m_accel_bias; }

void Estimator::calculate_m() {
    auto numerator = m_gravity_all.transpose().dot(m_force_all);
    auto denominator = m_gravity_all.transpose().dot(m_gravity_all);
    m_m = numerator / denominator;
}

void Estimator::calculate_r() {
    m_r = (1 / m_m) * (pseudo_inverse(m_A_all) * (m_torque_all));
}

float Estimator::get_m() { return m_m; }

Eigen::VectorXd Estimator::get_r() { return m_r; }
