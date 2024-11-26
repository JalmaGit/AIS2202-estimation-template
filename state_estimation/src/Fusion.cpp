#include "Fusion.hpp"

#include <filesystem>

Fusion::Fusion(double m_hat, Eigen::VectorXd &r_hat, Eigen::VectorXd &force_var, Eigen::VectorXd &torque_var,
               Eigen::VectorXd &accel_var, Eigen::VectorXd &V_b_hat) {
    m_r_hat = r_hat;
    m_m_hat = m_hat;

    std::cout<<"Calc A and B..."<<std::endl;
    m_identity = Eigen::MatrixXd::Identity(9, 9);
    m_A_k = Eigen::MatrixXd::Identity(9, 9);
    m_B_k = Eigen::MatrixXd::Zero(9, 3);

    m_B_k << Eigen::MatrixXd::Identity(3, 3),
            Eigen::MatrixXd::Identity(3, 3) * m_hat,
            skewSymmetric(r_hat) * m_hat;

    m_Q_k = Eigen::MatrixXd::Zero(9, 9);

    std::cout<<"Assigning variances..."<<std::endl;
    m_force_var = force_var;
    m_torque_var = torque_var;
    m_accel_var = accel_var;


    std::cout<<"Initialising H Matrices..."<<std::endl;
    m_H_f = Eigen::MatrixXd::Zero(6, 9);
    m_H_f << Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(),
    Eigen::MatrixXd::Zero(3,6), Eigen::Matrix3d::Identity();

    m_H_a = Eigen::MatrixXd::Zero(3, 9);
    m_H_a << Eigen::Matrix3d::Identity(), Eigen::MatrixXd::Zero(3, 6);

    m_V_b_hat = V_b_hat;
}

void Fusion::load_data_sets(std::string accel_file, std::string wrench_file, std::string rotation_file)
{

    std::cout<<"Loading data sets..."<<std::endl;
    rapidcsv::Document raw_data_accel("data/1-baseline_accel.csv");
    m_data_accel = csv_to_mat(raw_data_accel);
    m_data_accel.col(1) = m_data_accel.col(1) * 9.81;
    m_data_accel.col(2) = m_data_accel.col(2) * 9.81;
    m_data_accel.col(3) = m_data_accel.col(3) * 9.81;

    rapidcsv::Document raw_data_wrench("data/1-baseline_wrench.csv");
    m_data_wrench = csv_to_mat(raw_data_wrench);
    rapidcsv::Document raw_data_rotation("data/1-baseline_orientations.csv");
    m_data_rotation = csv_to_mat(raw_data_rotation);
}

void Fusion::init(double s_a, double s_t, double s_f, double sigma_k) {
    m_s_a = s_a;
    m_s_t = s_t;
    m_s_f = s_f;
    m_sigma_k = sigma_k;

    const double f_a = calc_freq(m_data_accel);
    const double f_f = calc_freq(m_data_wrench);
    const double f_r = calc_freq(m_data_rotation);

    comb_avg_freq = f_r/(f_f + f_a);

    std::cout << "Calc Q..." << std::endl;
    m_Q_k << Eigen::MatrixXd::Identity(3, 3), Eigen::MatrixXd::Zero(3, 6),
        Eigen::MatrixXd::Zero(3, 3), m_m_hat * Eigen::MatrixXd::Identity(3, 3), Eigen::MatrixXd::Zero(3, 3),
        Eigen::MatrixXd::Zero(3, 6), m_m_hat * m_r_hat.norm() * Eigen::MatrixXd::Identity(3, 3);

    m_Q_k = m_Q_k * m_sigma_k;

    std::cout << m_force_var.diagonal() << std::endl;

    std::cout<<"Initialising R Matrices..."<<std::endl;
    m_R_f = Eigen::MatrixXd::Zero(6, 6);
    m_R_f << diagonal_matrix(m_force_var)*s_f, Eigen::Matrix3d::Identity(),
    Eigen::Matrix3d::Identity(), diagonal_matrix(m_torque_var)*s_t;

    m_R_a = Eigen::MatrixXd::Zero(3, 3);
    m_R_a << diagonal_matrix(m_accel_var)*s_a;

    m_P_k = Eigen::MatrixXd::Zero(9, 9);

    std::cout<<"Constructing Kalman Filter..."<<std::endl;
    m_kalman_filter = estimation::Kalman_filter(m_A_k,m_B_k,m_H_a,m_P_k,m_R_a,m_Q_k);

    Eigen::MatrixXd P_k_1 = Eigen::MatrixXd::Zero(9, 9);
    Eigen::VectorXd x_hat_k_1 = Eigen::VectorXd::Zero(9);

    std::cout<<"Initializing Kalman Filter..."<<std::endl;
    m_kalman_filter->init(x_hat_k_1,P_k_1);
}

//TODO: Kalman Filter algorithm.

void Fusion::run() {

    std::cout << "Starting Estimation ..." << std::endl;

    Eigen::VectorXd V_s = Eigen::VectorXd::Zero(6);
    V_s << m_data_wrench.row(0)[1], m_data_wrench.row(0)[2], m_data_wrench.row(0)[3]
    , m_data_wrench.row(0)[4], m_data_wrench.row(0)[5], m_data_wrench.row(0)[6];

    Eigen::VectorXd V = V_s - m_V_b_hat;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(9);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(3);
    a << m_data_accel.row(0)[1],m_data_accel.row(0)[2],m_data_accel.row(0)[3];
    x << a, V;

    Eigen::MatrixXd R_ws = Eigen::MatrixXd::Zero(3, 3);
    R_ws << m_data_rotation.row(0)[1],m_data_rotation.row(0)[2],m_data_rotation.row(0)[3],
    m_data_rotation.row(0)[4], m_data_rotation.row(0)[5], m_data_rotation.row(0)[6],
    m_data_rotation.row(0)[7], m_data_rotation.row(0)[8], m_data_rotation.row(0)[9];

    Eigen::VectorXd u_k = (R_ws*m_g_w + m_g_s)*comb_avg_freq;
    m_g_s = R_ws*m_g_w;

    int a_idx = 1;
    int w_idx = 1;
    int r_idx = 1;

    std::vector<Eigen::VectorXd> x_hat;
    std::vector<double> time;

    while (true)
    {
        if (m_data_accel.row(a_idx)[0] <  m_data_wrench.row(w_idx)[0] || m_data_accel.row(a_idx)[0]  < m_data_rotation.row(r_idx)[0])
        {
            double d_t = (m_data_accel.row(a_idx)[0]-m_data_accel.row(a_idx-1)[0])/1000000;
            time.push_back((m_data_accel.row(a_idx)[0]-m_data_accel.row(0)[0])/1000000);

            Eigen::VectorXd z_k = m_H_a * x;

            m_kalman_filter->update(m_H_a, m_R_a, d_t);
            m_kalman_filter->prediction_update(u_k);
            m_kalman_filter->correction_update(z_k);
            x_hat.push_back(m_kalman_filter->get_kalman_state());

            a_idx++;
        }
        else if(m_data_wrench.row(w_idx)[0] <  m_data_rotation.row(r_idx)[0])
        {
            double d_t = (m_data_wrench.row(w_idx)[0]-m_data_wrench.row(w_idx-1)[0])/1000000;
            time.push_back((m_data_wrench.row(w_idx)[0]-m_data_wrench.row(0)[0] )/1000000);

            Eigen::VectorXd z_k = m_H_f * x;

            m_kalman_filter->update(m_H_f, m_R_f, d_t);
            m_kalman_filter->prediction_update(u_k);
            m_kalman_filter->correction_update(z_k);
            x_hat.push_back(m_kalman_filter->get_kalman_state());

            w_idx++;
        }
        else
        {
            R_ws << m_data_rotation.row(r_idx)[1],m_data_rotation.row(r_idx)[2],m_data_rotation.row(r_idx)[3],
            m_data_rotation.row(r_idx)[4], m_data_rotation.row(r_idx)[5], m_data_rotation.row(r_idx)[6],
            m_data_rotation.row(r_idx)[7], m_data_rotation.row(r_idx)[8], m_data_rotation.row(r_idx)[9];

            u_k = (R_ws*m_g_w + m_g_s)*comb_avg_freq;
            m_g_s = R_ws*m_g_w;

            r_idx++;
        }

        if (a_idx == m_data_accel.rows() || w_idx == m_data_wrench.rows() || r_idx == m_data_rotation.rows())
            break;

    }
    std::cout << "Fusion run with " << r_idx+w_idx+a_idx << " iterations" <<std::endl;

    std::cout << "Starting CVS File Writer..." << std::endl;
    rapidcsv::Document doc;
    std::string fileName = "data/results/baseline.csv";

    std::cout << "Writing Time Data to CSV File..." << std::endl;
    doc.InsertColumn(0, time, "time");
    std::vector<std::string> labels{"ax","ay","az","fx","fy","fz","tx","ty","tz"};

    std::cout << "Writing Estimation Data To CSV File..." << std::endl;

    for (int i = 0; i < x_hat[0].rows(); ++i) {

        std::vector<double> columnData;
        for (const auto& vec : x_hat) {
            columnData.push_back(vec(i));
        }

        doc.InsertColumn(i + 1, columnData, labels[i]);
    }
    std::cout << "Saving CSV File To " << std::filesystem::current_path() << "..." << std::endl;
    doc.Save("../../data/results/output.csv");

    std::cout << "Fusion Complete" << std::endl;
}

Eigen::MatrixXd Fusion::diagonal_matrix(const Eigen::Vector3d& v)
{
    Eigen::MatrixXd diag = Eigen::MatrixXd::Zero(3,3);

    diag << v[0], 0, 0,
            0, v[1], 0,
            0, 0, v[2];

    return diag;
}

Eigen::Matrix3d Fusion::skewSymmetric(const Eigen::Vector3d &v) {
    Eigen::Matrix3d skewMat = Eigen::Matrix3d::Zero();
    skewMat.col(0) = v.cross(Eigen::Vector3d::UnitX());
    skewMat.col(1) = v.cross(Eigen::Vector3d::UnitY());
    skewMat.col(2) = v.cross(Eigen::Vector3d::UnitZ());
    return skewMat;
}

double Fusion::calc_freq(const Eigen::MatrixXd& data)
{
    double total = (data.block(1, 0, data.rows() - 1, 1)
              - data.block(0, 0, data.rows() - 1, 1))
              .array()
              .sum() / 1000000.0;

    double freq = 1/(total / data.rows());

    return freq;
}

Eigen::MatrixXd Fusion::csv_to_mat(rapidcsv::Document &doc) {
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
