#include "Fusion.hpp"

#include <filesystem>
#include <iostream>

Fusion::Fusion(const double m_est, const Eigen::VectorXd& r_est, const Eigen::VectorXd& var_a,
               const Eigen::VectorXd& var_f, const Eigen::VectorXd& var_t, const Eigen::VectorXd& fts_bias, const Eigen::VectorXd& accel_bias)
{
    std::cout<<"Init A and B..."<<std::endl;
    init_A();
    init_B(m_est, r_est);

    std::cout<<"Init Q and P..."<<std::endl;
    init_Q(m_est, r_est);
    init_P();

    std::cout<<"Init H matrices..." <<std::endl;
    init_Hf();
    init_Ha();
    init_Hc(m_est, r_est);

    Rfa_ = Eigen::MatrixXd::Zero(3, 3);
    Rfa_ << 0, -1, 0,
        0, 0, 1,
        -1, 0, 0;

    var_a_ = var_a;
    var_f_ = var_f;
    var_t_ = var_t;
    accel_bias_ = accel_bias;
    fts_bias_ = fts_bias;
}

void Fusion::init(double s_a, double s_f, double s_t, double sigma_k)
{
    std::cout << "Init R and matrices..."<<std::endl;
    init_Rf(s_f, s_t);
    init_Ra(s_a);

    fa_ = calc_freq(data_accel_);
    ff_ = calc_freq(data_wrench_);
    fr_ = calc_freq(data_rotation_);

    Eigen::VectorXd x_init = Eigen::VectorXd::Ones(9);
    x_init << data_accel_.row(0)[1], data_accel_.row(0)[2], data_accel_.row(0)[3],
           data_wrench_.row(0)[1], data_wrench_.row(0)[2], data_wrench_.row(0)[3],
           data_wrench_.row(0)[4], data_wrench_.row(0)[5], data_wrench_.row(0)[6];

    std::cout << "Init kalman filter..." <<std::endl;
    kf_ = estimation2::Kalman_filter(A_, B_, P0_, Q_, Ra_, Ha_);
    kf_->init(x_init);

    sigma_k_ = sigma_k;
}

void Fusion::load_data_sets(const std::string& accel_file, const std::string& wrench_file,
                               const std::string& rotation_file)
{
    std::cout << "Loading data sets..." << std::endl;
    rapidcsv::Document raw_data_accel(accel_file);
    data_accel_ = csv_to_mat(raw_data_accel);

    //Scaling For Gravity
    data_accel_.col(1) *= -9.81;
    data_accel_.col(2) *= -9.81;
    data_accel_.col(3) *= -9.81;

    //rotating the data
    for (int i = 0; i < data_accel_.rows(); i++)
    {
        Eigen::VectorXd a = Eigen::VectorXd::Zero(3);
        a << data_accel_.row(i)[1], data_accel_.row(i)[2], data_accel_.row(i)[3];

        a << Rfa_*a;

        data_accel_.row(i)[1] = a.col(0)(0);
        data_accel_.row(i)[2] = a.col(0)(1);
        data_accel_.row(i)[3] = a.col(0)(2);
    }

    //Removing Bias
    for (int i = 1; i < data_accel_.cols(); ++i) {
        for (int j = 0; j < data_accel_.rows(); ++j) {
            data_accel_.row(j)[i] -= accel_bias_(i - 1, 0);
        }
    }

    rapidcsv::Document raw_data_wrench(wrench_file);
    data_wrench_ = csv_to_mat(raw_data_wrench);

    //Removing the FTS bias from the dataset
    for (int i = 1; i < data_wrench_.cols(); ++i) {
        for (int j = 0; j < data_wrench_.rows(); ++j) {
            data_wrench_.row(j)[i] -= fts_bias_(i - 1, 0);
        }
    }

    rapidcsv::Document raw_data_rotation(rotation_file);
    data_rotation_ = csv_to_mat(raw_data_rotation);

    double zero = 0.0;
    if (data_accel_.row(0)[0] < data_wrench_.row(0)[0] && data_accel_.row(0)[0] < data_rotation_.row(0)[0])
        zero = data_accel_.row(0)[0];
    else if (data_wrench_.row(0)[0] < data_rotation_.row(0)[0])
        zero = data_wrench_.row(0)[0];
    else
        zero = data_rotation_.row(0)[0];

    for (int i = 0; i < data_accel_.rows(); ++i)
        data_accel_.row(i)[0] = data_accel_.row(i)[0] - zero;
    std::cout << "normalized accel time: " << data_accel_.col(0)[0] << std::endl;
    for (int i = 0; i < data_wrench_.rows(); ++i)
        data_wrench_.row(i)[0] = data_wrench_.row(i)[0] - zero;
    std::cout << "normalized wrench time: " << data_wrench_.col(0)[0] << std::endl;

    for (int i = 0; i < data_rotation_.rows(); ++i)
        data_rotation_.row(i)[0] = data_rotation_.row(i)[0] - zero;
    std::cout << "normalized rotation time: " << data_rotation_.col(0)[0] << std::endl;
}

void Fusion::run(const std::string& experiment)
{
    std::cout << "Running Fusion..." << std::endl;
    int a_idx = 1;
    int w_idx = 1;
    int r_idx = 0;

    double t = 0.0;
    double prev_t = 0.0;

    Eigen::VectorXd u = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd x = Eigen::VectorXd::Zero(9);

    Eigen::VectorXd V = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(3);

    a << data_accel_(0,1),data_accel_(0,2),data_accel_(0,3);

    V << data_wrench_(0,1), data_wrench_(0,2), data_wrench_(0,3),
    data_wrench_(0,4), data_wrench_(0,5), data_wrench_(0,6);

    std::vector<Eigen::VectorXd> x_unbiased;
    std::vector<Eigen::VectorXd> x_est;
    std::vector<double> time;

    std::cout<<"Starting While Loop..."<<std::endl;

    while(true)
    {
        if (t == data_accel_(a_idx,0))
        {
            double dt = (t-prev_t)/1000000;
            prev_t = t;

            a << data_accel_(a_idx,1),data_accel_(a_idx,2),data_accel_(a_idx,3);

            Eigen::VectorXd z = Eigen::VectorXd::Zero(3);
            z << a;

            x << a, V;

            Eigen::MatrixXd Q = dt*Q_*sigma_k_;
            kf_->set_Q(Q);
            kf_->set_H(Ha_);
            kf_->set_R(Ra_);
            kf_->predict(u);
            kf_->correct(z);

            x_est.push_back(kf_->get_x());
            x_unbiased.push_back(x);

            time.push_back(t/1000000);


            ++a_idx;
        }
        if (t == data_wrench_(w_idx,0))
        {
            double dt = (t-prev_t)/1000000;
            prev_t = t;

            Eigen::VectorXd z = Eigen::VectorXd::Zero(6);
            V << data_wrench_(w_idx,1), data_wrench_(w_idx,2), data_wrench_(w_idx,3),
            data_wrench_(w_idx,4), data_wrench_(w_idx,5), data_wrench_(w_idx,6);

            z << V;

            x << a, V;

            Eigen::MatrixXd Q = dt*Q_*sigma_k_;
            kf_->set_Q(Q);
            kf_->set_H(Hf_);
            kf_->set_R(Rf_);

            kf_->predict(u);
            kf_->correct(z);

            x_est.push_back(kf_->get_x());
            x_unbiased.push_back(x);

            time.push_back(t/1000000);

            ++w_idx;
        }
        if (t == data_rotation_(r_idx,0))
        {
            Eigen::MatrixXd R_ws = Eigen::MatrixXd::Zero(3,3);
            R_ws << data_rotation_(r_idx,1), data_rotation_(r_idx,2), data_rotation_(r_idx,3),
                data_rotation_(r_idx,4),data_rotation_(r_idx,5),data_rotation_(r_idx,6),
                data_rotation_(r_idx,7),data_rotation_(r_idx,8),data_rotation_(r_idx,9);

            Eigen::VectorXd gs = Eigen::VectorXd::Zero(3);
            gs = R_ws*gw_;
            u = (gs - prev_gs_)*fr_/(ff_ + fa_);
            prev_gs_ = gs;

            ++r_idx;
        }

        if (t == 6000000) //After six seconds, to lazy to scale properly with experiment run time.
            break;

        ++t;
    }

    std::vector<Eigen::VectorXd> cont_wrench;
    for (int i = 0; i<x_est.size(); ++i)
    {
        cont_wrench.push_back(Hc_*x_est[i]);
    }

    std::cout << "Fusion run with " << r_idx+w_idx+a_idx << " iterations" <<std::endl;
    std::vector<std::string> labels{"ax","ay","az","fx","fy","fz","tx","ty","tz"};
    csv_writer(time, x_est, (experiment + "_estimate"), labels);
    csv_writer(time, x_unbiased, (experiment + "_unbiased"), labels);
    labels = {"fx","fy","fz","tx","ty","tz"};
    csv_writer(time,cont_wrench, (experiment + "_cont_wrench"), labels);

    std::cout << "Fusion Complete" << std::endl;
}

Eigen::Matrix3d Fusion::skew_symmetric(const Eigen::Vector3d& vec) {
    Eigen::Matrix3d skew_mat;
    skew_mat <<  0,       -vec(2),  vec(1),
                vec(2),   0,       -vec(0),
               -vec(1),  vec(0),    0;
    return skew_mat;
}


Eigen::MatrixXd Fusion::diagonal_matrix(const Eigen::Vector3d& v)
{
    Eigen::MatrixXd diag = Eigen::MatrixXd::Zero(3, 3);

    diag << v[0], 0, 0,
        0, v[1], 0,
        0, 0, v[2];

    return diag;
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

Eigen::MatrixXd Fusion::csv_to_mat(rapidcsv::Document& doc)
{
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

void Fusion::csv_writer(const std::vector<double>& time, std::vector<Eigen::VectorXd> data, std::string file_name, std::vector<std::string>& labels)
{
    std::cout << "Starting CVS File Writer..." << std::endl;
    rapidcsv::Document doc;

    doc.InsertColumn(0, time, "time");

    std::cout << "Writing Estimation Data To CSV File..." << std::endl;

    for (int i = 0; i < data[0].rows(); ++i) {
        std::vector<double> columnData;
        for (const auto& vec : data) {

            columnData.push_back(vec(i));
        }

        doc.InsertColumn(i + 1, columnData, labels[i]);
    }
    std::cout << "Saving CSV File To plotter/results" << "..." << std::endl;
    std::string filepath = "../../plotter/results/" + file_name + ".csv";
    doc.Save(filepath);
}

void Fusion::init_A()
{
    A_ = Eigen::MatrixXd::Identity(9, 9);
}

void Fusion::init_B(const double m_est, const Eigen::Vector3d& r_est)
{
    B_ = Eigen::MatrixXd::Zero(9, 3);

    B_ << Eigen::MatrixXd::Identity(3, 3),
        Eigen::MatrixXd::Identity(3, 3) * m_est,
        skew_symmetric(r_est) * m_est;
}

void Fusion::init_P()
{
    P0_ = 10* Eigen::MatrixXd::Identity(9,9);
}

void Fusion::init_Q(const double m_est, const Eigen::Vector3d& r_est)
{
    Q_ = Eigen::MatrixXd::Zero(9, 9);
    Q_ << Eigen::MatrixXd::Identity(3, 3), Eigen::MatrixXd::Zero(3, 6),
        Eigen::MatrixXd::Zero(3, 3), m_est * Eigen::MatrixXd::Identity(3, 3), Eigen::MatrixXd::Zero(3, 3),
        Eigen::MatrixXd::Zero(3, 6), m_est * r_est.norm() * Eigen::MatrixXd::Identity(3, 3);
}

void Fusion::init_Hf()
{
    Hf_ = Eigen::MatrixXd::Zero(6, 9);
    Hf_ << Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(),
        Eigen::MatrixXd::Zero(3, 6), Eigen::Matrix3d::Identity();
}

void Fusion::init_Ha()
{
    Ha_ = Eigen::MatrixXd::Zero(3, 9);
    Ha_ << Eigen::Matrix3d::Identity(), Eigen::MatrixXd::Zero(3, 6);
}

void Fusion::init_Hc(const double m_est, const Eigen::Vector3d& r_est)
{
    Hc_ = Eigen::MatrixXd::Zero(6, 9);

    Hc_ << (-m_est) * Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(),
       (-m_est) * skew_symmetric(r_est), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity();

}

void Fusion::init_Rf(const double s_f, const double s_t)
{
    Rf_ = Eigen::MatrixXd::Zero(6, 6);
    Rf_ << diagonal_matrix(var_f_)*s_f, Eigen::Matrix3d::Zero(),
        Eigen::Matrix3d::Zero(), diagonal_matrix(var_t_)*s_t;
}

void Fusion::init_Ra(const double s_a)
{
    Ra_ = Eigen::MatrixXd::Zero(3, 3);
    Ra_ << diagonal_matrix(var_a_)*s_a;
}
