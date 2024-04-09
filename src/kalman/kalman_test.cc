// #include "kalman_filter.hpp"

// int main() 
// {
//     double t = 1;
//     Eigen::MatrixXd Q(4,4);
//     Q << 0.1, 0, 0, 0,
//         0, 0.1, 0, 0,
//         0, 0, 0.1, 0,
//         0, 0, 0, 0.1;
//     Eigen::MatrixXd R(2,2);
//     R << 0.1, 0,
//         0, 0.1;

//     // without control
//     Eigen::MatrixXd B = Eigen::MatrixXd::Zero(4,1);
//     Eigen::VectorXd u = Eigen::VectorXd::Zero(1);

//     // // with control, acc = 1
//     // Eigen::MatrixXd B(4,1);
//     // B << 0.5 * pow(t, 2), 0.5 * pow(t, 2), t, t;
//     // Eigen::VectorXd u = Eigen::VectorXd::Zero(1);
//     // u(0,0) = 1;
//     auto kf = std::make_shared<KalmanFilter>(4, 2, 0, Q, R, B);
//     Eigen::VectorXd x_0 = Eigen::VectorXd::Zero(4);
//     kf->init(x_0);
//     int iter = 1;
//     Eigen::VectorXd z_k = Eigen::VectorXd::Zero(2);
//     for (int i = 0; i < iter; i++) {
//         Eigen::VectorXd x_p_k = kf->predict(u, t);
//         z_k << 1, 1;
//         Eigen::VectorXd x_k = kf->update(z_k);
//     }

//     return 0;
// }

#include "extend_kalman.hpp"
#include "matplotlibcpp.h"
#include <iostream>
#include <random>
#include <cmath>
namespace plt = matplotlibcpp;

using namespace Eigen;

double gaussian_distribution(double mean, double std_dev) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::normal_distribution<> dist(mean, std_dev);
    return dist(gen);
}
 
VectorXd forward(VectorXd x_, double v_ , double w_, double t_)
{
    VectorXd x = x_;
    x(0,0) += t_ * x_(3,0) * cos(x_(2,0));
	x(1,0) += t_ * x_(3,0) * sin(x_(2,0));
	x(2,0) += t_ * x_(4,0);
    x(3,0) = v_;
    x(4,0) = w_;
    return x;
}

int main() {

    VectorXd x_last = Eigen::VectorXd::Zero(5, 1);
    VectorXd x_now = x_last;
    double v0 = 0.4;
    double w0 = 0.3;
    double dt = 0.1;
    int n = 300;

    // 设定均值和标准差
    double mean = 0.0;
    double std_dev = 0.1;

    double mean_p = 0.0;
    double std_dev_p = 0.001;

    std::vector<double> x(n), y(n),x_ob(n),y_ob(n),x_ob2(n),y_ob2(n);

    Matrix<double,2,2> Q;
    Q(0,0) = 0.01; Q(0,1) = 0;
    Q(1,0) = 0; Q(1,1) = 0.01;
    Matrix<double,2,2> R;
    R(0,0) = 0.0001; R(0,1) = 0;
    R(1,0) = 0; R(1,1) = 0.0001;

    ExtendKalman kalman(Q,R);
    VectorXd x_0 = Eigen::VectorXd::Zero(5, 1);
    VectorXd u = Eigen::VectorXd::Zero(1, 1);
    VectorXd z = Eigen::VectorXd::Zero(2, 1);
    kalman.init(x_0);

    for(int i = 0; i<n ;++i)
    {
        if(i<100)
        {
            w0 = 0.2;
        }
        else if(i<200)
        {
            w0 = -0.15;
        }
        else
        {
            w0 = 0.25;
        }
        double vk = v0 + gaussian_distribution(mean, std_dev);
        double wk = w0 + gaussian_distribution(mean, std_dev);
        x_now = forward(x_now,vk,wk,dt);
        VectorXd x_pred = kalman.predict(u,dt);

        x.at(i) = x_now(0,0);
        y.at(i) = x_now(1,0);
        x_ob.at(i) = x_pred(0,0);
        y_ob.at(i) = x_pred(1,0);

        // z(0,0) = x_now(0,0)+ gaussian_distribution(mean_p, std_dev_p);
        // z(1,0) = x_now(1,0)+ gaussian_distribution(mean_p, std_dev_p);
        
        z(0,0) = x_now(0,0);
        z(1,0) = x_now(1,0);
        VectorXd x_l = kalman.update(z);
        x_ob2.at(i) = z(0,0);
        y_ob2.at(i) = z(1,0);
    }

    plt::figure_size(1200, 780);

    // plt::plot(x, y);
    // Plot a red dashed line from given x and y data.
    plt::plot(x_ob, y_ob,"r--");
    plt::plot(x_ob2, y_ob2);

    plt::title("Sample figure");
    // Enable legend.
    plt::legend();
    // Save the image (file format is determined by the extension)
    // plt::save("./basic.png");
    plt::show();
    return 0;
}


