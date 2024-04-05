
#include <cmath>
#include <string>
#include <iostream>
#include <memory>
#include <Eigen/Dense>

class KalmanInterface {
public:
	KalmanInterface(int dim_x, int dim_z, int dim_u)
		: dim_x(dim_x), dim_z(dim_z), dim_u(dim_u) 
	{
		x_p_k = Eigen::VectorXd::Zero(dim_x, 1);
		x_l_k = Eigen::VectorXd::Zero(dim_x, 1);
		x_k = Eigen::VectorXd::Zero(dim_x, 1);
		z_k = Eigen::VectorXd::Zero(dim_z, 1);
		A = Eigen::MatrixXd::Identity(dim_x, dim_x);
		B = Eigen::MatrixXd::Zero(dim_x, dim_u);
		H = Eigen::MatrixXd::Zero(dim_z, dim_x);
	    K = Eigen::MatrixXd::Zero(dim_x, dim_z);
	    Q = Eigen::MatrixXd::Zero(dim_x, dim_x);
	    P = Eigen::MatrixXd::Zero(dim_x, dim_x);
	    R = Eigen::MatrixXd::Zero(dim_z, dim_z);
    }
	KalmanInterface(int dim_x, int dim_z, int dim_u,const Eigen::MatrixXd Q_, const Eigen::MatrixXd R_)
		: dim_x(dim_x), dim_z(dim_z), dim_u(dim_u), Q(Q_),R(R_)
	{
		x_p_k = Eigen::VectorXd::Zero(dim_x, 1);
		x_l_k = Eigen::VectorXd::Zero(dim_x, 1);
		x_k = Eigen::VectorXd::Zero(dim_x, 1);
		z_k = Eigen::VectorXd::Zero(dim_z, 1);
		A = Eigen::MatrixXd::Identity(dim_x, dim_x);
		B = Eigen::MatrixXd::Zero(dim_x, dim_u);
		H = Eigen::MatrixXd::Zero(dim_z, dim_x);
	    K = Eigen::MatrixXd::Zero(dim_x, dim_z);
	    P = Eigen::MatrixXd::Zero(dim_x, dim_x);
	}


	virtual void init(Eigen::VectorXd &x_k) = 0;

	virtual Eigen::VectorXd predict(Eigen::VectorXd &u, double t) = 0;
	
	virtual Eigen::VectorXd update(Eigen::VectorXd &z_k) = 0;



	int dim_x, dim_z, dim_u;			

	Eigen::VectorXd x_p_k;  // predicted state quantity
	Eigen::VectorXd	x_k;    // updated state quantity
	Eigen::VectorXd	x_l_k;  // last moment state
	Eigen::VectorXd	z_k;    // measurement quantity
	Eigen::VectorXd	u;  // controlling quantity

	Eigen::MatrixXd A;	// state-transition matrix
	Eigen::MatrixXd B;  //  control quantity transfer matrix
	Eigen::MatrixXd H;  //  dimensional transformation matrix

	Eigen::MatrixXd	P;  // state error covariance matrix
	Eigen::MatrixXd	K;  // gain matrix
	Eigen::MatrixXd	Q;  // process error covariance matrix
	Eigen::MatrixXd R;  // measurement error covariance matrix

};



