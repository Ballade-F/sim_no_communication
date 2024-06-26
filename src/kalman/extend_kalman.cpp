#include "extend_kalman.hpp"


void ExtendKalman::init(Eigen::VectorXd &x_k) 
{
	x_p_k = x_k;
	x_l_k = x_k;
	P = Eigen::MatrixXd::Zero(dim_x, dim_x);
	H(0,0) = 1;
	H(1,1) = 1;
}

Eigen::VectorXd ExtendKalman::predict(Eigen::VectorXd &u, double t) 
{
	Eigen::VectorXd delta_x = Eigen::VectorXd::Zero(dim_x, 1);
	delta_x(0,0) = t * x_l_k(3,0) * cos(x_l_k(2,0));
	delta_x(1,0) = t * x_l_k(3,0) * sin(x_l_k(2,0));
	delta_x(2,0) = t * x_l_k(4,0);

	x_p_k = x_l_k + delta_x;


	A = Eigen::MatrixXd::Identity(dim_x, dim_x);
	A(0,2) = -t * x_l_k(3,0) * sin(x_l_k(2,0));
	A(0,3) = t * cos(x_l_k(2,0));
	A(1,2) = t * x_l_k(3,0) * cos(x_l_k(2,0));
	A(1,3) = t * sin(x_l_k(2,0));
	A(2,4) = t;


	Eigen::MatrixXd W = Eigen::MatrixXd::Zero(5, 2);
	W(0,0) = cos(x_l_k(2, 0)) * pow(t, 2) / 2;
	W(1,0) = sin(x_l_k(2, 0)) * pow(t, 2) / 2;
	W(2,0) = t;
	W(3,1) = pow(t, 2) / 2;
	W(4,1) = t;

	P = A * P * A.transpose() + W * Q * W.transpose();
	
	return x_p_k;

   	// A = df_ctrv(x_l_k, u, t);
    // Eigen::MatrixXd W = se_df_ctrv(x_l_k, u, t);
   	// x_p_k = ctrv(x_l_k, u, t);
	// P = A * P * A.transpose() + W * Q * W.transpose();
	// return x_p_k;
}

//TODO: 对v和w限幅
Eigen::VectorXd ExtendKalman::update(Eigen::VectorXd &z_k) 
{
	K = P * H.transpose() * (H * P * H.transpose() +  R).inverse();
	x_k = x_p_k + K * (z_k - H*x_p_k);

	//对v和w限幅
	x_k(3,0) = x_k(3,0)<0?0:x_k(3,0)>0.5?0.5:x_k(3,0);
	x_k(4,0) = x_k(4,0)<-0.5?-0.5:x_k(4,0)>0.5?0.5:x_k(4,0);


	P = P - K * H * P;
	x_l_k = x_k;
	return x_k;
}

// Eigen::MatrixXd ExtendKalman::ctrv(Eigen::VectorXd &x_l_k, Eigen::VectorXd &u, double t) {
// 	double ctrv_delta_t = t;

// 	B(0, 0) = cos(x_l_k(3, 0) * KALMAN_DEC) *
// 				  pow(ctrv_delta_t, 2) / 2;
// 	B(1, 0) = sin(x_l_k(3, 0) * KALMAN_DEC) *
// 				  pow(ctrv_delta_t, 2) / 2;
// 	B(2, 0) = ctrv_delta_t;
// 	B(3, 1) = pow(ctrv_delta_t, 2) / 2;
// 	B(4, 1) = ctrv_delta_t;
// 	B(0, 1) = 0; B(1, 1) = 0; B(2, 1) = 0; B(3, 0) = 0; B(4, 0) = 0;

// 	if (x_l_k(4, 0) != 0)
// 	{
// 		A(0, 0) = (1 + x_l_k(2, 0) * (sin(x_l_k(3, 0) * KALMAN_DEC + ctrv_delta_t * x_l_k(4, 0) * KALMAN_DEC) 
//             - sin(x_l_k(3, 0) * KALMAN_DEC)) / (x_l_k(4, 0) * x_l_k(0, 0)));
// 		A(1, 1) = (1 + x_l_k(2, 0) * (-cos(x_l_k(3, 0) * KALMAN_DEC + ctrv_delta_t * x_l_k(4, 0) * KALMAN_DEC) 
//             + cos(x_l_k(3, 0) * KALMAN_DEC)) / (x_l_k(4, 0) * x_l_k(1, 0)));
// 	}
// 	else if (x_l_k(4, 0) == 0)
// 	{
// 		A(0, 0) = (1 + x_l_k(2, 0) * ctrv_delta_t * cos(x_l_k(3, 0) * KALMAN_DEC) / x_l_k(0, 0));
// 		A(1, 1) = (1 + x_l_k(2, 0) * ctrv_delta_t * sin(x_l_k(3, 0) * KALMAN_DEC) / x_l_k(1, 0));
// 	}
// 	A(2, 2) = 1; A(3, 3) = 1; A(4, 4) = 1; A(3, 4) = ctrv_delta_t; A(0, 1) = 0;
// 	A(0, 2) = 0; A(0, 3) = 0; A(0, 4) = 0; A(1, 0) = 0; A(1, 2) = 0; A(1, 3) = 0;
// 	A(1, 4) = 0; A(2, 0) = 0; A(2, 1) = 0; A(2, 3) = 0; A(2, 4) = 0; A(3, 0) = 0;
// 	A(3, 1) = 0; A(3, 2) = 0; A(4, 0) = 0; A(4, 1) = 0; A(4, 2) = 0; A(4, 3) = 0;

// 	x_p_k = A * x_l_k + B * u;
// 	return x_p_k;
// }

// Eigen::MatrixXd ExtendKalman::df_ctrv(Eigen::VectorXd &x_l_k, Eigen::VectorXd &u, double t)
// {
// 	(void) u;
// 	double ctrv_delta_t = t;
// 	Eigen::MatrixXd F_Matrix(5, 5);

// 	if (x_l_k(4, 0) != 0) {
// 		F_Matrix(0, 0) = 1;
// 		F_Matrix(0, 1) = 0;
// 		F_Matrix(0, 2) = (-sin(x_l_k(3, 0) * KALMAN_DEC) + sin(ctrv_delta_t * x_l_k(4, 0) * KALMAN_DEC + x_l_k(3, 0) * KALMAN_DEC)) / x_l_k(4, 0);
// 		F_Matrix(0, 3) = x_l_k(2, 0) * (-cos(x_l_k(3, 0) * KALMAN_DEC) + cos(ctrv_delta_t * x_l_k(4, 0) * KALMAN_DEC + x_l_k(3, 0) * KALMAN_DEC)) / x_l_k(4, 0);
// 		F_Matrix(0, 4) = ctrv_delta_t * x_l_k(2, 0) / x_l_k(4, 0) * cos(ctrv_delta_t * x_l_k(4, 0) * KALMAN_DEC + x_l_k(3, 0) * KALMAN_DEC) -
// 							 x_l_k(2, 0) / pow(x_l_k(4, 0), 2) * (-sin(x_l_k(3, 0) * KALMAN_DEC) + sin(ctrv_delta_t * x_l_k(4, 0) * KALMAN_DEC + x_l_k(3, 0) * KALMAN_DEC));
// 		F_Matrix(1, 0) = 0;
// 		F_Matrix(1, 1) = 1;
// 		F_Matrix(1, 3) = x_l_k(2, 0) * (-sin(x_l_k(3, 0) * KALMAN_DEC) + sin(ctrv_delta_t * x_l_k(4, 0) * KALMAN_DEC + x_l_k(3, 0) * KALMAN_DEC)) / x_l_k(4, 0);
// 		F_Matrix(1, 2) = (cos(x_l_k(3, 0) * KALMAN_DEC) - cos(ctrv_delta_t * x_l_k(4, 0) * KALMAN_DEC + x_l_k(3, 0) * KALMAN_DEC)) / x_l_k(4, 0);
// 		F_Matrix(1, 4) = ctrv_delta_t * x_l_k(2, 0) / x_l_k(4, 0) * sin(ctrv_delta_t * x_l_k(4, 0) * KALMAN_DEC + x_l_k(3, 0) * KALMAN_DEC) - x_l_k(2, 0) /
// 						pow(x_l_k(4, 0), 2) * (cos(x_l_k(3, 0) * KALMAN_DEC) - cos(ctrv_delta_t * x_l_k(4, 0) * KALMAN_DEC + x_l_k(3, 0) * KALMAN_DEC));
// 	}
// 	else if (x_l_k(4, 0) == 0) {
// 		F_Matrix(0, 0) = 1; F_Matrix(0, 1) = 0; F_Matrix(0, 4) = 0;
// 		F_Matrix(0, 2) = ctrv_delta_t * cos(x_l_k(3, 0) * KALMAN_DEC);
// 		F_Matrix(0, 3) = -ctrv_delta_t * x_l_k(2, 0) * sin(x_l_k(3, 0) * KALMAN_DEC);
// 		F_Matrix(1, 0) = 0; F_Matrix(1, 1) = 1; F_Matrix(1, 4) = 0;
// 		F_Matrix(1, 2) = ctrv_delta_t * sin(x_l_k(3, 0) * KALMAN_DEC);
// 		F_Matrix(1, 3) = ctrv_delta_t * x_l_k(2, 0) * cos(x_l_k(3, 0) * KALMAN_DEC);
// 	}
// 	F_Matrix(2, 0) = 0; F_Matrix(2, 1) = 0; F_Matrix(2, 2) = 1; F_Matrix(2, 3) = 0;
// 	F_Matrix(2, 4) = 0; F_Matrix(3, 0) = 0; F_Matrix(3, 1) = 0; F_Matrix(3, 2) = 0;
// 	F_Matrix(3, 3) = 1; F_Matrix(3, 4) = ctrv_delta_t; F_Matrix(4, 0) = 0; F_Matrix(4, 1) = 0;
// 	F_Matrix(4, 2) = 0; F_Matrix(4, 3) = 0; F_Matrix(4, 4) = 1;

// 	return F_Matrix;
// }

// Eigen::MatrixXd ExtendKalman::se_df_ctrv(Eigen::VectorXd &x_l_k, Eigen::VectorXd &u, double t) {
// 	(void) x_l_k;
// 	(void) u;
// 	double ctrv_delta_t = t;
// 	(void) ctrv_delta_t;
// 	Eigen::MatrixXd Matrix(5, 5);
// 	Matrix(0, 0) = 1; Matrix(0, 1) = 0; Matrix(0, 2) = 0; Matrix(0, 3) = 0; Matrix(0, 4) = 0;
// 	Matrix(1, 0) = 0; Matrix(1, 1) = 1; Matrix(1, 2) = 0; Matrix(1, 3) = 0; Matrix(1, 4) = 0;
// 	Matrix(2, 0) = 0; Matrix(2, 1) = 0; Matrix(2, 2) = 1; Matrix(2, 3) = 0; Matrix(2, 4) = 0;
// 	Matrix(3, 0) = 0; Matrix(3, 1) = 0; Matrix(3, 2) = 0; Matrix(3, 3) = 1; Matrix(3, 4) = 0;
// 	Matrix(4, 0) = 0; Matrix(4, 1) = 0; Matrix(4, 2) = 0; Matrix(4, 3) = 0; Matrix(4, 4) = 1;

// 	return Matrix;
// }

// Eigen::MatrixXd ExtendKalman::ctrv_sensor(Eigen::VectorXd &x_p_k) {
// 	Eigen::MatrixXd Matrix(2, 5);
// 	Matrix(0, 0) = 1; Matrix(0, 1) = 0; Matrix(0, 2) = 0; Matrix(0, 3) = 0;
// 	Matrix(0, 4) = 0; Matrix(1, 0) = 0; Matrix(1, 1) = 1; Matrix(1, 2) = 0;
// 	Matrix(1, 3) = 0; Matrix(1, 4) = 0; 
    
//     	Matrix = Matrix * x_p_k;

// 	return Matrix;
// }

// Eigen::MatrixXd ExtendKalman::df_ctrv_sensor(Eigen::VectorXd &x_p_k) {
// 	(void) x_p_k;
// 	Eigen::MatrixXd Matrix(2, 5);
// 	Matrix(0, 0) = 1; Matrix(0, 1) = 0; Matrix(0, 2) = 0; Matrix(0, 3) = 0;
// 	Matrix(0, 4) = 0; Matrix(1, 0) = 0; Matrix(1, 1) = 1; Matrix(1, 2) = 0;
// 	Matrix(1, 3) = 0; Matrix(1, 4) = 0;

// 	return Matrix;
// }

// Eigen::MatrixXd ExtendKalman::se_df_ctrv_sensor(Eigen::VectorXd &x_p_k) {
// 	(void) x_p_k;
// 	Eigen::MatrixXd Matrix(2, 2);
// 	Matrix(0, 0) = 1; Matrix(0, 1) = 0;
// 	Matrix(1, 0) = 0; Matrix(1, 1) = 1;
// 	return Matrix;
// }


