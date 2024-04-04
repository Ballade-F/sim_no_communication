

#include "kalman_interface.hpp"



const double KALMAN_DEC = M_PI / 180;

class ExtendKalman : public KalmanInterface 
{
public:

	/**
	* @brief Construction of Extend Kalman filter class
	* 
	* @param dim_x dimension of state(x_k)
	* @param dim_z dimension of measurement(z_k)
	* @param dim_u dimension of control(u_k)
	*/
	ExtendKalman(int dim_x, int dim_z, int dim_u) 
		: KalmanInterface(dim_x, dim_z, dim_u) 
	{
		H = Eigen::MatrixXd::Zero(dim_z, dim_x);
	}

	/**
	* @brief Construction of Extend Kalman filter interface class
	* 
	* @param dim_x dimension of state(x_k)
	* @param dim_z dimension of measurement(z_k)
	* @param dim_u dimension of control(u_k)
	* @param Q process error covariance matrix
	* @param R measurement error covariance matrix
	*/
	ExtendKalman(int dim_x, int dim_z, int dim_u, const Eigen::MatrixXd Q, 
		const Eigen::MatrixXd R) : ExtendKalman(dim_x, dim_z, dim_u) 
	{
		this->Q = Q;
		this->R = R;
	}

	/**
	* @brief initialization of extend kalman filter(override)
	*
	* @param x_k initialization of state
	*/
	void init(Eigen::VectorXd &x_k) override;

	/**
	* @brief prediction of extend kalman filter(override)
	*
	* @param u controlling quantity
	* @param t step size
	*/
	Eigen::VectorXd predict(Eigen::VectorXd &u, double t) override;

	/**
	* @brief update of extend kalman filter(override)
	*
	* @param z_k measurement quantity
	*/      
	Eigen::VectorXd update(Eigen::VectorXd &z_k) override;

    public:
	Eigen::MatrixXd H;  //  dimensional transformation matrix
};


