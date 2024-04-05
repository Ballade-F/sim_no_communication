

#include "kalman_interface.hpp"

class KalmanFilter : public KalmanInterface {
    public:
        KalmanFilter(int dim_x, int dim_z, int dim_u) 
            : KalmanInterface(dim_x, dim_z, dim_u) 
		{
            if (dim_u > 0) 
                B = Eigen::MatrixXd::Zero(dim_x, dim_u);
            else
                B = Eigen::MatrixXd::Zero(dim_x, dim_x);
            	H = Eigen::MatrixXd::Zero(dim_z, dim_x);
        }

        void init(Eigen::VectorXd &x_k) override;

        Eigen::VectorXd predict(Eigen::VectorXd &u, double t) override;
    
        Eigen::VectorXd update(Eigen::VectorXd &z_k) override;

    public:
        Eigen::MatrixXd B;  //  control quantity transfer matrix
        Eigen::MatrixXd H;  //  dimensional transformation matrix
};



