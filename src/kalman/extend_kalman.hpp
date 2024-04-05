

#include "kalman_interface.hpp"


const double KALMAN_DEC = M_PI / 180;

class ExtendKalman : public KalmanInterface {
    public:

        ExtendKalman(const Eigen::Matrix<double,2,2> Q_, const Eigen::Matrix<double,2,2>  R_) 
            : KalmanInterface(5, 2, 1,Q_,R_) 
		{

        }
        
        void init(Eigen::VectorXd &x_k) override;
        Eigen::VectorXd predict(Eigen::VectorXd &u, double t) override;
        Eigen::VectorXd update(Eigen::VectorXd &z_k) override;

        // // The following functions are for reference only
        // Eigen::MatrixXd ctrv(Eigen::VectorXd &x_l_k, Eigen::VectorXd &u, double t);
        // Eigen::MatrixXd df_ctrv(Eigen::VectorXd &x_l_k, Eigen::VectorXd &u, double t);
        // Eigen::MatrixXd se_df_ctrv(Eigen::VectorXd &x_l_k, Eigen::VectorXd &u, double t);
        // Eigen::MatrixXd ctrv_sensor(Eigen::VectorXd &x_p_k);
        // Eigen::MatrixXd df_ctrv_sensor(Eigen::VectorXd &x_p_k);
        // Eigen::MatrixXd se_df_ctrv_sensor(Eigen::VectorXd &x_p_k);


};
