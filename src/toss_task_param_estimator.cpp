
#include "iam_dual_arm_control/toss_task_param_estimator.h"


// // function to compute pseudo inverse of a maxtrix
// // Compute the pseudo inverse of a matrix
// template<typename _Matrix_Type_> _Matrix_Type_ get_pseudoInverse(const _Matrix_Type_ &a, float epsilon = std::numeric_limits<float>::epsilon())
// {
    
//     Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);

//     int svdSize = svd.singularValues().size();

//     T tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);

//     return svd.matrixV().leftCols(svdSize) *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().leftCols(svdSize).adjoint();
// }

//
toss_task_param_estimator::toss_task_param_estimator(){}

toss_task_param_estimator::~toss_task_param_estimator(){}


void toss_task_param_estimator::init(std::string file_gmm[], Eigen::Vector3f x_release, Eigen::Vector4f q_release, Eigen::Vector3f v_release, Eigen::Vector3f w_release){

	x_release_ = x_release;
	q_release_ = q_release;
	v_release_ = v_release;
	w_release_ = w_release;

	datalog.Load_gmm_param2(file_gmm, Prior_gmm_toss_, Mean_gmm_toss_, CovMx_gmm_toss_);
}

void toss_task_param_estimator::cartesian2planar(Eigen::Vector3f Pos_3d_in, Eigen::Vector2f &pos_bar, float &phi_d, float &theta_d)
{
	//Coordinates transformations
    //convert cartesian into planar data using cylindrical coordinates
    pos_bar(0) = Pos_3d_in.head(2).norm();
    pos_bar(1) = Pos_3d_in(2);

    phi_d   = std::atan2(Pos_3d_in(1), Pos_3d_in(0));
    // direction of desired positions
    theta_d = std::atan2(pos_bar(1), pos_bar(0));
}

//
void toss_task_param_estimator::get_min_release_speed(Eigen::Vector2f pos_d, float &ang_release_i, float &v_release_i, float &t_flight_i)
{

	float theta_d = std::atan2(pos_d(1), pos_d(0));
    // function to estimate the minimum release velocity based of an known release angle
   	float theta_i = std::atan(pos_d(1)/pos_d(0) + std::sqrt((pos_d(1)/pos_d(0))*(pos_d(1)/pos_d(0)) + 1.f));

   	if(theta_i > M_PI/2.f){
   		    theta_i = (90.f-5.f)/180.f * M_PI;
   	}
	else if (theta_i < theta_d){
		    theta_i = theta_d + 5.f/180.f * M_PI;
	}
	
    float delta_r = pos_d(0);
    float delta_z = pos_d(1);
    //
    float num = (this->g * delta_r*delta_r * (1.f+std::tan(theta_i)*std::tan(theta_i)));
    float den = (2.f * (delta_r * std::tan(theta_i) - delta_z));
    // get the initial velocity in (m/s)
    v_release_i = fabs(std::sqrt(num / den));

    // 
    t_flight_i = fabs((1./this->g)*(v_release_i*std::sin(theta_i) + std::sqrt(std::sin(theta_i)*std::sin(theta_i) + 2.f*this->g*delta_z)));
}

//
void toss_task_param_estimator::estimate_2d_throwing_param(ProjectileType type, Eigen::Vector2f pos_d,  
															float &ang_release_i, float &v_release_i, float &t_flight_i)
{
	//
	if(type == PHYS_IDEAL){
		this->get_min_release_speed(pos_d, ang_release_i, v_release_i, t_flight_i);
	}
	else if(type == PHYS_WITH_DRAG){
		this->get_min_release_speed(pos_d, ang_release_i, v_release_i, t_flight_i);	// TO DO replace with appropriate function
	}
	else{ //(LEARNED)

		float ang_release, v_release;
		this->get_min_release_speed(pos_d, ang_release, v_release, t_flight_i);	// TO DO replace with appropriate function
		//
		Eigen::MatrixXf E_Cov_I;
		Eigen::MatrixXf E_Cov_O;
		Eigen::VectorXf theta_vel_release;

		this->get_GMR_IO(this->Prior_gmm_toss_, this->Mean_gmm_toss_, this->CovMx_gmm_toss_, pos_d,  theta_vel_release, E_Cov_I, E_Cov_O);
		ang_release_i = theta_vel_release(0);
		v_release_i   = theta_vel_release(1);
	}

	//

}

//
bool toss_task_param_estimator::estimate_tossing_param(ProjectileType type, Eigen::Vector3f Pos_landing, Eigen::Vector3f Pos_Release){
	//
	Eigen::Vector3f Pos_3d_in = Pos_landing - Pos_Release;

	Eigen::Vector2f pos_bar;  
	float ang_release_i; 
	float v_release_i; 
	float t_flight_i;
	float theta_d;
	float phi_d;

	//
	this->cartesian2planar(Pos_3d_in, pos_bar, phi_d, theta_d); 							  // in: Pos_3d_in --> out : pos_bar, phi_d, theta_d

	this->estimate_2d_throwing_param(type, pos_bar,  ang_release_i, v_release_i, t_flight_i); // --> out: ang_release_i, v_release_i, t_flight_i

	//
	this->x_release_    = Pos_Release;
	this->v_release_(0) = v_release_i*cos(ang_release_i)*cos(phi_d);
	this->v_release_(1) = v_release_i*cos(ang_release_i)*sin(phi_d);
	this->v_release_(2) = v_release_i*sin(ang_release_i);

}

Eigen::Vector3f toss_task_param_estimator::get_release_position()
{
	return x_release_;
}
Eigen::Vector4f toss_task_param_estimator::get_release_orientation()
{
	return q_release_;
}
Eigen::Vector3f toss_task_param_estimator::get_release_linear_velocity()
{
	return v_release_;
}
Eigen::Vector3f toss_task_param_estimator::get_release_angular_velocity()
{
	return w_release_;
}


float toss_task_param_estimator::gaussPDF(Eigen::VectorXf Data, Eigen::VectorXf Mu, Eigen::MatrixXf Sigma)
{
	Eigen::VectorXf delta_x = Data - Mu;
	float x_mu = delta_x.transpose() * Psdinv.pseudoInverse(Sigma) * delta_x;

	return exp(-0.5f*x_mu)/std::sqrt(std::pow(2*M_PI, Data.rows()) * (std::fabs(Sigma.determinant()) + 1e-30));
}


bool toss_task_param_estimator::get_GMR_IO(Eigen::VectorXf Prior, Eigen::MatrixXf Mean_gmm, Eigen::MatrixXf CovMx, Eigen::VectorXf Input_, 
										   Eigen::VectorXf &Output_, Eigen::MatrixXf &E_Cov_I, Eigen::MatrixXf &E_Cov_O)
{
	// Extract the number of Gaussian
	int nStates = Prior.rows();
	// Extract dimension of q_star and xi_star
	int nI = Input_.rows();
	int nO = Mean_gmm.rows() - nI;

	Output_.resize(nO);
	Output_.setZero();
	//
	E_Cov_I.resize(nI, nI);		    E_Cov_I.setZero();	// Expected Input covariance q
	E_Cov_O.resize(nO, nO);			E_Cov_O.setZero();  // Expected Output covariance xi
	//
	Eigen::MatrixXf Cov_I_k(nI,nI);
	Eigen::MatrixXf Cov_O_k(nO,nO);
	Eigen::MatrixXf Cov_IO_k(nI,nO);
	Eigen::MatrixXf Cov_OI_k(nO,nI);

	// |  nI | nIO |  
	// |-----------|
	// | nOI | nO  | 

	
	// Computation of h
	Eigen::VectorXf h = Eigen::VectorXf::Zero(nStates);
	Eigen::VectorXf Normal_h = Eigen::VectorXf::Zero(nStates);
	//
	int dim = nI+nO;
	//
	for(int k=0; k<nStates; k++)
	{
		Cov_I_k = CovMx.block(k*dim,  0, nI, nI);
		//
		h(k) = Prior(k) * this->gaussPDF(Input_, Mean_gmm.col(k).head(nI), Cov_I_k);
	}

	Normal_h = h/h.sum();
	//
	Eigen::MatrixXf Cov_k(dim, dim);
	//
	for(int k=0; k<nStates; k++)
	{
		//
		Cov_k 	 = CovMx.block(k*dim, 0, dim, dim);
		//
		Cov_I_k  = Cov_k.block( 0,   0,  nI, nI);	//
		Cov_IO_k = Cov_k.block( 0,  nI,  nI, nO);   //
		Cov_OI_k = Cov_k.block(nI,   0,  nO, nI);	//
		Cov_O_k  = Cov_k.block(nI,  nI,  nO, nO);   //
		//
		// Posterior of q
		Eigen::VectorXf D_In_mu 			= Input_ - Mean_gmm.col(k).head(nI);

		Eigen::MatrixXf Sigma_OI_invSigma_I = Cov_OI_k * Psdinv.pseudoInverse(Cov_I_k);

		Output_ += Normal_h(k)*( Mean_gmm.col(k).tail(nO) + Sigma_OI_invSigma_I * D_In_mu );
		// Associated covaraince of Posterior q
		E_Cov_O += Normal_h(k)*Normal_h(k)*(Cov_O_k - Sigma_OI_invSigma_I * Cov_IO_k);
		//
		E_Cov_I += Normal_h(k)*Normal_h(k)*(Cov_I_k);		
	}

	return true;
}