#ifndef TOSS_TASK_PARAM_ESTIMATOR_H
#define TOSS_TASK_PARAM_ESTIMATOR_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/SVD"

#include "iam_dual_arm_control/data_logging.hpp"

// struct ProjectileOption {
// 	Eigen::Vector3f dim_obj = Eigen::Vector3f(0.2, 0.2, 0.2);
// 	float rho_air 		= 1.29f;   // Air density (kg/m^3)
// 	float coeff_drag 	= 0.5f;    // Drag coefficient (spherical projectile)
// 	float gravity 		= 9.81f;
// 	float mass    		= 0.5f;    // object mass in kg
// 	Eigen::Matrix3f inertia  = Iner_obj;
// 	int max_iter 		= 500;
// 	float reach_tol		= 0.05f;
// }

typedef Eigen::Matrix<float, 6, 1> Vector6f;
typedef Eigen::Matrix<float, 7, 1> Vector7f;

class MatrixPseudoInverse
{

  public : 

      MatrixPseudoInverse(){}

      ~MatrixPseudoInverse(){}

      // Compute the pseudo inverse of a matrix
      template<typename _Matrix_Type_> _Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
      {
          
        Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);

        double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);

        return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
      }

};

class firstOrderFilter
{
    double Ts;

    // Eigen::VectorXf init_fn;
    Eigen::VectorXf init_fn2;
    Eigen::VectorXf init_fn3;
    Eigen::VectorXf init_fn4;
    Eigen::VectorXf delta1;
    Eigen::VectorXf delta2;
    Eigen::VectorXf delta3;
    Eigen::VectorXf delta4;
    Eigen::VectorXf y_t;

	public:

    float pole;
    float gain;
    Eigen::VectorXf init_fn;

    firstOrderFilter(){}
    // 
    void InitializeFilter(float T, float gn, float pl, Eigen::VectorXf init_fn_val)
    {
        Ts = T;
        gain = gn;
        pole = pl;

        init_fn.resize(init_fn_val.rows(), init_fn_val.cols());
        init_fn2.resize(init_fn_val.rows(), init_fn_val.cols());
        init_fn3.resize(init_fn_val.rows(), init_fn_val.cols());
        init_fn4.resize(init_fn_val.rows(), init_fn_val.cols());

        delta1.resize(init_fn_val.rows(), init_fn_val.cols());
        delta2.resize(init_fn_val.rows(), init_fn_val.cols());
        delta3.resize(init_fn_val.rows(), init_fn_val.cols());
        delta4.resize(init_fn_val.rows(), init_fn_val.cols());

        y_t.resize(init_fn_val.rows(), init_fn_val.cols());
        init_fn = init_fn_val;

    }

     ~firstOrderFilter(){}

    Eigen::VectorXf function_dot(float gn, float pl, const Eigen::VectorXf &init_fn_val, const Eigen::VectorXf &fn_t)
    {
        return - pl * init_fn_val + gn * fn_t;
    }

    // compute the integral of first order differential eq. using RK4
    Eigen::VectorXf getRK4Integral(const Eigen::VectorXf &fn_t)
    {
      delta1   = Ts * function_dot(gain, pole, init_fn, fn_t);
      init_fn2 = init_fn + 0.5 * delta1;
      delta2   = Ts * function_dot(gain, pole, init_fn2, fn_t);
      init_fn3 = init_fn + 0.5 * delta2;
      delta3   = Ts * function_dot(gain, pole, init_fn3, fn_t);
      init_fn4 = init_fn + 0.5 * delta3;
      delta4   = Ts * function_dot(gain, pole, init_fn4, fn_t);

      // solution
      y_t      = init_fn + 1/6. * (delta1 + 2.* delta2 + 2.* delta3 + delta4);
      init_fn  = y_t;

      return y_t;
    }

    Eigen::VectorXf getEulerIntegral(const Eigen::VectorXf &fn_t)
    {
      delta1   = Ts * function_dot(gain, pole, init_fn, fn_t);
      // solution
      y_t      = init_fn + delta1;
      init_fn  = y_t;

      return y_t;

    }

    void setGain(float _gain){
        gain = _gain;
    }

    void setPole(float _pole){
        pole = _pole;
    }

    void setSampleTime(float T){
        Ts = T;
    }   
};

class my_pdf_gmr
{
	private:
		Eigen::VectorXf Prior_gmm_toss_;
		Eigen::MatrixXf Mean_gmm_toss_;
		Eigen::MatrixXf CovMx_gmm_toss_;

	protected:
		data_logging  datalog;
		MatrixPseudoInverse Psdinv;

	public:
		my_pdf_gmr(){};
		~my_pdf_gmr(){};

		bool init(std::string file_gmm[]){
			//
			datalog.Load_gmm_param2(file_gmm, Prior_gmm_toss_, Mean_gmm_toss_, CovMx_gmm_toss_);
			return true;
		}

		float gaussPDF(Eigen::VectorXf Data, Eigen::VectorXf Mu, Eigen::MatrixXf Sigma){

			Eigen::VectorXf delta_x = Data - Mu;
			float x_mu = delta_x.transpose() * Psdinv.pseudoInverse(Sigma) * delta_x;

			return exp(-0.5f*x_mu)/std::sqrt(std::pow(2*M_PI, Data.rows()) * (std::fabs(Sigma.determinant()) + 1e-30));
		}

		bool get_GMR_IO(Eigen::VectorXf Prior, Eigen::MatrixXf Mean_gmm, Eigen::MatrixXf CovMx, Eigen::VectorXf Input_, Eigen::VectorXf &Output_, Eigen::MatrixXf &E_Cov_I, Eigen::MatrixXf &E_Cov_O){

			// Extract the number of Gaussian
			int nStates = Prior.rows();
			// Extract dimension of q_star and xi_star
			int nI = Input_.rows();
			int nO = Mean_gmm.rows() - nI;

			Output_.resize(nO);
			Output_.setZero();
			//
			E_Cov_I.resize(nI, nI);		  E_Cov_I.setZero();	// Expected Input covariance q
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

		bool compute_gmr_outputs(Eigen::VectorXf Input_, Eigen::VectorXf &Output_, Eigen::MatrixXf &E_Cov_I, Eigen::MatrixXf &E_Cov_O){

			this->get_GMR_IO(this->Prior_gmm_toss_, this->Mean_gmm_toss_, this->CovMx_gmm_toss_, Input_,  Output_, E_Cov_I, E_Cov_O);
			return true;
		}

		Eigen::VectorXf  get_gmm_vector_of_priors(){
			return Prior_gmm_toss_;
		}

		Eigen::MatrixXf get_gmm_matrix_of_means(){
			return Mean_gmm_toss_;
		} 
		Eigen::MatrixXf  get_gmm_matrix_of_covariant_matrices(){
			return CovMx_gmm_toss_;
		}

};


class toss_task_param_estimator
{
	private:

		Eigen::Vector3f xD_landing;		// Target postion 
		Eigen::Vector3f xD_release;		// Release position
		Eigen::Vector3f X3d_bar;			// 3d Relative position between landing and release position 
		Eigen::Vector2f x2d_bar;			// 2d (planar) relative position from X3d_bar (delta_r and delta_z)	
		float phi_d; 									// angle of horizontal direction (XY) of the throwing plane
		float theta_d;								// elevation angle of of relative position atan(z,r)

		Eigen::Vector3f x_release_;
		Eigen::Vector4f q_release_;
		Eigen::Vector3f v_release_;
		Eigen::Vector3f w_release_;

		float g = 9.81f;

		Eigen::VectorXf Prior_gmm_toss_;
		Eigen::MatrixXf Mean_gmm_toss_;
		Eigen::MatrixXf CovMx_gmm_toss_;

	protected:
		data_logging  datalog;

	public:

		// PHYS_IDEAL 			: physical model of ideal (point mass) projectile with no aerodynamic drag
		// PHYS_WITH_DRAG 	: physical model of projectile (ball) with Newton (aerodynamic) drag
		// LEARNED 					: learned model of projectile (ball) with Newton (aerodynamic) drag
		enum ProjectileType {PHYS_IDEAL = 0, PHYS_WITH_DRAG = 1, LEARNED = 2};

		toss_task_param_estimator();
		~toss_task_param_estimator();

		MatrixPseudoInverse Psdinv;
		my_pdf_gmr myGMR;
		firstOrderFilter myRK4;

		void init(std::string file_gmm[], Eigen::Vector3f x_release, Eigen::Vector4f q_release, Eigen::Vector3f v_release, Eigen::Vector3f w_release);
		// conversion of cartesian to planar coordinates
		void cartesian2planar(Eigen::Vector3f Pos_3d_in, Eigen::Vector2f &pos_bar, float &phi_d, float &theta_d);
		// compute min release velocity for the ideal projectile (point mass) no aerodynamic drag
		void get_min_release_speed(Eigen::Vector2f pos_d, float &ang_release_i, float &v_release_i, float &t_flight_i);
		void estimate_2d_throwing_param(ProjectileType type, Eigen::Vector2f pos_d, float &ang_release_i, float &v_release_i, float &t_flight_i);


		bool estimate_tossing_param(ProjectileType type, Eigen::Vector3f Pos_landing, Eigen::Vector3f Pos_Release);
		
		Eigen::Vector3f get_release_position();
		Eigen::Vector4f get_release_orientation();
		Eigen::Vector3f get_release_linear_velocity();
		Eigen::Vector3f get_release_angular_velocity();

		// float gaussPDF(Eigen::VectorXf Data, Eigen::VectorXf Mu, Eigen::MatrixXf Sigma);
		// bool get_GMR_IO(Eigen::VectorXf Prior, Eigen::MatrixXf Mean_gmm, Eigen::MatrixXf CovMx, Eigen::VectorXf Input_, Eigen::VectorXf &Output_, Eigen::MatrixXf &E_Cov_I, Eigen::MatrixXf &E_Cov_O);

		// 
		Eigen::Vector3f get_state_variation(float dt, Eigen::Vector3f X, Eigen::Vector3f Xd, Eigen::Vector3f dX_d); //  need to add ds
		Eigen::Vector3f cartesian2spherical(Eigen::Vector3f Pos_d); //
		Eigen::Vector3f cartesian2planar(Eigen::Vector3f Pos_d);		//
		Eigen::Vector4f ds_projectile2d(Eigen::Vector4f rz, float g, float mu); //
		void projectileMotion2d(float T, float g, float mu, Eigen::Vector2f pos_i, Eigen::Vector2f pos_d, float v0_i, float theta_i, float& flytime, Eigen::Vector2f& Xland2d);//
		Eigen::Vector2f estimateTarget_SimpPathLength_AverageSpeed(Eigen::Vector3f X, Eigen::Vector3f Xd, Eigen::Vector3f aVtarget); //
		Eigen::Vector2f estimate_robot_Lpath_avgVel(Eigen::Vector3f X, Eigen::Vector3f Xd);
		void estimate_landing_state(float T, float g, float mu, Vector7f Xd_land, Vector7f Xrelease, Eigen::Vector3f Vel_feas, float &flytime, Eigen::Vector3f &Xf_land); //
		// Eigen::Vector3f estimate_state_to_go(float T, float g, float mu, Vector7f X_bot, Vector7f Xrelease, Eigen::Vector3f Vel_feas, Eigen::Vector3f X_tgt, Eigen::Vector3f dX_tgt, Vector7f Xd_land);
		Eigen::Vector3f estimate_target_state_to_go(Eigen::Vector3f Xtarget, Eigen::Vector3f dX_target, Eigen::Vector3f Xintercept, Eigen::Vector2f Lp_Va_pred_bot, Eigen::Vector2f Lp_Va_pred_tgt, float flytime_obj);


};

#endif // TOSS_TASK_PARAM_ESTIMATOR_H

