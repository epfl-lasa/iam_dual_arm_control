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

class toss_task_param_estimator
{
	private:

		Eigen::Vector3f xD_landing;		// Target postion 
		Eigen::Vector3f xD_release;		// Release position
		Eigen::Vector3f X3d_bar;		// 3d Relative position between landing and release position 
		Eigen::Vector2f x2d_bar;		// 2d (planar) relative position from X3d_bar (delta_r and delta_z)	
		float phi_d; 					// angle of horizontal direction (XY) of the throwing plane
		float theta_d;					// elevation angle of of relative position atan(z,r)

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

		// PHYS_IDEAL 		: physical model of ideal (point mass) projectile with no aerodynamic drag
		// PHYS_WITH_DRAG 	: physical model of projectile (ball) with Newton (aerodynamic) drag
		// LEARNED 			: learned model of projectile (ball) with Newton (aerodynamic) drag
		enum ProjectileType {PHYS_IDEAL = 0, PHYS_WITH_DRAG = 1, LEARNED = 2};

		toss_task_param_estimator();
		~toss_task_param_estimator();

		MatrixPseudoInverse Psdinv;

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

		float gaussPDF(Eigen::VectorXf Data, Eigen::VectorXf Mu, Eigen::MatrixXf Sigma);
		bool get_GMR_IO(Eigen::VectorXf Prior, Eigen::MatrixXf Mean_gmm, Eigen::MatrixXf CovMx, Eigen::VectorXf Input_, Eigen::VectorXf &Output_, Eigen::MatrixXf &E_Cov_I, Eigen::MatrixXf &E_Cov_O);

};

#endif // TOSS_TASK_PARAM_ESTIMATOR_H

