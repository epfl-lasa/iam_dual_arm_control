#pragma once

#ifndef dualArmFreeMotionController_H
#define dualArmFreeMotionController_H


#include <iostream>
#include <iomanip>
#include <fstream>
#include <pthread.h>
#include <vector>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"

#include "throwingDS.h"

#define NB_ROBOTS 2                  // Number of robots

typedef Eigen::Matrix<float, 7, 1> Vector7f;
typedef Eigen::Matrix<float, 6, 1> Vector6f;
typedef Eigen::Matrix<float, 6, 6> Matrix6f;

class dualArmFreeMotionController
{

	public :
		// Robot ID: left or right
	    enum ROBOT {LEFT = 0, RIGHT = 1};

	protected: 

	
		Vector6f _error_abs;
		Vector6f _error_rel;
		Vector6f _error_obj;
		Vector6f _V_abs;
		Vector6f _V_rel;
		Vector6f _V_obj;
		// Vector6f _Vd_ee[NB_ROBOTS];		// desired velocity twist

		
		float _cpl_grasp;


    float _coord_abs2;
    float _cpl_rel;
		float _cp_ap;

    Eigen::Vector4f qdPrev[NB_ROBOTS];
    

    Vector6f 				 _Twist_vo;
		Eigen::Matrix4f  _w_H_vo;
		Eigen::Matrix4f  _w_H_vgp[NB_ROBOTS];
		Eigen::Vector3f  _xvgp_o[NB_ROBOTS];
		Eigen::Matrix4f  _qvgp_o[NB_ROBOTS];

	public :

		Eigen::Matrix3f gain_p_abs;
		Eigen::Matrix3f gain_o_abs;
		Eigen::Matrix3f gain_p_rel;
		Eigen::Matrix3f gain_o_rel;
		Matrix6f _Tbi;

		float _v_max;
    	float _w_max;

		//
		float _dt;
		float reachable_p;
		float _go2object;
		//
		float a_proximity_;
		float a_normal_;
		float a_tangent_;
		float a_retract_;
		float a_release_;
		float v_a_release_;
		bool  release_flag_;
		//
		float rho_;
		float range_norm_;
		float range_tang_;

		float sw_proxim_;
		float sw_norm_;
		float sw_tang_;
		//
		float a_normal_Do_;
		//
		Eigen::Matrix4f _w_H_eeStandby[NB_ROBOTS];
		//
		float _desVreach;
		float _refVreach[NB_ROBOTS];
		float _refVtoss_EE;
		bool _modulated_reaching = true;
		bool _isNorm_impact_vel  = false;
		float _height_via_point = 0.25f;
		int _smoothcount;
		//
		float _sw_EE_obsAv;
		float _min_dist_EE;
		float _safe_radius;
		Eigen::Vector3f Omega_object_d_;
		Eigen::Vector3f _integral_Vee_d[NB_ROBOTS];

		Eigen::Vector3f _objectDim;
		float _alpha_obs[NB_ROBOTS];
		float _activationAperture;
		Vector6f _Vd_o;
		
		//
		dualArmFreeMotionController();
		~dualArmFreeMotionController();

		bool init(Eigen::Matrix4f w_H_eeStandby[], Matrix6f gain_abs_, Matrix6f gain_rel_);
		void computeCoordinatedMotion(Eigen::Matrix4f w_H_ee[],  Eigen::Matrix4f w_H_gp[], Eigen::Matrix4f w_H_o, Vector6f (&Vd_ee)[NB_ROBOTS], Eigen::Vector4f (&qd)[NB_ROBOTS], bool isOrient3d);
		void computeAsyncMotion(Eigen::Matrix4f w_H_ee[],  Eigen::Matrix4f w_H_gp[], Eigen::Matrix4f w_H_o, Vector6f (&Vd_ee)[NB_ROBOTS], Eigen::Vector4f (&qd)[NB_ROBOTS], bool isOrient3d);
		void computeDesiredOrientation(float weight, Eigen::Matrix4f w_H_ee[],  Eigen::Matrix4f w_H_gp[], Eigen::Matrix4f w_H_o, Eigen::Vector4f (&_qd)[NB_ROBOTS], bool isOrient3d);

		void computeConstrainedMotion(Eigen::Matrix4f w_H_ee[],  Eigen::Matrix4f w_H_gp[], Eigen::Matrix4f w_H_o, Vector6f (&Vd_ee)[NB_ROBOTS], Eigen::Vector4f (&qd)[NB_ROBOTS], bool isOrient3d);
		void computeReleaseAndRetractMotion(Eigen::Matrix4f w_H_ee[],  Eigen::Matrix4f w_H_gp[], Eigen::Matrix4f w_H_o, Vector6f (&Vd_ee)[NB_ROBOTS], Eigen::Vector4f (&qd)[NB_ROBOTS], bool isOrient3d);
		

		void computeCoordinatedMotion2(Eigen::Matrix4f w_H_ee[],  Eigen::Matrix4f w_H_gp[], Eigen::Matrix4f w_H_o, Vector6f (&Vd_ee)[NB_ROBOTS], Eigen::Vector4f (&qd)[NB_ROBOTS], bool isOrient3d);

		Eigen::Vector3f compute_modulated_motion(float activation, Eigen::Matrix3f BasisQ, Eigen::Vector3f Areach_ee, Eigen::Vector3f Amodul_ee_norm, Eigen::Vector3f Amodul_ee_tang);

		Vector6f compute_modulated_motion_dual(float activation, Eigen::Matrix3f BasisQ[], Vector6f DS_ee_nominal, Vector6f Amodul_ee_norm, Vector6f Amodul_ee_tang);

		void dual_arm_motion(Eigen::Matrix4f w_H_ee[],  Vector6f Vee[], Eigen::Matrix4f w_H_gp[],  Eigen::Matrix4f w_H_o, Eigen::Matrix4f w_H_Do, Vector6f Vd_o,
                                                  Eigen::Matrix3f BasisQ[], Eigen::Vector3f VdImp[], bool isOrient3d, int taskType, Vector6f (&Vd_ee)[NB_ROBOTS], Eigen::Vector4f (&qd)[NB_ROBOTS], bool &release_flag);

		Eigen::Vector3f getAbsoluteTangentError(Eigen::Matrix4f w_H_o, Eigen::Matrix4f w_H_ee[], Eigen::Matrix4f w_H_gp[]);

		void generatePlacingMotion(	Eigen::Matrix4f w_H_ee[],  Eigen::Matrix4f w_H_gp[], Eigen::Matrix4f w_H_o, Eigen::Matrix4f w_H_Do, float via_height, 
																Vector6f (&Vd_ee)[NB_ROBOTS], Eigen::Vector4f (&qd)[NB_ROBOTS], bool isOrient3d);

		Vector6f generatePlacingMotion2(Eigen::Matrix4f w_H_o, Eigen::Matrix4f w_H_Do, float via_height, Vector6f Vo, bool isPlaceTossing);

		Vector6f generatePlacingMotion3(Eigen::Matrix4f w_H_o, Eigen::Matrix4f w_H_Do, float via_height, Vector6f Vo);

		void compute_EE_avoidance_velocity(Eigen::Matrix4f w_H_ee[], Vector6f (&VEE_oa)[NB_ROBOTS]);

		void constrained_ang_vel_correction(Eigen::Matrix4f w_H_ee[], Eigen::Matrix4f w_H_gp[], Eigen::Matrix4f w_H_o, Eigen::Matrix4f w_H_Do, Vector6f (&VEE)[NB_ROBOTS], bool wIntegral);

		void computeCoordinatedMotion3(Eigen::Matrix4f w_H_ee[],  Eigen::Matrix4f w_H_gp[], Eigen::Matrix4f w_H_o, Vector6f Vo, Eigen::Vector3f _x_intercept, 
                                                            Vector6f (&Vd_ee)[NB_ROBOTS], Eigen::Vector4f (&qd)[NB_ROBOTS], bool isOrient3d);
		void set_virtual_object_frame(Eigen::Matrix4f w_H_vo);

		void updateDesiredGraspingPoints(bool no_dual_mds_method, 
                                      bool isPlacing,
                                      bool isThrowing,
                                      bool isClose2Release,
                                      Eigen::Vector3f xgp_o[],
                                      Eigen::Vector4f qgp_o[],
                                      Eigen::Matrix4f o_H_ee[],
                                      Eigen::Matrix4f w_H_o,
                                      Eigen::Matrix4f &w_H_Do,
                                      Eigen::Vector3f xDo_placing,
                                      Eigen::Vector4f qDo_placing,
                                      Eigen::Vector3f release_position,
                                      Eigen::Vector4f release_orientation,
                                      Eigen::Matrix4f &w_H_DesObj,
                                      Eigen::Matrix4f (&w_H_gp)[NB_ROBOTS],
                                      Eigen::Matrix4f (&w_H_Dgp)[NB_ROBOTS]);


		void getDesiredMotion(bool no_dual_mds_method,
                          bool isContact, 
                          bool isPlacing,
                          bool isThrowing,
                          bool isClose2Release,
                          int dualTaskSelector,
                          Eigen::Matrix4f w_H_ee[], 
                          Eigen::Vector3f xgp_o[],
                          Eigen::Vector4f qgp_o[],
                          Eigen::Matrix4f o_H_ee[],
                          Eigen::Matrix4f w_H_o, 
                          Eigen::Matrix4f &w_H_Do,
                          Eigen::Vector3f xDo_placing,
                          Eigen::Vector4f qDo_placing,
                          Eigen::Vector3f release_position,
                          Eigen::Vector4f release_orientation,
                          float height_via_point,
                          Vector6f Vee[],
                          Vector6f Vd_o,
                          Eigen::Matrix3f BasisQ[],
                          Eigen::Vector3f VdImpact[],
                          Eigen::Matrix4f (&w_H_Dgp)[NB_ROBOTS],
                          Vector6f (&Vd_ee)[NB_ROBOTS], 
                          Eigen::Vector4f (&qd)[NB_ROBOTS], 
                          bool &release_flag);


		Eigen::Vector2f estimateRobot_PathLength_AverageSpeed(throwingDS &dsThrowing,
																													bool no_dual_mds_method, 
																													bool isPlacing, 
																													bool isThrowing, 
																													int dualTaskSelector, 
																													float dt,
																													float desVimp,
																													float tolerance_dist2contact,
																													float height_via_point,
																													Eigen::Vector3f xDo_placing,
																													Eigen::Vector4f qDo_placing,
																													Eigen::Vector3f release_position,  
																		                      Eigen::Vector4f release_orientation,
																		                      Eigen::Vector3f xgp_o[],
																													Eigen::Vector4f qgp_o[],
																													Eigen::Vector3f VdImpact[],
																													Eigen::Matrix3f BasisQ[], 
																													Eigen::Matrix4f w_H_Do,
																													Eigen::Matrix4f w_H_o,
																													Eigen::Matrix4f w_H_Dgp[],
																													Eigen::Matrix4f w_H_ee[]);

		void getCoordinatedTranslation(	Eigen::Vector3f x_ee[],  
                                    Eigen::Vector3f x_gp[], 
                                    Eigen::Vector3f x_std[],
                                    Eigen::Matrix3f w_R_o, 
                                    Eigen::Vector3f (&vd_ee)[NB_ROBOTS]);

		Eigen::Vector2f predictRobotTranslation(	Eigen::Matrix4f w_H_ee[],  
						                                  Eigen::Matrix4f w_H_gp[], 
						                                  Eigen::Matrix4f w_H_eeStandby[], 
						                                  Eigen::Matrix4f w_H_o,
						                                  Eigen::Vector3f x_release,
						                                  float vtoss,
						                                  float tolerance_dist2contact,
						                                  float dt,
						                                  float speedScaling);

		Eigen::Vector3f boost_ang_velocity(const Eigen::Vector3f& tmp_omega, float maxDq, float oriGainMx_);
		Eigen::Vector3f compute_desired_angular_velocity(Eigen::Vector4f quat, Eigen::Vector4f quat_d, Eigen::Matrix3f gain_o);
		Eigen::MatrixXf get_bimanual_grasp_mx(const Eigen::Matrix4f &w_H_o, Eigen::Matrix4f w_H_gp[]);
		Vector6f compute_desired_task_twist( const Eigen::Matrix4f &w_H_c, const Eigen::Matrix4f &w_H_d);

		Vector6f get_des_object_motion();

		void generateCShapeMotion(Eigen::Matrix4f w_H_ee[],  
                              Eigen::Matrix4f w_H_gp[], 
                              Eigen::Matrix4f w_H_o,
                              float via_height[], 
                              Vector6f (&Vd_ee)[NB_ROBOTS], 
                              Eigen::Vector4f (&qd)[NB_ROBOTS], 
                              bool isOrient3d);

		void computeFastCoordinatedMotion(Eigen::Matrix4f w_H_ee[],  Eigen::Matrix4f w_H_gp[], Eigen::Matrix4f w_H_o, 
																			Vector6f (&Vd_ee)[NB_ROBOTS], Eigen::Vector4f (&qd)[NB_ROBOTS], bool isOrient3d);

		void computeFastReleaseMotion(	Eigen::Matrix4f w_H_ee[],  Eigen::Matrix4f w_H_gp[], Eigen::Matrix4f w_H_o, 
																		Vector6f (&Vd_ee)[NB_ROBOTS], Eigen::Vector4f (&qd)[NB_ROBOTS], bool isOrient3d);

		Vector6f gen_directional_motion(const Eigen::Matrix4f &w_H_c, const Eigen::Matrix4f &w_H_d, const Eigen::Vector3f &des_dir);

};

#endif // dualArmFreeMotionController_H




