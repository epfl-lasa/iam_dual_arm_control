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

		Eigen::Matrix3f gain_p_abs;
		Eigen::Matrix3f gain_o_abs;
		Eigen::Matrix3f gain_p_rel;
		Eigen::Matrix3f gain_o_rel;
		float _cpl_grasp;

    float reachable_p;
    float _v_max;
    float _w_max;

    float _coord_abs2;
    float _cpl_rel;
		float _cp_ap;

    Eigen::Vector4f qdPrev[NB_ROBOTS];
    Matrix6f _Tbi;

	public :
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
		float _refVreach; //[NB_ROBOTS];
		float _refVtoss_EE;
		bool _modulated_reaching = true;
		bool _isNorm_impact_vel  = false;
		float _height_via_point = 0.25f;
		//
		dualArmFreeMotionController();
		~dualArmFreeMotionController();

		bool init(Eigen::Matrix4f w_H_eeStandby[], Matrix6f gain_abs_, Matrix6f gain_rel_);
		void computeCoordinatedMotion(Eigen::Matrix4f w_H_ee[],  Eigen::Matrix4f w_H_gp[], Eigen::Matrix4f w_H_o, Vector6f (&Vd_ee)[NB_ROBOTS], Eigen::Vector4f (&qd)[NB_ROBOTS], bool isOrient3d);
		void computeAsyncMotion(Eigen::Matrix4f w_H_ee[],  Eigen::Matrix4f w_H_gp[], Eigen::Matrix4f w_H_o, Vector6f (&Vd_ee)[NB_ROBOTS], Eigen::Vector4f (&qd)[NB_ROBOTS], bool isOrient3d);
		void computeDesiredOrientation(float weight, Eigen::Matrix4f w_H_ee[],  Eigen::Matrix4f w_H_gp[], Eigen::Matrix4f w_H_o, Eigen::Vector4f (&_qd)[NB_ROBOTS], bool isOrient3d);

		void computeConstrainedMotion(Eigen::Matrix4f w_H_ee[],  Eigen::Matrix4f w_H_gp[], Eigen::Matrix4f w_H_o, Vector6f (&Vd_ee)[NB_ROBOTS], Eigen::Vector4f (&qd)[NB_ROBOTS], bool isOrient3d);
		void computeReleaseAndRetractMotion(Eigen::Matrix4f w_H_ee[],  Eigen::Matrix4f w_H_gp[], Eigen::Matrix4f w_H_o, Vector6f (&Vd_ee)[NB_ROBOTS], Eigen::Vector4f (&qd)[NB_ROBOTS], bool isOrient3d);
		void generatePlacingMotion(	Eigen::Matrix4f w_H_ee[],  Eigen::Matrix4f w_H_gp[], Eigen::Matrix4f w_H_o, Eigen::Matrix4f w_H_Do, float via_height, 
																Vector6f (&Vd_ee)[NB_ROBOTS], Eigen::Vector4f (&qd)[NB_ROBOTS], bool isOrient3d);

		void computeCoordinatedMotion2(Eigen::Matrix4f w_H_ee[],  Eigen::Matrix4f w_H_gp[], Eigen::Matrix4f w_H_o, Vector6f (&Vd_ee)[NB_ROBOTS], Eigen::Vector4f (&qd)[NB_ROBOTS], bool isOrient3d);

		Eigen::Vector3f compute_modulated_motion(float activation, Eigen::Matrix3f BasisQ, Eigen::Vector3f Areach_ee, Eigen::Vector3f Amodul_ee_norm, Eigen::Vector3f Amodul_ee_tang);

		Vector6f compute_modulated_motion_dual(float activation, Eigen::Matrix3f BasisQ[], Vector6f DS_ee_nominal, Vector6f Amodul_ee_norm, Vector6f Amodul_ee_tang);

		void dual_arm_motion(Eigen::Matrix4f w_H_ee[],  Vector6f Vee[], Eigen::Matrix4f w_H_gp[],  Eigen::Matrix4f w_H_o, Eigen::Matrix4f w_H_Do, Vector6f Vd_o,
                                                  Eigen::Matrix3f BasisQ[], Eigen::Vector3f VdImp[], bool isOrient3d, int taskType, Vector6f (&Vd_ee)[NB_ROBOTS], Eigen::Vector4f (&qd)[NB_ROBOTS], bool &release_flag);

		Eigen::Vector3f getAbsoluteTangentError(Eigen::Matrix4f w_H_o, Eigen::Matrix4f w_H_ee[], Eigen::Matrix4f w_H_gp[]);

		Vector6f generatePlacingMotion2(Eigen::Matrix4f w_H_o, Eigen::Matrix4f w_H_Do, float via_height, Vector6f Vo);

};

#endif // dualArmFreeMotionController_H




