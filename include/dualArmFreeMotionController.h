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

	Eigen::Matrix4f _w_H_eeStandby[NB_ROBOTS];
	Vector6f _error_abs;
	Vector6f _error_rel;
	Vector6f _V_abs;
	Vector6f _V_rel;
	// Vector6f _Vd_ee[NB_ROBOTS];		// desired velocity twist

	Eigen::Matrix3f gain_p_abs;
	Eigen::Matrix3f gain_o_abs;
	Eigen::Matrix3f gain_p_rel;
	Eigen::Matrix3f gain_o_rel;

    float reachable_p;
    float _v_max;
    float _w_max;

	public :
		//
		//
		dualArmFreeMotionController();
		~dualArmFreeMotionController();

		bool init(Eigen::Matrix4f w_H_eeStandby[], Matrix6f gain_abs_, Matrix6f gain_rel_);
		void computeCoordinatedMotion(Eigen::Matrix4f w_H_ee[],  Eigen::Matrix4f w_H_gp[], Vector6f (&Vd_ee)[NB_ROBOTS], Eigen::Vector4f (&qd)[NB_ROBOTS]);
		void computeAsyncMotion(Eigen::Matrix4f w_H_ee[],  Eigen::Matrix4f w_H_gp[], Vector6f (&Vd_ee)[NB_ROBOTS], Eigen::Vector4f (&qd)[NB_ROBOTS]);
		void computeDesiredOrientation(float weight, Eigen::Matrix4f w_H_ee[],  Eigen::Matrix4f w_H_gp[], Eigen::Vector4f (&_qd)[NB_ROBOTS]);

		void computeConstrainedMotion(Eigen::Matrix4f w_H_ee[],  Eigen::Matrix4f w_H_gp[], Vector6f (&Vd_ee)[NB_ROBOTS], Eigen::Vector4f (&qd)[NB_ROBOTS]);

};

#endif // dualArmFreeMotionController_H




