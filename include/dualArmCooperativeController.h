
#pragma once

#ifndef dualArmCooperativeController_H
#define dualArmCooperativeController_H


#include <iostream>
#include <iomanip>
#include <fstream>
#include <pthread.h>
#include <vector>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"

extern "C" {
#include "bwc_solver.h"
}

#define NB_ROBOTS 2                  // Number of robots

typedef Eigen::Matrix<float, 7, 1> Vector7f;
typedef Eigen::Matrix<float, 6, 1> Vector6f;
typedef Eigen::Matrix<float, 6, 6> Matrix6f;

class dualArmCooperativeController
{

	public :
		// Robot ID: left or right
	    enum ROBOT {LEFT = 0, RIGHT = 1};

	protected: 

	float _tol_dist2contact;
	float _dist2contact[NB_ROBOTS];
	float _min_Fz;
	float _mu_ee;
	float _gamma_ee;
	float _deltaX_ee;
	float _deltaY_ee;
	float _min_nF;
	float _max_nF;

	bool _withForceSaturation;
	bool _contactOccured;
	//
	// EE
	Eigen::Matrix<float,6,12> _GraspMatrixEEs;
	Matrix6f _world_Xstar_desEE[NB_ROBOTS];
	Vector6f _wrench_correction_ee[NB_ROBOTS];
	Eigen::Matrix<float, 1,6>  _complementaryConstraintMatrix[NB_ROBOTS];
	Eigen::Matrix<float,12,1>  _contactConstraintVector[NB_ROBOTS];
	Eigen::Matrix<float,11,6>  _contactConstraintMatrix[NB_ROBOTS];
	Vector6f _optimal_slack;

	Eigen::Matrix<double, 12, 1> _weight_EEs_wrench;
	Eigen::Matrix<double, 6, 1>  _weight_EEs_slack;

	public :
		float _ContactConfidence ;
		Eigen::Matrix<float,12,1>  _optimal_contact_wrench_EEs;
		Vector6f _f_applied[NB_ROBOTS];
		Vector6f _f_In_EE[NB_ROBOTS];
		//
		dualArmCooperativeController();
		~dualArmCooperativeController();

		bool init();
		void check_contact_proximity(Eigen::Matrix4f w_H_ee[], Eigen::Matrix4f w_H_cp[]);

		void getGraspKineDynVariables(Eigen::Matrix4f w_H_o, Eigen::Matrix4f w_H_ee[], Eigen::Matrix4f w_H_cp[]);
		bool computeBimanualGraspMatrix(Eigen::Matrix4f w_H_o, Eigen::Matrix4f w_H_ee[], Eigen::Matrix<float,6,12> &GraspMxHands_);
		void setMinNormalForcesEEs(float min, Eigen::Matrix3f w_R_h, Vector6f &Wrench_w);
		void thresholdNormalForcesEEs(float min, float max, Eigen::Matrix3f w_R_h, Vector6f &Wrench_w);
		void computeOptimalWrench(Vector6f desired_object_wrench_);
		bool getComplementaryConstraints(Matrix6f world_Xstar_desEE[], float dist2contact[], float tol_dist2contact);
		bool getContactConstraints(Matrix6f world_Xstar_EE[]);
		void load_wrench_data(Vector6f desired_object_wrench_);
		void computeControlWrench(Eigen::Matrix4f w_H_o, Eigen::Matrix4f w_H_ee[], Eigen::Matrix4f w_H_cp[], Vector6f desired_object_wrench_);
};

#endif // dualArmFreeMotionController_H




