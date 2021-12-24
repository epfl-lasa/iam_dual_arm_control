
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
	  // Contact state:
    // CONTACT: Both robots are in contact with the object
    // CLOSE_TO_CONTACT: Both robots are close to make contact with the object
    // NO_CONTACT: Both robots are not in contact with the object
    enum ContactState {CONTACT = 0, CLOSE_TO_CONTACT = 1, NO_CONTACT = 2};
    // Exection mode:
    // REACHING_GRASPING_ONLY: The two robots reach and grasp the object
    // REACHING_GRASPING_MANIPULATING: The two robots reach, grasp and move the object to a predefined position                               
    enum Mode {REACHING_GRASPING = 0, REACHING_GRASPING_MANIPULATING = 1};

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
	float _targetForce;

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
	//
	Eigen::Vector3f _nC[NB_ROBOTS];               					// Normal vector to surface object for each robot (3x1)

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
		void getPredefinedContactForceProfile(bool goHome, int contactState, Eigen::Matrix4f w_H_o, Eigen::Matrix4f w_H_ee[], Eigen::Matrix4f w_H_cp[]);
		// void getAppliedWrenches(bool goHome, int contactState, Eigen::Matrix4f w_H_o, Eigen::Matrix4f w_H_ee[], Eigen::Matrix4f w_H_cp[], Vector6f desired_object_wrench_, bool qp_wrench_generation);
		void getAppliedWrenches(bool goHome, int contactState, Eigen::Matrix4f w_H_o, Eigen::Matrix4f w_H_ee[], Eigen::Matrix4f w_H_cp[], Vector6f desired_object_wrench, float object_mass, bool qp_wrench_generation);
		void getPredefinedContactForceProfile(bool goHome, int contactState, Eigen::Matrix4f w_H_o, Eigen::Matrix4f w_H_ee[], Eigen::Matrix4f w_H_cp[], float object_mass);
};

#endif // dualArmFreeMotionController_H




