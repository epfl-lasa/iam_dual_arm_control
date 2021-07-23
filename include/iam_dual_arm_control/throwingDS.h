/*
 * Copyright (C) 2018 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Authors: Mahdi Khoramshahi and Nadia Figueroa
 * email:   {mahdi.khoramshahi,nadia.figueroafernandez}@epfl.ch
 * website: lasa.epfl.ch
 *
 * This work was supported by the European Communitys Horizon 2020 Research and Innovation 
 * programme ICT-23-2014, grant agreement 644727-Cogimon and 643950-SecondHands.
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef __THROWING_DS_H__
#define __THROWING_DS_H__

#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <stdio.h>

#include <vector>
#include <mutex>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"

#include "Utils.hpp"

typedef Eigen::Matrix<float, 6, 1> Vector6f;

struct tossDsParam{
	float modulRegion[3]; 	// modulation parameters: distances [0] = radial, [1] = normal, [2] = tangent
	Eigen::Matrix3f Kp[3];  // Stiffness gains for position [0] : reaching  [1]: tossing [2]: retraction 
	Eigen::Matrix3f Dp[3]; 	// Damping gains   for position [0] : reaching  [1]: tossing [2]: retraction 
	Eigen::Matrix3f Ko[3]; 	// Stiffness gains for orienation [0] : reaching  [1]: tossing [2]: retraction 
	Eigen::Matrix3f Do[3]; 	// Damping gains   for orienation [0] : reaching  [1]: tossing [2]: retraction 
	bool is2ndOrder;				// boolean for the type of DS  True: second order, false: first order
};


struct tossingTaskVariables{

	Eigen::Vector3f release_position;
	Eigen::Vector4f release_orientation;
	Eigen::Vector3f release_linear_velocity;
	Eigen::Vector3f release_angular_velocity;
	Eigen::Vector3f rest_position;
	Eigen::Vector4f rest_orientation;
};


class throwingDS
{
	public:
		// TASK ID: Reaching, tossing, going to retract
	    enum TASK_ID {REACH = 0, TOSS = 1, RETRACT = 2};
	  // MOTION ID
	    enum MOTION_ID {TRANSLATION = 0, ROTATION = 1};


	private:

		// nominal DS
		Eigen::Matrix3f Kp_[3];  // Stifness gains for position of reaching task    [0], modulated motion task, [1] go to retract task [2]
		Eigen::Matrix3f Dp_[3];  // Damping gains  for position of reaching task    [0], modulated motion task, [1] go to retract task [2]
		Eigen::Matrix3f Ko_[3];  // Stifness gains for orientation of reaching task [0], modulated motion task, [1] go to retract task [2]
		Eigen::Matrix3f Do_[3];  // Damping gains  for orientation of reaching task [0], modulated motion task, [1] go to retract task [2]
		//
		float rho_;
		float range_norm_;
		float range_tang_;

		float sw_proxim_;
		float sw_norm_;
		float sw_tang_;

		bool is2ndOrder_;

		Eigen::Matrix4f w_H_de_; 
		Eigen::Matrix4f w_H_re_; 
		Eigen::Matrix4f w_H_po_;	
		// Vector6f Vee_;
		Eigen::Matrix3f BasisQp_;
		Eigen::Matrix3f BasisQo_;
		//
		Vector6f V_ee_d_;
		Vector6f A_ee_d_;
		bool release_flag_;




	public:
		//
		Eigen::Vector3f v_toss_;
		Eigen::Vector3f w_toss_;
		//
		tossDsParam ds_param_;
		//
		float a_proximity_;
		float a_normal_;
		float a_tangent_;
		float a_retract_;
		float coupling_;
		float _refVtoss;
		float a_toss_;
		float _v_max;
		float _w_max;
		bool _stop_and_toss = false;


		throwingDS();
		~throwingDS();

		bool init(float modulRegion[], Eigen::Matrix3f Kp[], Eigen::Matrix3f Dp[], Eigen::Matrix3f Ko[], Eigen::Matrix3f Do[], bool is2ndOrder);

		bool init(tossDsParam ds_param, Eigen::Vector3f releasePos, Eigen::Vector4f releaseOrient, Eigen::Vector3f releaseLinVel, Eigen::Vector3f releaseAngVel, 
								Eigen::Vector3f restPos, Eigen::Vector4f restOrient);

		/**
		 * @brief parameters for the throwing motion generation (2nd order)
		 * @param w_H_ce homogeneous transformation of current end effector pose
		 * @param Vee current velocity twist of the end-effector
		 * @param w_H_de homogeneous transformation of desired end effector pose
		 * @param w_H_re homogeneous transformation of rest end effector pose right after throwing
		 * @param Vdtoss desired throwing velocity of the end-effector (3x1)
		 * @param release_flag  a boolean flag to trigger the release of the object
		 */
		Vector6f generate_throwing_motion(Eigen::Matrix4f w_H_ce,  Vector6f Vee, Eigen::Matrix4f w_H_de, Eigen::Matrix4f w_H_re,  
									 								Eigen::Matrix3f BasisQ, Eigen::Vector3f Vdtoss, bool &release_flag);

		Eigen::Vector3f compute_modulated_motion(float activation, Eigen::Matrix3f BasisQ, Eigen::Vector3f Areach_ee, 
																							Eigen::Vector3f Amodul_ee_norm, Eigen::Vector3f Amodul_ee_tang);
		Eigen::Vector3f compute_angular_motion(float coupling, Eigen::Matrix4f w_H_c, Eigen::Vector3f Omega, 
																						Eigen::Matrix4f w_H_d, Eigen::Matrix3f Ko, Eigen::Matrix3f Do, bool is2ndOrder);
		Eigen::MatrixXf createOrthonormalMatrixFromVector(Eigen::VectorXf inVec);

		Vector6f apply(Eigen::Vector3f curPos, Eigen::Vector4f curOrient, Eigen::Vector3f curLinVel, Eigen::Vector3f curAngVel, int task_type);
		//
		bool set_gains(int taskId, int motionId, Eigen::Matrix3f K, Eigen::Matrix3f D);
		bool set_modulationParameters(float new_modul_param[]);
		bool set_toss_linear_velocity(Eigen::Vector3f newLinVel);
		bool set_toss_angular_velocity(Eigen::Vector3f newAngVel);
		bool set_toss_pose(Eigen::Vector3f new_releasePos, Eigen::Vector4f new_releaseOrient);
		bool set_rest_pose(Eigen::Vector3f new_restPos, Eigen::Vector4f new_restOrient);
		bool get_release_flag();
		bool set_pickup_object_pose(Eigen::Vector3f pickup_Pos, Eigen::Vector4f pickup_Orient);
};

#endif
