/** Class RobotVariables

*/

#pragma once

#ifndef ROBOT_VARIABLES_H
#define ROBOT_VARIABLES_H

#include <vector>
#include <deque>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"
#include "sg_filter.h"
#include "Utils.hpp"

#define NB_ROBOTS 2                  // Number of robots
#define NB_FT_SENSOR_SAMPLES 50      // Number of force torque sensors' samples used for initial calibration (compute the offsets)
#define MOVING_FORCE_WINDOW_SIZE 10  // Window's size used to average the force data and detect peristent contact


class robot_var{

	public:

		// --------------------------------------------------------------------------------
		Eigen::Vector3f _gravity;
		int _nb_joints[NB_ROBOTS];
		float _toolMass[NB_ROBOTS];                         				// Tool mass [kg]
		float _toolOffsetFromEE[NB_ROBOTS];          								// Tool offset along z axis of end effector [m]   
		Eigen::Vector3f _toolComPositionFromSensor[NB_ROBOTS];      // Tool CoM offset along z axis of end effector [m]
		int _wrenchCount[NB_ROBOTS];                      					// Counter used to pre-process the force data
		std::deque<float> _normalForceWindow[NB_ROBOTS];  					// Moving window saving the robots' measured normal force to the object's surface [N]  
		float _normalForceAverage[NB_ROBOTS];             					// Average normal force measured through the force windows [N]
		float _normalForce[NB_ROBOTS];                    					// Normal force to the surface [N] 
		// float _c;                                        				// Contact value (1 = CONTACT, 0 otherwise)
		bool _wrenchBiasOK[NB_ROBOTS];                 							// Check if computation of force/torque sensor bias is OK
		// --------------------------------------------------------------------------------
		Eigen::Vector3f _x[NB_ROBOTS];
		Eigen::Vector4f _q[NB_ROBOTS];
		Eigen::Matrix3f _wRb[NB_ROBOTS]; 														// Orientation matrix (3x3)
		Eigen::Vector3f _xd[NB_ROBOTS];
		Eigen::Vector4f _qd[NB_ROBOTS];
		Eigen::Vector3f _aad[NB_ROBOTS];														// desired axis angle 

		Eigen::Vector3f _vd[NB_ROBOTS];
		Eigen::Vector3f _omegad[NB_ROBOTS];
		Eigen::Vector3f _v[NB_ROBOTS];
		Eigen::Vector3f _w[NB_ROBOTS];
		Vector6f 				_Vee[NB_ROBOTS];
		Vector6f 				_Vd_ee[NB_ROBOTS];													// desired velocity twist
		Matrix6f 				_tcp_W_EE[NB_ROBOTS];												// Velocity Twist transformation between the robot EE and the tool center point (tcp)
		Vector6f 				_VEE_oa[NB_ROBOTS]; 												// self collision (obstacle) avoidance
		Eigen::Vector3f _fxc[NB_ROBOTS];     												// Desired conservative parts of the nominal DS [m/s] (3x1)
		Vector6f  			_wrench[NB_ROBOTS];          								// Wrench [N and Nm] (6x1)
		Vector6f 				_wrenchBias[NB_ROBOTS];											// Wrench bias [N and Nm] (6x1)
		Vector6f        _filteredWrench[NB_ROBOTS];  								// Filtered wrench [N and Nm] (6x1)

		float _targetForce;                  												// Target force in contact [N]
		// float _d1[NB_ROBOTS];
		// float _err[NB_ROBOTS];
		// bool _qp_wrench_generation;
		bool _firstRobotPose[NB_ROBOTS];
		bool _firstRobotTwist[NB_ROBOTS];
		bool _firstWrenchReceived[NB_ROBOTS];

		Eigen::Matrix4f _w_H_ee[NB_ROBOTS];													// Homogenenous transform of the End-effectors poses (4x4)
		Eigen::Matrix4f _w_H_eeStandby[NB_ROBOTS];									// Homogenenous transform of Standby pose of the End-effectors (4x4)
		Eigen::Matrix4f _w_H_rb[NB_ROBOTS];													// Homogenenous transform of robots base frame (4x4)
		Eigen::Matrix4f _rb_H_eeStandby[NB_ROBOTS];									// Homogenenous transform of EE standby poses relatve to robots base (4x4)
		Eigen::Vector3f _xrbStandby[NB_ROBOTS];		    							// quaternion orientation of EE standby poses relatve to robots base (3x1)
		Eigen::Vector4f _qrbStandby[NB_ROBOTS];		    							// quaternion orientation of EE standby poses relatve to robots base (4x1)

		std::unique_ptr<SGF::SavitzkyGolayFilter> _sgf_ddq_filtered_l;
		std::unique_ptr<SGF::SavitzkyGolayFilter> _sgf_ddq_filtered_r;
		//
		Vector7f _joints_positions[NB_ROBOTS];
		Vector7f _joints_velocities[NB_ROBOTS];
		Vector7f _joints_accelerations[NB_ROBOTS];
		Vector7f _joints_torques[NB_ROBOTS];


		robot_var(){}
		~robot_var(){};

		void init_robot(int sgf_q[], float dt, Eigen::Vector3f gravity){
			//
			_gravity = gravity;

			for(int k= 0; k < NB_ROBOTS; k++)
			{
				// robots
				_nb_joints[k] = 7;
				_x[k].setConstant(0.0f);
				_q[k].setConstant(0.0f);
				_wRb[k].setIdentity();
				_w_H_ee[k].setConstant(0.0f);
				_w_H_eeStandby[k].setConstant(0.0f);
				_w_H_rb[k].setIdentity();
				_rb_H_eeStandby[k].setIdentity();
				_xrbStandby[k].setZero();
				_qrbStandby[k].setZero();

				// desired values
				_Vd_ee[k].setZero();
				_Vee[k].setZero();
				_tcp_W_EE[k].setIdentity();
				_xd[k].setZero();
				_v[k].setZero();
				_w[k].setZero();
				_vd[k].setZero();
				_omegad[k].setZero();
				_qd[k].setZero();
				_aad[k].setZero();

				// forces control variables
				_fxc[k].setZero();
				// _d1[k] 	= 1.0f;
				// _Fd[k] 	= 0.0f;
				// _err[k] = 1.0f;
				_filteredWrench[k].setZero();
				_wrench[k].setZero();
				_wrenchBias[k].setZero();
				_normalForceAverage[k] = 0.0f;
				_wrenchCount[k] 		= 0;
				_normalForce[k] 		= 0.0f;
				// _wrenchBiasOK[k] 		= false;
				// _firstRobotPose[k] 	= false;
				// _firstRobotTwist[k] = false;
		
				// joinr variables
				_joints_positions[k].setZero();
				_joints_velocities[k].setZero();
				_joints_accelerations[k].setZero();
				_joints_torques[k].setZero();

				_VEE_oa[k].setZero();
			}
			// Filtered variable (SG)
			_sgf_ddq_filtered_l = std::make_unique<SGF::SavitzkyGolayFilter>(sgf_q[0], sgf_q[1], sgf_q[2], dt); //(7,3,6,_dt); // dim, order. window lenght
			_sgf_ddq_filtered_r = std::make_unique<SGF::SavitzkyGolayFilter>(sgf_q[0], sgf_q[1], sgf_q[2], dt); //(7,3,6,_dt); // dim, order. window lenght
		}

		void get_StandbyHmgTransformInBase(){
			_w_H_eeStandby[0] = Utils<float>::pose2HomoMx(_xrbStandby[0],  _qrbStandby[0]);		// 
			_w_H_eeStandby[1] = Utils<float>::pose2HomoMx(_xrbStandby[1],  _qrbStandby[1]);		// 
		}

		void get_StandbyHmgTransformInWorld(){
			_w_H_eeStandby[0] =  _w_H_rb[0] * Utils<float>::pose2HomoMx(_xrbStandby[0],  _qrbStandby[0]);		// 
			_w_H_eeStandby[1] =  _w_H_rb[1] * Utils<float>::pose2HomoMx(_xrbStandby[1],  _qrbStandby[1]);		// 
		}

		void get_EndEffectorHmgTransform(){
			_w_H_ee[0]  = Utils<float>::pose2HomoMx(_x[0],  _q[0]);			// WITH EE pose wrt. the world
			_w_H_ee[1]  = Utils<float>::pose2HomoMx(_x[1],  _q[1]);			// WITH EE pose wrt. the world
		}

		void get_desired_lin_task_velocity(float applyVelo, float nu_Wr0){
			_vd[0] = applyVelo *_vd[0] + nu_Wr0 * _fxc[0];
			_vd[1] = applyVelo *_vd[1] + nu_Wr0 * _fxc[1];
		}

		void get_robotBaseFrameInWorld(Eigen::Vector3f xB, Eigen::Vector4f q, int k){
			_w_H_rb[k].block(0,3,3,1) = xB;
			_w_H_rb[k].block(0,0,3,3) = Utils<float>::quaternionToRotationMatrix(q);
		}

		void update_EndEffectorPosesInWorld(Eigen::Vector3f xB, Eigen::Vector4f q, int k){
			// update positions and Orientations of the EEs
			_x[k] = xB;
			_q[k] = q;

			// update EE positions with tool offset
			_wRb[k] = Utils<float>::quaternionToRotationMatrix(_q[k]);
			_x[k]   = _x[k]+_toolOffsetFromEE[k]*_wRb[k].col(2);

			// update velocity Twist transformation from EE to tcp
			Eigen::Vector3f tcp = _toolOffsetFromEE[k]*_wRb[k].col(2);
			Eigen::Matrix3f skew_Mx_tcp; 
			skew_Mx_tcp <<   0.0f,   -tcp(2),     tcp(1),
			               tcp(2),      0.0f,    -tcp(0),
			              -tcp(1),    tcp(0),       0.0f;
			_tcp_W_EE[k].block(0,3,3,3) = skew_Mx_tcp;
		}

		void update_EndEffectorVelocity(Eigen::Vector3f vE, Eigen::Vector3f wE, int k){
			_v[k] = vE;
			_w[k] = wE;
			_Vee[k].head(3)  = _v[k];
			_Vee[k].tail(3)  = _w[k];
			_Vee[k] = _tcp_W_EE[k] * _Vee[k];
		}

		void update_EndEffectorWrench(Eigen::Matrix<float,6,1> raw, Eigen::Vector3f normalObj[], float filteredForceGain, bool wrenchBiasOK[], int k){
			//
			if(!wrenchBiasOK[k])
		  {
		    Eigen::Vector3f loadForce 	 = _wRb[k].transpose()*_toolMass[k]*_gravity;
		    _wrenchBias[k].segment(0,3) -= loadForce;
		    _wrenchBias[k].segment(3,3) -= _toolComPositionFromSensor[k].cross(loadForce);
		    _wrenchBias[k] += raw; 
		    _wrenchCount[k]++;

		    if(_wrenchCount[k]==NB_FT_SENSOR_SAMPLES)
		    {
		      _wrenchBias[k] /= NB_FT_SENSOR_SAMPLES;
		      wrenchBiasOK[k] = true;
		      // std::cerr << "[robot]: Bias " << k << ": " <<_wrenchBias[k].transpose() << std::endl;
		    }
		  }

		  if(wrenchBiasOK[k])
		  {
		    _wrench[k] = raw -_wrenchBias[k];
		    Eigen::Vector3f loadForce = _wRb[k].transpose()*_toolMass[k]*_gravity;
		    _wrench[k].segment(0,3)  -= loadForce;
		    _wrench[k].segment(3,3)  -= _toolComPositionFromSensor[k].cross(loadForce);
		    _wrench[k].head(3) = _wRb[k] * _wrench[k].head(3);
				_wrench[k].tail(3) = _wRb[k] * _wrench[k].tail(3);
		    _filteredWrench[k] = filteredForceGain*_filteredWrench[k]+(1.0f-filteredForceGain)*_wrench[k];
		    //
		    _normalForce[k] = fabs((_filteredWrench[k].segment(0,3)).dot(normalObj[k]));
		  }
		}

		void get_estimated_joint_accelerations(int k){
			//
			SGF::Vec temp_acc(_nb_joints[k]);
			if(k==0){
				_sgf_ddq_filtered_l->AddData(_joints_velocities[k]);
		  	_sgf_ddq_filtered_l->GetOutput(1,temp_acc);
			}
			else{
				_sgf_ddq_filtered_r->AddData(_joints_velocities[k]);
		  	_sgf_ddq_filtered_r->GetOutput(1,temp_acc);
			}
		 	_joints_accelerations[k]	= temp_acc.cast<float>();	
		}

		void get_estimated_AverageNormalForce(){
			//
			for(int k = 0; k < NB_ROBOTS; k++){
		    if(_normalForceWindow[k].size()<MOVING_FORCE_WINDOW_SIZE){
		      _normalForceWindow[k].push_back(_normalForce[k]);
		      _normalForceAverage[k] = 0.0f;
		    }
		    else{
		      _normalForceWindow[k].pop_front();
		      _normalForceWindow[k].push_back(_normalForce[k]);
		      _normalForceAverage[k] = 0.0f;
		      for(int m = 0; m < MOVING_FORCE_WINDOW_SIZE; m++){
		        _normalForceAverage[k]+=_normalForceWindow[k][m];
		      }
		      _normalForceAverage[k] /= MOVING_FORCE_WINDOW_SIZE;
		    }
		  }
		}

		void set_init_parameters(	float toolMass_param[],     
															float toolOffsetFromEE_param[],
															Eigen::Vector3f toolComPositionFromSensor_param[],
															Eigen::Vector3f xrbStandby_param[],
															Eigen::Vector4f qrbStandby_param[]){
			//
			memcpy(_toolMass, 									&toolMass_param[0], 									NB_ROBOTS * sizeof *toolMass_param); 
			memcpy(_toolOffsetFromEE, 					&toolOffsetFromEE_param[0], 					NB_ROBOTS * sizeof *toolOffsetFromEE_param); 
			memcpy(_toolComPositionFromSensor, 	&toolComPositionFromSensor_param[0], 	NB_ROBOTS * sizeof *toolComPositionFromSensor_param); 
			memcpy(_xrbStandby, 								&xrbStandby_param[0], 								NB_ROBOTS * sizeof *xrbStandby_param); 
			memcpy(_qrbStandby, 								&qrbStandby_param[0], 								NB_ROBOTS * sizeof *qrbStandby_param); 

			// get stanby transformation of the EEs wrt. the dual-robot Base frame
			this->get_StandbyHmgTransformInBase();

		}

};

#endif // ROBOT_VARIABLES_H