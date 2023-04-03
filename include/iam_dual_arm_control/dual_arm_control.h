#ifndef DUAL_ARM_CONTROL_H
#define DUAL_ARM_CONTROL_H


#include <iostream>
#include <iomanip>
#include <signal.h>
#include <mutex>
#include <fstream>
#include <sstream>
#include <pthread.h>
#include <vector>
#include <deque>
#include <stdio.h>
#include <termios.h>


#include "ros/ros.h"
#include <ros/package.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/Marker.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"
#include "sg_filter.h"

#include "dualArmFreeMotionController.h"
#include "dualArmCooperativeController.h"
#include "throwingDS.h"
#include "toss_task_param_estimator.h"
#include "data_logging.hpp"

#define NB_ROBOTS 2                  // Number of robots
#define NB_FT_SENSOR_SAMPLES 50      // Number of force torque sensors' samples used for initial calibration (compute the offsets)
#define NB_OPTITRACK_SAMPLES 100     // Number of optitrack samples used for initial objects' pose estimation
#define NB_TRACKED_OBJECTS 6         // Number of objects tracked by the motion capture system (optitrack)
#define MOVING_FORCE_WINDOW_SIZE 10  // Window's size used to average the force data and detect peristent contact

#define NB_OBJECTS 3                  // Number of objects

// typedef Eigen::Matrix<float, 7, 1> Vector7f;
// typedef Eigen::Matrix<float, 6, 1> Vector6f;
// typedef Eigen::Matrix<float, 6, 6> Matrix6f;

//! reading keyboard functions
int khbit_2();
void nonblock_2(int state);
bool keyState_2(char key);

struct spherical_position{
	float r;
	float theta;
	float phi;

	void from_cartesian(Eigen::Vector3f pos){
		r     = pos.norm();
		theta = std::atan2(pos(1), pos(0));
		phi   = std::atan2(pos(2), pos.head(2).norm());
	}

	void to_cartesian(Eigen::Vector3f &pos){
		pos(0) = r * std::cos(phi) * std::cos(theta);
		pos(1) = r * std::cos(phi) * std::sin(theta);
		pos(2) = r * std::sin(phi);
	}
};

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
		// float _eD;                                       				// Error to desired distance vector [m]                       
		// float _eoD;                                      				// Error to object dimension vector [m]                       
		// float _eC;                                       				// Error to desired center position [m]
		// float _eoC;                                      				// Error to object center position [m]  
		Eigen::Vector3f _xdC;                             					// Desired center position [m] (3x1)
		Eigen::Vector3f _xdD;                             					// Desired distance vector [m] (3x1)
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

			std::cout << "[dual_arm_control]: _toolOffsetFromEE  OOOOOOOOOOOOOOOOO  \t" << _toolOffsetFromEE[k] << std::endl;  //

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

class object_to_grasp{

	public:

		// object
		float 					_objectMass;
		Eigen::Vector3f _objectDim;                  			// Object dimensions [m] (3x1)
		Eigen::Vector3f _xo;
		Eigen::Vector4f _qo;
		Eigen::Vector3f _xDo; 
		Eigen::Vector4f _qDo;
		Eigen::Matrix4f _w_H_o;
		Eigen::Matrix4f _w_H_Do;
		Eigen::Vector3f _xoC;                             // Measured object center position [m] (3x1)
		Eigen::Vector3f _xoD;                             // Measured object dimension vector [m] (3x1)
		Eigen::Vector3f _xgp_o[NB_ROBOTS];
		Eigen::Vector4f _qgp_o[NB_ROBOTS];
		Eigen::Matrix4f _w_H_gp[NB_ROBOTS];
		Eigen::Matrix4f _w_H_Dgp[NB_ROBOTS];
		Eigen::Vector3f _vo;
		Eigen::Vector3f _wo;
		// Vector6f 		_Vo;
		// Vector6f 		_Vd_o;   													// desired object velocity (toss)
		// Vector6f  		_desired_object_wrench; 
		Eigen::Vector3f _n[NB_ROBOTS];               			// Normal vector to surface object for each robot (3x1)
		Vector6f        _V_gpo[NB_ROBOTS];

		std::unique_ptr<SGF::SavitzkyGolayFilter> _xo_filtered;
		std::unique_ptr<SGF::SavitzkyGolayFilter> _qo_filtered;
		// KF_3DVeloFromPosEstimator 								_xo_KF_filtered; //
		// KF_3DVeloFromPosEstimator 								_wo_KF_filtered; //

		object_to_grasp(){};
		~object_to_grasp(){};

		void init_object(int sgf_p[], int sgf_o[], float dt, Eigen::Matrix3f o_R_gpl, Eigen::Matrix3f o_R_gpr){
			// object
			_vo.setZero();
			_wo.setZero();
			_xo.setZero(); 
			_xDo.setZero(); 
			// _Vd_o.setZero();
			_xoC.setZero();
			_xoD.setZero();

			_qo  << 1.0f, 0.0f, 0.0f, 0.0f;
			_qDo << 1.0f, 0.0f, 0.0f, 0.0f;
			_w_H_o  = Utils<float>::pose2HomoMx(_xo, _qo);	
			_w_H_Do = Utils<float>::pose2HomoMx(_xDo, _qDo);	
			_qgp_o[0] = Utils<float>::rotationMatrixToQuaternion(o_R_gpl); //
			_qgp_o[1] = Utils<float>::rotationMatrixToQuaternion(o_R_gpr); //

			// normal to contact surfaces
			_n[0] = o_R_gpl.col(2);
			_n[1] = o_R_gpr.col(2);
			_V_gpo[0].setZero();
			_V_gpo[1].setZero();

			//
			_xo_filtered = std::make_unique<SGF::SavitzkyGolayFilter>(sgf_p[0], sgf_p[1], sgf_p[2], dt); //(3,3,6,_dt);
			_qo_filtered = std::make_unique<SGF::SavitzkyGolayFilter>(sgf_o[0], sgf_o[1], sgf_o[2], dt); //(4,3,10,_dt); dim, order, win_l, dt

			// //
			// _xo_KF_filtered.init(_dt, Eigen::Vector2f(0.004, 0.1), 0.004, _xo);
			// _xo_KF_filtered.update(_xo);

			this->get_desiredHmgTransform();
		}

		void get_HmgTransform(){
				_w_H_o = Utils<float>::pose2HomoMx(_xo, _qo);
		}

		void get_desiredHmgTransform(){
				_w_H_Do = Utils<float>::pose2HomoMx(_xDo, _qDo);
		}

		void get_estimated_state(){
			// filtered object position
			SGF::Vec temp(3), temp_o(4);
			_xo_filtered->AddData(_xo);
			_xo_filtered->GetOutput(0,temp);
			_xo = temp;
			_xo_filtered->GetOutput(1,temp);
			_vo = temp;	
			//
			_qo_filtered->AddData(_qo);
			_qo_filtered->GetOutput(0,temp_o);
			_qo = temp_o;

			// normalizing the quaternion
			_qo.normalize();
			//
			if(_qo.norm() <= 1e-8){
				_qo = Eigen::Vector4f(1.0, 0.0, 0.0, 0.0);
			}
			//
			// ===========================================================
			_qo_filtered->GetOutput(1,temp_o);
			Eigen::Vector4f qo_dot = temp_o;
			//
			Eigen::MatrixXf wQ_map(3,4);
			wQ_map << -_qo(1), -_qo(0), -_qo(3),  _qo(2),
								-_qo(2),  _qo(3),  _qo(0), -_qo(1),
								-_qo(3), -_qo(2),  _qo(1),  _qo(0);
			_wo = 0.0*wQ_map*qo_dot;

		}

		void get_grasp_point_HTransform(){
			_w_H_gp[0]  = _w_H_o * Utils<float>::pose2HomoMx(_xgp_o[0],  _qgp_o[0]);
			_w_H_gp[1]  = _w_H_o * Utils<float>::pose2HomoMx(_xgp_o[1],  _qgp_o[1]);
		}

		void get_grasp_point_desiredHTransform(){
			_w_H_Dgp[0] = _w_H_Do * Utils<float>::pose2HomoMx(_xgp_o[0],  _qgp_o[0]);
			_w_H_Dgp[1] = _w_H_Do * Utils<float>::pose2HomoMx(_xgp_o[1],  _qgp_o[1]);
		}

		void get_grasp_point_desiredRotation(){
			_w_H_Dgp[0].block(0,0,3,3) = _w_H_Do.block(0,0,3,3) * Utils<float>::pose2HomoMx(_xgp_o[0],  _qgp_o[0]).block(0,0,3,3);
			_w_H_Dgp[1].block(0,0,3,3) = _w_H_Do.block(0,0,3,3) * Utils<float>::pose2HomoMx(_xgp_o[1],  _qgp_o[1]).block(0,0,3,3);
		}

		void update_grasp_normals(){
			_n[0]       = _w_H_gp[0].block(0,0,3,3).col(2);
			_n[1]       = _w_H_gp[1].block(0,0,3,3).col(2);
		}

		void get_grasp_point_velocity(){
			//velocity of grasp points on the object
		  for(int i=0; i<NB_ROBOTS; i++)
		  {
		    Eigen::Vector3f t = _w_H_o.block<3,3>(0,0) * _xgp_o[i];
		    Eigen::Matrix3f skew_Mx_gpo;
		    skew_Mx_gpo <<   0.0f,  -t(2),     t(1),
		                     t(2),   0.0f,    -t(0),
		                    -t(1),   t(0),     0.0f;             
		    // velocity
		    _V_gpo[i].head(3) = _vo - 0*skew_Mx_gpo * _wo;
		    _V_gpo[i].tail(3) = 0*_wo;
		    //
		    _V_gpo[i].head(3) *= 0.0f;
		    _V_gpo[i].tail(3) *= 0.0f;
		  }
		  
		}

};

class tossing_target{

	public:

	// target (tossing)
	Eigen::Vector3f _xt;
	Eigen::Vector4f _qt;
	Eigen::Vector3f _vt;
	Eigen::Vector3f _wt;

	Eigen::Vector3f _xd_landing;
	Eigen::Vector3f _x_pickup;
	Eigen::Vector3f _x_intercept;   // intercept point of the moving object
	Eigen::Vector3f _xt_state2go;

	std::unique_ptr<SGF::SavitzkyGolayFilter> _xt_filtered; // target
	KF_3DVeloFromPosEstimator 								_xt_KF_filtered; //

	tossing_target(){};
	~tossing_target(){};

	void init_target(int dim, int order, int win_l, float dt){
		// target
		_xt.setZero();
		_qt.setZero();
		_vt.setZero();
		_wt.setZero();
		_qt  << 1.0f, 0.0f, 0.0f, 0.0f;
		_x_pickup.setZero();
		_xt_state2go.setZero();
		//
		_xt_filtered = std::make_unique<SGF::SavitzkyGolayFilter>(dim, order, win_l, dt); //(3,3,10,dt); dim, order, win_l, dt
		_xt_KF_filtered.init(dt, Eigen::Vector2f(0.004, 0.1), 0.004, _xt);
		_xt_KF_filtered.update(_xt);
	}

	void get_filtered_state(){
		// filtered target position
		SGF::Vec temp(3);
	  _xt_filtered->AddData(_xt);
	  _xt_filtered->GetOutput(0,temp);
	  _xt = temp;
	  _xt_filtered->GetOutput(1,temp);
	  _vt = temp;	
	  _xt_KF_filtered.update(_vt);
		_vt = _xt_KF_filtered.get_estimate_position();
	}

};


class dual_arm_control
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
    // TaskType
    // enum TASK_TYPE {GOHOME = 0, RELEASE_AND_RETRACT = 1, PICK_AND_LIFT = 2, PICK_AND_THROW = 3, PICK_AND_PLACE = 4, 
    // 								PICK_AND_HANDOVER = 5, THROWING = 6, HANDINGOVER = 7, PAUSE_MOTION = 8};
    enum TASK_TYPE {REACH = 0, PICK_AND_LIFT = 1, TOSSING = 2, PICK_AND_TOSS = 3, PICK_AND_PLACE = 4, PLACE_TOSSING = 5, THROWING = 6, HANDINGOVER = 7, PAUSE_MOTION = 8};

		// 0=reach, 1=pick, 2=toss, 3=pick_and_toss, 4=pick_and_place

	protected: 
		std::mutex _mutex;
		///////////////////
		// ROS variables //
		///////////////////
		ros::NodeHandle nh_;	// Ros node handle
		ros::Rate loop_rate_;	// Ros loop rate [Hz]

		SGF::real _dt;
		float _t0_run;
		int _cycle_count;
		// int _nb_joints[NB_ROBOTS];

		//////////////////////////////
		// Subscribers declarations //
		//////////////////////////////
		ros::Subscriber _sub_object_pose;
		ros::Subscriber _sub_target_pose;
		ros::Subscriber _sub_base_pose[NB_ROBOTS];					// subscribe to the base pose of the robots
		ros::Subscriber _sub_ee_pose[NB_ROBOTS];						// subscribe to the end effectors poses
		ros::Subscriber _sub_ee_velo[NB_ROBOTS];						// subscribe to the end effectors velocity Twist
		ros::Subscriber _subForceTorqueSensor[NB_ROBOTS];		// Subscribe to force torque sensors
		ros::Subscriber _sub_joint_states[NB_ROBOTS];				// subscriber for the joint position
		//////////////////////////////
		// Publishers:
		//////////////////////////////
		ros::Publisher _pub_ts_commands[NB_ROBOTS];					// Publisher of the End effectors velocity twist
		ros::Publisher _pubDesiredTwist[NB_ROBOTS];        	// Publish desired twist to DS-impdedance controller
		ros::Publisher _pubDesiredOrientation[NB_ROBOTS];  	// Publish desired orientation to DS-impedance controller
		ros::Publisher _pubFilteredWrench[NB_ROBOTS];      	// Publish filtered measured wrench
		ros::Publisher _pubNormalForce[NB_ROBOTS];        	// Publish measured normal force to the surface
		ros::Publisher _pubDesiredVel_Quat[NB_ROBOTS];      // Publish desired EE linear velocity and quaternion

		ros::Publisher _pubDistAttractorEe[NB_ROBOTS];
		ros::Publisher _pubAttractor[NB_ROBOTS];
		ros::Publisher _pubNormLinVel[NB_ROBOTS];						// Publish norms of EE linear velocities

		ros::Publisher _pubAppliedWrench[NB_ROBOTS];				// Publish applied EE wrench
		ros::Publisher _pubApplied_fnornMoment[NB_ROBOTS]; 	// Publish the contact normal and the moment of the applied wrench

		ros::Publisher _pubConveyorBeltMode;              	// Publish conveyor belt mode
		ros::Publisher _pubConveyorBeltSpeed;              	// Publish conveyor belt Speed

		//////////////////////////////
		// List of the topics
		//////////////////////////////
		std::string _topic_pose_object;
		std::string _topic_pose_robot_base[NB_ROBOTS];
		std::string _topic_pose_robot_ee[NB_ROBOTS];
		std::string _topic_ee_commands[NB_ROBOTS];
		std::string _topic_subForceTorqueSensor[NB_ROBOTS];

		// Velocity commands to be sent to the robots
		std_msgs::Float64MultiArray _pubVelo[NB_ROBOTS]; 	// velocity Twist data to be published

		geometry_msgs::WrenchStamped _msgFilteredWrench;

		// --------------------------------------------------------------------------------
		// robot 
		robot_var robot_;

		float _toolMass;                             				// Tool mass [kg]
		float _toolOffsetFromEE[NB_ROBOTS];          				// Tool offset along z axis of end effector [m]   
		Eigen::Vector3f _gravity;
		Eigen::Vector3f _toolComPositionFromSensor;
		int _wrenchCount[NB_ROBOTS];                      	// Counter used to pre-process the force data
		ContactState _contactState;                       	// Contact state with the object
		std::deque<float> _normalForceWindow[NB_ROBOTS];  	// Moving window saving the robots' measured normal force to the object's surface [N]  
		float _normalForceAverage[NB_ROBOTS];             	// Average normal force measured through the force windows [N]
		float _normalForce[NB_ROBOTS];                    	// Normal force to the surface [N] 
		float _c;                                         	// Contact value (1 = CONTACT, 0 otherwise)
		bool _wrenchBiasOK[NB_ROBOTS];                 			// Check if computation of force/torque sensor bias is OK

		float _eD;                                        	// Error to desired distance vector [m]                       
		float _eoD;                                       	// Error to object dimension vector [m]                       
		float _eC;                                        	// Error to desired center position [m]
		float _eoC;                                       	// Error to object center position [m]  
		
		Eigen::Vector3f _xdC;                             	// Desired center position [m] (3x1)
		Eigen::Vector3f _xdD;                             	// Desired distance vector [m] (3x1)
		// --------------------------------------------------------------------------------
		float _Fd[NB_ROBOTS];                								// Desired force profiles [N]
		float _targetForce;                  								// Target force in contact [N]
		float _d1[NB_ROBOTS];
		float _err[NB_ROBOTS];
		bool _qp_wrench_generation;
		bool _firstRobotPose[NB_ROBOTS];
		bool _firstRobotTwist[NB_ROBOTS];
		bool _firstWrenchReceived[NB_ROBOTS];
		// ------------------------------------------------------------------------------------
		bool _sensedContact;

		bool _startlogging;

		Eigen::Vector3f _delta_pos; 											// variation of object position
		Eigen::Vector3f _delta_ang; 											// variation of object orientation euler angles
		Eigen::Vector3f _filt_delta_ang;
		Eigen::Vector3f _filt_delta_ang_mir;
		bool 						_objCtrlKey;

		//---------------------------------------------------------------------------------
		// object to grasp
		object_to_grasp object_;
		Vector6f 				_Vo;
		Vector6f 				_Vd_o;   													// desired object velocity (toss)
		Vector6f  			_desired_object_wrench; 
		Vector6f 				_Vd_oPg;   	

		// -------------------------------
		// tossing target
		tossing_target target_;

		// // target (tossing)
		// Eigen::Vector3f _xt;
		// Eigen::Vector4f _qt;
		// Eigen::Vector3f _vt;
		// Eigen::Vector3f _wt;

		// Eigen::Vector3f _xd_landing;
		// Eigen::Vector3f _x_pickup;
		// Eigen::Vector3f _x_intercept;   // intercept point of the moving object
		// Eigen::Vector3f _xt_state2go;

		//-------------------------------------------------------------------------------------------------
		Eigen::Matrix4f _o_H_ee[NB_ROBOTS];
		int 					  _objecPoseCount;
		int 						_initPoseCount;										// Counter of received initial poses measurements 

		Eigen::Vector3f _v_abs;
		Eigen::Vector3f _w_abs;
		Eigen::Vector3f _v_rel;
		Eigen::Vector3f _w_rel;

		Eigen::Vector3f _ep_abs;
		Eigen::Vector3f _eo_abs;
		Eigen::Vector3f _ep_rel;
		Eigen::Vector3f _eo_rel;

		Matrix6f _gain_abs;
		Matrix6f _gain_rel;

		// task
		Eigen::Vector3f _xDo_lifting; 
		Eigen::Vector4f _qDo_lifting;
		Eigen::Vector3f _xDo_placing; 
		Eigen::Vector4f _qDo_placing;

		float _reachable;
		float _v_max;
		float _w_max;
		float _filteredForceGain;
		float _forceThreshold;
		float _nu_Wr0;
		float _nu_Wr1;
		float _applyVelo;
		float _applyWrench;
		float _delta_oDx;
		float _delta_oDy;
		float _delta_oDz;
		float _desVtoss ;
		float _desVimp;
		float _desVreach;
		float _refVreach;
		float _friction_angle     = 0.0f; 
		float _max_friction_angle = 0.0f;
		float _height_via_point; 

		bool _goHome;
		bool _releaseAndretract;
		bool _stop;                           						// Check for CTRL+C
		bool _isThrowing;																	// if true execute throwing of the object
		bool _goToAttractors;															// send the robots to their attractors
		bool _isPlacing;
		bool _isPickupSet;
		bool _isPlaceTossing;															// fast interrupted placing motion
		bool _impact_dir_preset = true;
		int  _dualTaskSelector  = 1;
		bool _old_dual_method   = false;
 
		// data logging
		std::string   _DataID;
		// create data logging object
		data_logging  datalog;
		////////////////////////////////////////////
		ros::Subscriber _sub_N_objects_pose[NB_ROBOTS];		// subscribe to the base pose of the robots
		Eigen::Matrix4f _w_H_No[NB_OBJECTS];
		Eigen::Matrix4f _w_H_abs_Do;
		Eigen::Matrix4f _lDo_H_rDo;
		Eigen::Matrix4f _w_H_abs_o;
		Eigen::Matrix4f _lo_H_ro;
		//
		Eigen::Vector3f _xNo[NB_OBJECTS];
		Eigen::Vector4f _qNo[NB_OBJECTS];
		//
		Eigen::Matrix3f _BasisQ[NB_ROBOTS];
		Eigen::Matrix3f _E_xt_xd[NB_ROBOTS];
		// Vector6f 		_Vee[NB_ROBOTS];
		// Matrix6f 		_tcp_W_EE[NB_ROBOTS];			// Velocity Twist transformation between the robot EE and the tool center point (tcp)
		Eigen::Vector3f _dirImp[NB_ROBOTS];
		Eigen::Vector3f _VdImpact[NB_ROBOTS]; 
		Eigen::Vector3f _dual_angular_limit;
		bool _release_flag;

		//
		Vector7f _joints_positions[NB_ROBOTS];
		Vector7f _joints_velocities[NB_ROBOTS];
		Vector7f _joints_accelerations[NB_ROBOTS];
		Vector7f _joints_torques[NB_ROBOTS];

		float _delta_Imp  = 0.0f;
		float _delta_Toss = 0.0f;
		float _trackingFactor;
		float _delta_tracking;

		// Vector6f _VEE_oa[NB_ROBOTS];
		std::string _dsDampingTopic[NB_ROBOTS];

		Eigen::Vector3f _delta_rel_pos;
		bool _increment_release_pos = false;
		bool _increment_lift_pos      = false;
		bool _ctrl_mode_conveyor_belt = false;
		spherical_position release_pos;

		int _mode_conveyor_belt;
		int _desSpeed_conveyor_belt;
		int _nominalSpeed_conveyor_belt;
		int _magniture_pert_conveyor_belt;
		Eigen::Vector2f _dual_PathLen_AvgSpeed;
		bool _hasCaughtOnce = false;
		bool _isIntercepting = false;
		float _beta_vel_mod;
		bool _isDisturbTarget = false;
		float _initSpeedScaling;
		std::deque<float> _windowSpeedEE;
		float _movingAvgSpeedEE;
		int _winLengthAvgSpeedEE;
		// int _winCounterAvgSpeedEE;
		bool _adaptationActive = false;
		bool _isTargetFixed = true;
		bool userSelect_ = true;

		bool _feasibleAlgo = false;
		bool _pickupBased  = true;
		bool _trackTargetRotation = false;
		bool _isMotionTriggered =false;
		bool _isRatioFactor = false;
		float _tol_attractor = 0.07f;
		float _switchSlopeAdapt = 100.0f;
		float _beta_vel_mod_unfilt = 1.0f;
		float _time2intercept_tgt;
		float _time2intercept_bot;

		// ------------------------------------------------------------------------
		bool _updatePathEstim  = false;
		int _counter_monocycle = 0;
		int _counter_pickup 	 = 0;
		float _dxEE_dual_avg 	 = 0.f;
		float _dxEE_dual_avg_pcycle	 = 0.f;
		float _dxEE_dual_avg_0 = 0.f;
		float _Del_xEE_dual_avg 	 = 0.f;
		Eigen::Vector3f _xEE_dual;
		Eigen::Vector3f _xEE_dual_0;
		// ------------------------------------------------------------------------

		// target
		std::deque<Eigen::Vector3f> _windowVelTarget;
		Eigen::Vector3f _movingAvgVelTarget;
		////////////////////////////////////////////////////////////////////////
		// Objects for Unconstrained and contrained motion and force generation
		////////////////////////////////////////////////////////////////////////
		dualArmFreeMotionController 	FreeMotionCtrl;			// Motion generation
		dualArmCooperativeController 	CooperativeCtrl;		// Force generation
		throwingDS 										dsThrowing;				      //

		toss_task_param_estimator 		tossParamEstimator; 			// tossing task param estimator
		dualArmFreeMotionController 	FreeMotionCtrlEstim;
		throwingDS 										dsThrowingEstim;				//
		bool 													_isSimulation;


		// Callbacks
		void objectPoseCallback(const geometry_msgs::Pose::ConstPtr& msg);
		void targetPoseCallback(const geometry_msgs::Pose::ConstPtr& msg);
		void updateBasePoseCallback(const geometry_msgs::Pose::ConstPtr& msg , int k);
		void updateEEPoseCallback(const geometry_msgs::Pose::ConstPtr& msg , int k);
		void updateEETwistCallback(const geometry_msgs::Twist::ConstPtr& msg, int k);
		void updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg, int k);
		void updateRobotWrenchLeft(const geometry_msgs::WrenchStamped::ConstPtr& msg);
		void updateRobotWrenchRight(const geometry_msgs::WrenchStamped::ConstPtr& msg);
		void updateContactState();
		// void updateRobotStatesLeft(const sensor_msgs::JointState::ConstPtr &msg);
		// void updateRobotStatesRight(const sensor_msgs::JointState::ConstPtr &msg);
		void updateRobotStates(const sensor_msgs::JointState::ConstPtr &msg, int k);
		void updateObjectsPoseCallback(const geometry_msgs::Pose::ConstPtr& msg , int k);

	public :
		/////////////////////
		// SG Filter variables //
		/////////////////////
		// SGF::SavitzkyGolayFilter _xo_filtered;    			// Filter used for the object's center position
		// std::unique_ptr<SGF::SavitzkyGolayFilter> _xo_filtered;
		// std::unique_ptr<SGF::SavitzkyGolayFilter> _qo_filtered;
		// std::unique_ptr<SGF::SavitzkyGolayFilter> _sgf_ddq_filtered_l;
		// std::unique_ptr<SGF::SavitzkyGolayFilter> _sgf_ddq_filtered_r;
		// SGF::SavitzkyGolayFilter _x_filtered;    			// Filter used for the object's dimension vector
		// std::unique_ptr<SGF::SavitzkyGolayFilter> _xt_filtered; // target
		KF_3DVeloFromPosEstimator 								_xo_KF_filtered; //
		KF_3DVeloFromPosEstimator 								_wo_KF_filtered; //
		// KF_3DVeloFromPosEstimator 								_xt_KF_filtered; //

		tossingTaskVariables _tossVar;

	private:
    	// Callback called when CTRL is detected to stop the node       
      static void stopNode(int sig);
	//  static dual_arm_control* me; // Pointer on the instance of the class
	public :
		//
		dual_arm_control(	ros::NodeHandle &n, double frequency, 	//std::string dataID,
							std::string topic_pose_object_,
							std::string topic_pose_robot_base[],
							std::string topic_pose_robot_ee[],
							std::string topic_ee_commands[],
							std::string topic_sub_ForceTorque_Sensor[]);
		~dual_arm_control();

		bool init();
		void updatePoses();
		void get_pasive_ds_1st_damping();
		void computeCommands();
		void publish_commands();
		void publishData();
		void saveData();
		void run();
		void prepareCommands(Vector6f Vd_ee[], Eigen::Vector4f qd[], Vector6f V_gpo[]);
		void getGraspPointsVelocity();
		void Keyboard_reference_object_control();
		void Keyboard_virtual_object_control();
		Eigen::Vector3f get_impact_direction(Eigen::Vector3f des_object_force, Eigen::Vector3f normal, float coeff_friction);
		void reset_variables();
		void update_states_machines();
		Eigen::Vector3f get_object_desired_direction(int task_type, Eigen::Vector3f object_pos);
		void update_release_position();
		void publish_conveyor_belt_cmds();
		void update_placing_position(float y_t_min, float y_t_max);
		void constrain_placing_position(float x_t_min, float x_t_max, float y_t_min, float y_t_max);
		void set_2d_position_box_constraints(Eigen::Vector3f &position_vec, float limits[]);
		void mirror_target2object_orientation(Eigen::Vector4f qt, Eigen::Vector4f &qo, Eigen::Vector3f ang_lim);
		Eigen::Vector3f compute_intercept_with_target(const Eigen::Vector3f &x_pick, 
		                                              const Eigen::Vector3f &x_target, 
		                                              const Eigen::Vector3f &v_target, 
		                                              float phi_i);
		float get_desired_yaw_angle_target(const Eigen::Vector4f &qt, const Eigen::Vector3f &ang_lim);
		void estimate_moving_average_ee_speed();
		void estimate_moving_average_target_velocity();
		void find_desired_landing_position(Eigen::Vector3f x_origin, bool isPlacing, bool isPlaceTossing, bool isThrowing);
		void update_intercept_position(float flytime_obj, float intercep_limits[]);
		void find_release_configuration();
		void set_release_state();
		void estimate_target_state_to_go(Eigen::Vector2f Lp_Va_pred_bot, Eigen::Vector2f Lp_Va_pred_tgt, float flytime_obj);
		void compute_adaptation_factors(Eigen::Vector2f Lp_Va_pred_bot, Eigen::Vector2f Lp_Va_pred_tgt, float flytime_obj);
		
};

#endif // DUAL_ARM_CONTROL_H




