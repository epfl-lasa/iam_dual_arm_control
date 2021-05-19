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
#include "std_msgs/String.h"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"

#include "dualArmFreeMotionController.h"
#include "dualArmCooperativeController.h"

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

	protected: 
		std::mutex _mutex;
		///////////////////
	    // ROS variables //
	    ///////////////////
		ros::NodeHandle nh_;	// Ros node handle
		ros::Rate loop_rate_;	// Ros loop rate [Hz]

		float _t0_run;

		//////////////////////////////
    // Subscribers declarations //
    //////////////////////////////
		ros::Subscriber _sub_object_pose;
		ros::Subscriber _sub_base_pose[NB_ROBOTS];					// subscribe to the base pose of the robots
		ros::Subscriber _sub_ee_pose[NB_ROBOTS];						// subscribe to the end effectors poses
		ros::Subscriber _sub_ee_velo[NB_ROBOTS];						// subscribe to the end effectors velocity Twist
		ros::Subscriber _subForceTorqueSensor[NB_ROBOTS];		// Subscribe to force torque sensors
		// ros::Subscriber sub_joint_states[NB_ROBOTS];			// subscriber for the joint position
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

		//////////////////////////////
		// List of the topics
		//////////////////////////////
		std::string _topic_pose_object;
		std::string _topic_pose_robot_base[NB_ROBOTS];
		std::string _topic_pose_robot_ee[NB_ROBOTS];
		std::string _topic_ee_commands[NB_ROBOTS];
		std::string _topic_subForceTorqueSensor[NB_ROBOTS];

		// Velocity commands to be sent to the robots
		std_msgs::Float64MultiArray _pubVelo[NB_ROBOTS]; // velocity Twist data to be published

		geometry_msgs::WrenchStamped _msgFilteredWrench;

		// Callbacks
		void objectPoseCallback(const geometry_msgs::Pose::ConstPtr& msg);
		void updateBasePoseCallback(const geometry_msgs::Pose::ConstPtr& msg , int k);
		void updateEEPoseCallback(const geometry_msgs::Pose::ConstPtr& msg , int k);
		void updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg, int k);
		void updateRobotWrenchLeft(const geometry_msgs::WrenchStamped::ConstPtr& msg);
		void updateRobotWrenchRight(const geometry_msgs::WrenchStamped::ConstPtr& msg);
		void updateEETwistCallback(const geometry_msgs::Twist::ConstPtr& msg, int k);
		// Update contact state with the surface
    	void updateContactState();
		
		// --------------------------------------------------------------------------------
		float _toolMass;                             			// Tool mass [kg]
    	float _toolOffsetFromEE[NB_ROBOTS];          		// Tool offset along z axis of end effector [m]   
		Eigen::Vector3f _objectDim;                  			// Object dimensions [m] (3x1)
		Eigen::Vector3f _gravity;
		Eigen::Vector3f _toolComPositionFromSensor;
		int _wrenchCount[NB_ROBOTS];                      // Counter used to pre-process the force data
		ContactState _contactState;                       // Contact state with the object
		std::deque<float> _normalForceWindow[NB_ROBOTS];  // Moving window saving the robots' measured normal force to the object's surface [N]  
		float _normalForceAverage[NB_ROBOTS];             // Average normal force measured through the force windows [N]
		float _normalForce[NB_ROBOTS];                    // Normal force to the surface [N] 
		float _c;                                         // Contact value (1 = CONTACT, 0 otherwise)
		bool _wrenchBiasOK[NB_ROBOTS];                 		// Check if computation of force/torque sensor bias is OK

		float _eD;                                        // Error to desired distance vector [m]                       
    	float _eoD;                                       // Error to object dimension vector [m]                       
    	float _eC;                                        // Error to desired center position [m]
    	float _eoC;                                       // Error to object center position [m]  

    	Eigen::Vector3f _xoC;                             // Measured object center position [m] (3x1)
    	Eigen::Vector3f _xoD;                             // Measured object dimension vector [m] (3x1)
		Eigen::Vector3f _xdC;                             // Desired center position [m] (3x1)
    	Eigen::Vector3f _xdD;                             // Desired distance vector [m] (3x1)
		// --------------------------------------------------------------------------------
		Eigen::Vector3f _x[NB_ROBOTS];
		Eigen::Vector4f _q[NB_ROBOTS];
		Eigen::Matrix3f _wRb[NB_ROBOTS];             			// Orientation matrix (3x3)
		Eigen::Vector3f _xd[NB_ROBOTS];
		Eigen::Vector4f _qd[NB_ROBOTS];
		Eigen::Vector3f _aad[NB_ROBOTS];									// desired axis angle 
		Eigen::Vector3f _vd[NB_ROBOTS];
		Eigen::Vector3f _omegad[NB_ROBOTS];
		Eigen::Vector3f _v[NB_ROBOTS];
		Eigen::Vector3f _w[NB_ROBOTS];
		Vector6f _Vd_ee[NB_ROBOTS];												// desired velocity twist
		Eigen::Vector3f _fxc[NB_ROBOTS];     							// Desired conservative parts of the nominal DS [m/s] (3x1)

    	float _Fd[NB_ROBOTS];                							// Desired force profiles [N]
    	float _targetForce;                  							// Target force in contact [N]
    	float _d1[NB_ROBOTS];
    	float _err[NB_ROBOTS];
    	bool _qp_wrench_generation;
		bool _firstRobotPose[NB_ROBOTS];
		bool _firstRobotTwist[NB_ROBOTS];
		bool _firstWrenchReceived[NB_ROBOTS];
		bool _sensedContact;
		bool _goHome;

		Eigen::Matrix4f _w_H_ee[NB_ROBOTS];								// Homogenenous transform of the End-effectors poses (4x4)
	    Eigen::Matrix4f _w_H_eeStandby[NB_ROBOTS];				// Homogenenous transform of Standby pose of the End-effectors (4x4)
	    Eigen::Matrix4f _w_H_rb[NB_ROBOTS];								// Homogenenous transform of robots base frame (4x4)
	    Eigen::Matrix4f _rb_H_eeStandby[NB_ROBOTS];				// Homogenenous transform of EE standby poses relatve to robots base (4x4)
	    Eigen::Vector3f _xrbStandby[NB_ROBOTS];		    		// quaternion orientation of EE standby poses relatve to robots base (3x1)
	    Eigen::Vector4f _qrbStandby[NB_ROBOTS];		    		// quaternion orientation of EE standby poses relatve to robots base (4x1)
	   
	    Vector6f  		  _wrench[NB_ROBOTS];          			// Wrench [N and Nm] (6x1)
	    Vector6f 				_wrenchBias[NB_ROBOTS];			 			// Wrench bias [N and Nm] (6x1)
	    Vector6f        _filteredWrench[NB_ROBOTS];  			// Filtered wrench [N and Nm] (6x1)
	    int 						_initPoseCount;										// Counter of received initial poses measurements 

	    Eigen::Matrix4f _w_H_gp[NB_ROBOTS];
	    Eigen::Vector3f _xgp_o[NB_ROBOTS];
	    Eigen::Vector4f _qgp_o[NB_ROBOTS];
	    Eigen::Vector3f _n[NB_ROBOTS];               			// Normal vector to surface object for each robot (3x1)
	    Vector6f        _V_gpo[NB_ROBOTS];

	    Eigen::Vector3f _delta_pos; 											// variation of object position
	    Eigen::Vector3f _delta_ang; 											// variation of object orientation euler angles
	    bool _objCtrlKey;
	    bool _releaseAndretract;

		  float _objectMass;
	    Eigen::Vector3f _xo;
	    Eigen::Vector4f _qo;
	    Eigen::Vector3f _vo;
	    Eigen::Vector3f _wo;
	    Eigen::Matrix4f _w_H_o;

	    Eigen::Vector3f _xDo; 
		Eigen::Vector4f _qDo;
		Eigen::Matrix4f _w_H_Do;
		Eigen::Matrix4f _o_H_ee[NB_ROBOTS];
		Eigen::Matrix4f _w_H_Dgp[NB_ROBOTS];
		int _objecPoseCount;

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

		float _reachable;
		float _v_max;
		float _w_max;
		float _filteredForceGain;
		float _forceThreshold;
		float _nu_Wr0;
		float _nu_Wr1;
		float _applyVelo;
		float _delta_oDx;
		float _delta_oDy;
		float _delta_oDz;
		float _desVtoss ;

		bool _stop;                                     // Check for CTRL+C
		bool _isThrowing;																// if true execute throwing of the object
		bool _goToAttractors;														// send the robots to their attractors
		bool _isPlacing;

		// data logging
		std::string   _DataID;
		std::ofstream _OutRecord_pose;
		std::ofstream _OutRecord_velo;
		std::ofstream _OutRecord_efforts;
		////////////////////////////////////////////
		ros::Subscriber _sub_N_objects_pose[NB_ROBOTS];			// subscribe to the base pose of the robots
		Eigen::Matrix4f _w_H_No[NB_OBJECTS];
		Eigen::Matrix4f _w_H_abs_Do;
		Eigen::Matrix4f _lDo_H_rDo;
		Eigen::Matrix4f _w_H_abs_o;
		Eigen::Matrix4f _lo_H_ro;
		//
		Eigen::Vector3f _xNo[NB_OBJECTS];
		Eigen::Vector4f _qNo[NB_OBJECTS];

    void updateObjectsPoseCallback(const geometry_msgs::Pose::ConstPtr& msg , int k);
   
    ////////////////////////////////////////////////////////////////////////
    // Objects for Unconstrained and contrained motion and force generation
    ////////////////////////////////////////////////////////////////////////
	  dualArmFreeMotionController 	FreeMotionCtrl;		// Motion generation
	  dualArmCooperativeController 	CooperativeCtrl;	// Force generation

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
		void computeCommands();
		void publish_commands();
		void publishData();
		void saveData();
		void run();
		void prepareCommands(Vector6f Vd_ee[], Eigen::Vector4f qd[], Vector6f V_gpo[]);
		void getGraspPointsVelocity();
		void Keyboard_reference_object_control();
		void Keyboard_object_control();

};

#endif // DUAL_ARM_CONTROL_H




