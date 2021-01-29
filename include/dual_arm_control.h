#ifndef DUAL_ARM_CONTROL_H
#define DUAL_ARM_CONTROL_H


#include <iostream>
#include <iomanip>
#include <signal.h>
#include <mutex>
#include <fstream>
#include <pthread.h>
#include <vector>
#include <deque>
#include "ros/ros.h"
#include <ros/package.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Twist.h"
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

// typedef Eigen::Matrix<float, 7, 1> Vector7f;
// typedef Eigen::Matrix<float, 6, 1> Vector6f;
// typedef Eigen::Matrix<float, 6, 6> Matrix6f;


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

		///////////////////
	    // ROS variables //
	    ///////////////////
		ros::NodeHandle nh_;	// Ros node handle
		ros::Rate loop_rate_;	// Ros loop rate [Hz]

		float t0_run;

		//////////////////////////////
	    // Subscribers declarations //
	    //////////////////////////////
		ros::Subscriber sub_object_pose;
		ros::Subscriber sub_left_base_pose;
		ros::Subscriber sub_left_ee_pose;
		ros::Subscriber sub_right_base_pose;
		ros::Subscriber sub_right_ee_pose;
		// ros::Subscriber sub_left_joint_states;
		// ros::Subscriber sub_right_joint_states;
		ros::Subscriber _subForceTorqueSensor[NB_ROBOTS];     // Subscribe to force torque sensors

		// Publishers:
		ros::Publisher pub_left_ts_commands;		// ee left velocity twist
		ros::Publisher pub_right_ts_commands;		// ee right velocity twist
		// ros::Publisher pub_left_js_commands;		// ee left velocity twist
		// ros::Publisher pub_right_js_commands;		// ee right velocity twist

		// Velocity commands to be sent to the robots
		std_msgs::Float64MultiArray pubVelo[NB_ROBOTS]; // velocity Twist data to be published
		geometry_msgs::WrenchStamped _msgFilteredWrench;

		// Callbacks
		void objectPoseCallback(const geometry_msgs::Pose::ConstPtr& msg);
		void updateBasePoseCallback(const geometry_msgs::Pose::ConstPtr& msg , int k);
		void updateEEPoseCallback(const geometry_msgs::Pose::ConstPtr& msg , int k);
		void updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg, int k);

		// Update contact state with the surface
    void updateContactState();
    // Compute desired contact force profile
    void computeDesiredContactForceProfile();
		// void rarmPoseCallback(const geometry_msgs::Pose::ConstPtr& msg);
		// void larmJointStatesCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
		// void rarmJointStatesCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

		std::string _topic_pose_object;
		std::string _topic_pose_robot_base_left;
		std::string _topic_pose_robot_ee_left;
		std::string _topic_ee_commands_left;
		std::string _topic_pose_robot_base_right;
		std::string _topic_pose_robot_ee_right;
		std::string _topic_ee_commands_right;


		// --------------------------------------------------------------------------------
		float _toolMass;                             // Tool mass [kg]
    float _toolOffsetFromEE;                     // Tool offset along z axis of end effector [m]   
		Eigen::Vector3f _objectDim;                  // Object dimensions [m] (3x1)
		Eigen::Vector3f _gravity;
		Eigen::Vector3f _toolComPositionFromSensor;
		int _wrenchCount[NB_ROBOTS];                          // Counter used to pre-process the force data
		ContactState _contactState;                           // Contact state with the object
		std::deque<float> _normalForceWindow[NB_ROBOTS];  // Moving window saving the robots' measured normal force to the object's surface [N]  
		float _normalForceAverage[NB_ROBOTS];             // Average normal force measured through the force windows [N]
		float _normalForce[NB_ROBOTS];                        // Normal force to the surface [N] 
		float _c;                                             // Contact value (1 = CONTACT, 0 otherwise)
		bool _wrenchBiasOK[NB_ROBOTS];                 // Check if computation of force/torque sensor bias is OK

		float _eD;                                        // Error to desired distance vector [m]                       
    float _eoD;                                       // Error to object dimension vector [m]                       
    float _eC;                                        // Error to desired center position [m]
    float _eoC;                                       // Error to object center position [m]  
		// --------------------------------------------------------------------------------

		Eigen::Vector3f _x[NB_ROBOTS];
		Eigen::Vector4f _q[NB_ROBOTS];
		Eigen::Matrix3f _wRb[NB_ROBOTS];             // Orientation matrix (3x3)

		Eigen::Vector3f _xd[NB_ROBOTS];
		Eigen::Vector4f _qd[NB_ROBOTS];
		Eigen::Vector3f _aad[NB_ROBOTS];						// desired axis angle 
		Eigen::Vector3f _vd[NB_ROBOTS];
		Eigen::Vector3f _omegad[NB_ROBOTS];
		Vector6f _Vd_ee[NB_ROBOTS];									// desired velocity twist

		Eigen::Vector3f _fxc[NB_ROBOTS];     // Desired conservative parts of the nominal DS [m/s] (3x1)
    float _Fd[NB_ROBOTS];                // Desired force profiles [N]
    float _targetForce;                  // Target force in contact [N]
    float _err[NB_ROBOTS];




		float _d1[NB_ROBOTS];

		bool _firstRobotPose[NB_ROBOTS];
		bool _firstRobotTwist[NB_ROBOTS];
		bool _firstWrenchReceived[NB_ROBOTS];
		bool _sensedContact;
		bool _goHome;

		Eigen::Matrix4f _w_H_ee[NB_ROBOTS];
    Eigen::Matrix4f _w_H_eeStandby[NB_ROBOTS];
    Eigen::Matrix4f _w_H_rb[NB_ROBOTS];
    Vector6f  		  _wrench[NB_ROBOTS];          // Wrench [N and Nm] (6x1)
    Vector6f 				_wrenchBias[NB_ROBOTS];			 // Wrench bias [N and Nm] (6x1)
    Vector6f        _filteredWrench[NB_ROBOTS];  // Filtered wrench [N and Nm] (6x1)

    Eigen::Matrix4f _w_H_gp[NB_ROBOTS];
    Eigen::Vector3f _xgp_o[NB_ROBOTS];
    Eigen::Vector4f _qgp_o[NB_ROBOTS];
    Eigen::Vector3f _n[NB_ROBOTS];                    // Normal vector to surface object for each robot (3x1)
    Vector6f        _V_gpo[NB_ROBOTS];

    
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

    float reachable_p;
    float reachable_o;
    float _v_max;
    float _w_max;
    float _filteredForceGain;
    float _forceThreshold;
    float _nuWrench;

    bool _stop;                                    // Check for CTRL+C
	  //
	  dualArmFreeMotionController 	FreeMotionCtrl;
	  dualArmCooperativeController 	CooperativeCtrl;

	private:
    	// Callback called when CTRL is detected to stop the node       
      static void stopNode(int sig);
	//  static dual_arm_control* me; // Pointer on the instance of the class

	public :
		//
		//
		dual_arm_control(ros::NodeHandle &n, double frequency, 	std::string topic_pose_object_,
																														std::string topic_pose_robot_base_left_,
																														std::string topic_pose_robot_ee_left_,
																														std::string topic_ee_commands_left_,
																														std::string topic_pose_robot_base_right_,
																														std::string topic_pose_robot_ee_right_,
																														std::string topic_ee_commands_right_);
		~dual_arm_control();

		bool init();
		void updatePoses();
		void computeCommands();
		void publish_commands();
		void run();

		 
};

#endif // DUAL_ARM_CONTROL_H




