
#include "dual_arm_control.h"
#include "Utils.hpp"
#include <chrono>
#include <ctime>

using namespace std;
using namespace Eigen;

// ---------------------------------------------------------------------
//! reading keyboard functions
int khbit_2()
{
    struct timeval tv;
    fd_set fds;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
    return FD_ISSET(STDIN_FILENO, &fds);
}

void nonblock_2(int state)
{
    struct termios ttystate;
    tcgetattr(STDIN_FILENO, &ttystate);

    if ( state == 1)
    {
        ttystate.c_lflag &= (~ICANON & ~ECHO); //Not display character
        ttystate.c_cc[VMIN] = 1;
    }
    else if (state == 0)
    {
        ttystate.c_lflag |= ICANON;
        ttystate.c_lflag |= ECHO;
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
}

bool keyState_2(char key)
{
    bool pressed = false;
    int i = khbit_2(); //Alow to read from terminal
    if (i != 0)
    {
        char c = fgetc(stdin);
        if (c == key)
        {
            pressed = true;
        }
        else
        {
            pressed = false;
        }
    }

    return pressed;
}
// ---------------------------------------------------------------------

dual_arm_control::dual_arm_control(	ros::NodeHandle &n, double frequency, 	//std::string dataID,
									std::string topic_pose_object_,
									std::string topic_pose_robot_base[],
									std::string topic_pose_robot_ee[],
									std::string topic_ee_commands[],
									std::string topic_sub_ForceTorque_Sensor[])	: nh_(n)
																				, loop_rate_(frequency)
																				, _dt(1.0f/frequency)
																				// , _xo_filtered(3,3,6,1.0f/frequency)

{
	// me = this;
	_stop = false;
	std::string dataID = "_02";
	_DataID = dataID;
	//
	_topic_pose_object = topic_pose_object_;
	memcpy(_topic_pose_robot_base, &topic_pose_robot_base[0], NB_ROBOTS * sizeof *topic_pose_robot_base); 
	memcpy(_topic_pose_robot_ee, &topic_pose_robot_ee[0], NB_ROBOTS * sizeof *topic_pose_robot_ee); 
	memcpy(_topic_ee_commands, &topic_ee_commands[0], NB_ROBOTS * sizeof *topic_ee_commands); 
	memcpy(_topic_subForceTorqueSensor, &topic_sub_ForceTorque_Sensor[0], NB_ROBOTS * sizeof *topic_sub_ForceTorque_Sensor); 
	//
	_objectMass = 1.0f;
	_objectDim << 0.20f, 0.20f, 0.20f;
	_toolOffsetFromEE[0] = 0.115f;
	_toolOffsetFromEE[1] = 0.115f;
	_toolMass = 0.2f;    // TO CHANGE !!!!
	_gravity << 0.0f, 0.0f, -9.80665f;
	_toolComPositionFromSensor << 0.0f,0.0f,0.035f;
	//
	for(int k= 0; k < NB_ROBOTS; k++)
	{
		// robots
		_x[k].setConstant(0.0f);
		_q[k].setConstant(0.0f);
		_wRb[k].setIdentity();
		_w_H_ee[k].setConstant(0.0f);
		_w_H_eeStandby[k].setConstant(0.0f);
		_w_H_rb[k].setIdentity();
		_rb_H_eeStandby[k].setIdentity();
		_w_H_gp[k].setConstant(0.0f);
		_xrbStandby[k].setZero();
		_qrbStandby[k].setZero();
		// object
		_xgp_o[k].setConstant(0.0f);
		_qgp_o[k].setConstant(0.0f);
		_V_gpo[k].setConstant(0.0f);
		_o_H_ee[k].setIdentity();
		_w_H_Dgp[k].setIdentity();

		// desired values
		_Vd_ee[k].setZero();
		_xd[k].setZero();
		_v[k].setZero();
		_w[k].setZero();
		_vd[k].setZero();
		_omegad[k].setZero();
		_qd[k].setZero();
		_aad[k].setZero();

		// forces control variables
		_fxc[k].setZero();
		_d1[k] = 1.0f;
		_Fd[k] = 0.0f;
		_err[k] = 1.0f;
		_filteredWrench[k].setZero();
		_wrench[k].setZero();
		_wrenchBias[k].setZero();
		_normalForceAverage[k] = 0.0f;
		_wrenchCount[k] = 0;
		_normalForce[k] = 0.0f;
		_wrenchBiasOK[k] = false;

		_firstRobotPose[k] = false;
		_firstRobotTwist[k] = false;
		_firstWrenchReceived[k] = false;
		// 
		_pubVelo[k].data.clear();
		_pubVelo[k].data.push_back(0.0);		// axis angle poses _x	
		_pubVelo[k].data.push_back(0.0);		// axis angle poses _y
		_pubVelo[k].data.push_back(0.0);		// axis angle poses _z
		_pubVelo[k].data.push_back(0.0);		// linear velocity v_x
		_pubVelo[k].data.push_back(0.0);		// linear velocity v_y
		_pubVelo[k].data.push_back(0.0);		// linear velocity v_z
		//
		_BasisQ[k].setIdentity();
		_E_xt_xd[k].setIdentity();
		_Vee[k].setZero();
	}
	// 
	// Filtered variable (SG)
	_xo_filtered = std::make_unique<SGF::SavitzkyGolayFilter>(3,3,6,_dt);
	//
	_vo.setZero();
	_wo.setZero();
	_xo.setZero(); 
	_xDo.setZero(); 
	_qo  << 1.0f, 0.0f, 0.0f, 0.0f;
	_qDo << 1.0f, 0.0f, 0.0f, 0.0f;
	_w_H_o  = Utils<float>::pose2HomoMx(_xo, _qo);	
	_w_H_Do = Utils<float>::pose2HomoMx(_xDo, _qDo);	
	_initPoseCount  = 0;
	_objecPoseCount = 0;

	// object desired grasping points
	_xgp_o[0] << 0.0f, -_objectDim(1)/2.1f,  -0.01f;  	// left   // _xgp_o[0] << 0.0f, -_objectDim(1)/1.8f, 0.0f;  // left
	_xgp_o[1] << 0.0f,  _objectDim(1)/2.1f,  -0.01f; 		// right 	// _xgp_o[1] << 0.0f,  _objectDim(1)/1.8f,  0.0f; // right

	Eigen::Matrix3f o_R_gpl;	o_R_gpl.setZero();		
	Eigen::Matrix3f o_R_gpr;	o_R_gpr.setZero();	
	o_R_gpl(0,0) =  1.0f;	o_R_gpl(2,1) = -1.0f; 	o_R_gpl(1,2) = 1.0f;
	o_R_gpr(0,0) =  1.0f;	o_R_gpr(2,1) =  1.0f; 	o_R_gpr(1,2) =-1.0f;
	// o_R_gpr(0,0) = -1.0f;	o_R_gpr(2,1) = -1.0f; 	o_R_gpr(1,2) =-1.0f; 
	// o_R_gpr(2,0) = -1.0f;	o_R_gpr(0,1) =  1.0f; 	o_R_gpr(1,2) =-1.0f; 
	_qgp_o[0] = Utils<float>::rotationMatrixToQuaternion(o_R_gpl); //
	_qgp_o[1] = Utils<float>::rotationMatrixToQuaternion(o_R_gpr); //

	// normal to contact surfaces
	_n[LEFT]  = o_R_gpl.col(2);
	_n[RIGHT] = o_R_gpr.col(2);

	_c   = 0.0f;
	// coordination velocity
	_v_abs.setZero();
	_w_abs.setZero();
	_v_rel.setZero();
	_w_rel.setZero();
	_v_max = 5.00f;     // velocity limits
	_w_max = 4.0f;     // velocity limits
	// coordination errors
	_ep_abs.setZero();
	_eo_abs.setZero();
	_ep_rel.setZero();
	_eo_rel.setZero();
	_xoC.setZero();
	_xoD.setZero();
	_xdC.setZero();
	_xdD.setZero();
	_eD  = 0.0f;
	_eoD = 0.0f;
	_eC  = 0.0f;
	_eoC = 0.0f;
	// coordination motion gains
	_gain_abs.setZero();
	_gain_abs.topLeftCorner(3,3)     = 1.5f*Eigen::Vector3f(1.5f, 1.5f, 1.5f).asDiagonal();  //3.0f, 3.0f, 3.0f
	_gain_abs.bottomRightCorner(3,3) = 1.5f*Eigen::Vector3f(1.0f, 1.0f, 1.0f).asDiagonal();  //1.5f, 1.5f, 1.5f
	_gain_rel.setZero();
	_gain_rel.topLeftCorner(3,3)     = 1.f*Eigen::Vector3f(2.0f, 2.0f, 2.0f).asDiagonal();  //4.0f, 4.0f, 4.0f
	_gain_rel.bottomRightCorner(3,3) = 1.f*Eigen::Vector3f(1.0f, 1.0f, 1.0f).asDiagonal();  //1.5f, 1.5f, 1.5f

	_reachable = 1.0f;
	_targetForce = 10.0f;
	_filteredForceGain = 0.9f;
	_sensedContact     = false;
	_forceThreshold    = 2.0f;
	_nu_Wr0 				   = 0.0f;
	_nu_Wr1 				   = 0.0f;
	_qp_wrench_generation = false;
	//
	// Stanby pose
	_xrbStandby[LEFT]   = Vector3f(0.35f,  0.20f, 0.80f); // relative to the robot base
	_xrbStandby[RIGHT]  = Vector3f(0.35f, -0.20f, 0.80f); // relative to the robot base
	_qrbStandby[LEFT]  << 0.3827f, -0.9239f, 0.0f, 0.0f;
	_qrbStandby[RIGHT] << 0.3827f,  0.9239f, 0.0f, 0.0f;

	_delta_pos.setZero();
	_delta_ang.setZero();

	_delta_oDx  = 0.0f;
	_delta_oDy  = 0.0f;
	_delta_oDz  = 0.0f;
	_desVtoss   = 0.0f;
	_applyVelo  = 0.0f;
	_desVimp    = 0.3f;

	_objCtrlKey 		= true;
	_goHome 			= true;
	_goToAttractors 	= true;
	_releaseAndretract 	= false;
	_isThrowing 		= false;
	_isPlacing          = false;
	// _t0_run = ros::Time::now().toSec();
	_release_flag = false;
}
//
dual_arm_control::~dual_arm_control(){}

//
bool dual_arm_control::init()
{
	//-------------
	// Subscribers
	//------------
	_sub_object_pose 			 	= nh_.subscribe(_topic_pose_object,1, &dual_arm_control::objectPoseCallback, this, ros::TransportHints().reliable().tcpNoDelay());
	_sub_base_pose[LEFT] 	 	 	= nh_.subscribe<geometry_msgs::Pose>(_topic_pose_robot_base[LEFT], 1, boost::bind(&dual_arm_control::updateBasePoseCallback,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	_sub_ee_pose[LEFT] 			 	= nh_.subscribe<geometry_msgs::Pose>(_topic_pose_robot_ee[LEFT], 1, boost::bind(&dual_arm_control::updateEEPoseCallback,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	_sub_ee_velo[LEFT] 			 	= nh_.subscribe<geometry_msgs::Twist>("/simo_track/robot_left/ee_velo", 1, boost::bind(&dual_arm_control::updateEETwistCallback,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	_subForceTorqueSensor[LEFT] 	= nh_.subscribe<geometry_msgs::WrenchStamped>(_topic_subForceTorqueSensor[LEFT], 1, boost::bind(&dual_arm_control::updateRobotWrench,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	// _subForceTorqueSensor[LEFT]	= nh_.subscribe(_topic_subForceTorqueSensor[LEFT],1, &dual_arm_control::updateRobotWrenchLeft, this, ros::TransportHints().reliable().tcpNoDelay());
	_sub_base_pose[RIGHT] 		 	= nh_.subscribe<geometry_msgs::Pose>(_topic_pose_robot_base[RIGHT], 1, boost::bind(&dual_arm_control::updateBasePoseCallback,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	_sub_ee_pose[RIGHT]   		 	= nh_.subscribe<geometry_msgs::Pose>(_topic_pose_robot_ee[RIGHT], 1, boost::bind(&dual_arm_control::updateEEPoseCallback,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	_sub_ee_velo[RIGHT] 		 	= nh_.subscribe<geometry_msgs::Twist>("/simo_track/robot_right/ee_velo", 1, boost::bind(&dual_arm_control::updateEETwistCallback,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	_subForceTorqueSensor[RIGHT]	= nh_.subscribe<geometry_msgs::WrenchStamped>(_topic_subForceTorqueSensor[RIGHT], 1, boost::bind(&dual_arm_control::updateRobotWrench,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	// _subForceTorqueSensor[RIGHT]	= nh_.subscribe(_topic_subForceTorqueSensor[RIGHT],1, &dual_arm_control::updateRobotWrenchRight, this, ros::TransportHints().reliable().tcpNoDelay());
	//-------------
	// Publishers:
	//-------------
	_pub_ts_commands[LEFT]    		= nh_.advertise<std_msgs::Float64MultiArray>(_topic_ee_commands[LEFT], 1);														// commands
	_pubDesiredTwist[LEFT] 		  	= nh_.advertise<geometry_msgs::Twist>("/dual_arm_control/robot_left/desired/ee_velocity", 1);
	_pubDesiredOrientation[LEFT]	= nh_.advertise<geometry_msgs::Quaternion>("/dual_arm_control/robot_left/desired/ee_orientation", 1);
	_pubFilteredWrench[LEFT] 	  	= nh_.advertise<geometry_msgs::WrenchStamped>("/dual_arm_control/robot_left/filteredWrenchLeft", 1);
	_pubNormalForce[LEFT] 		  	= nh_.advertise<std_msgs::Float64>("/dual_arm_control/robot_left/normalForceLeft", 1);

	_pub_ts_commands[RIGHT]   		= nh_.advertise<std_msgs::Float64MultiArray>(_topic_ee_commands[RIGHT], 1);														// commands
	_pubDesiredTwist[RIGHT] 	 	= nh_.advertise<geometry_msgs::Twist>("/dual_arm_control/robot_right/desired/ee_velocity", 1);
	_pubDesiredOrientation[RIGHT]	= nh_.advertise<geometry_msgs::Quaternion>("/dual_arm_control/robot_right/desired/ee_orientation", 1);
	_pubFilteredWrench[RIGHT] 	 	= nh_.advertise<geometry_msgs::WrenchStamped>("/dual_arm_control/robot_right/filteredWrenc0hRight", 1);
	_pubNormalForce[RIGHT] 		 	= nh_.advertise<std_msgs::Float64>("/dual_arm_control/robot_right/normalForceRight", 1);

	// Desired command for the dual iiwa toolkit
	_pubDesiredVel_Quat[LEFT]   	= nh_.advertise<geometry_msgs::Pose>("/passive_control/iiwa1/vel_quat", 1);
	_pubDesiredVel_Quat[RIGHT]  	= nh_.advertise<geometry_msgs::Pose>("/passive_control/iiwa_blue/vel_quat", 1);

	_pubDistAttractorEe[LEFT] 		= nh_.advertise<std_msgs::Float64>("/passive_control/iiwa1/error", 1);
	_pubDistAttractorEe[RIGHT] 		= nh_.advertise<std_msgs::Float64>("/passive_control/iiwa_blue/error", 1);
  	_pubAttractor[LEFT] 			= nh_.advertise<geometry_msgs::Pose>("/passive_control/iiwa1/attractor", 1);
	_pubAttractor[RIGHT] 			= nh_.advertise<geometry_msgs::Pose>("/passive_control/iiwa_blue/attractor", 1);

	//
	// std::string data_number = _DataID;	
	auto now = std::chrono::system_clock::now();
	auto in_time_t = std::chrono::system_clock::to_time_t(now);
	std::stringstream ss;
	ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d-%X");
	_DataID = ss.str();
	_OutRecord_pose.open(ros::package::getPath(std::string("dual_arm_control"))+"/Data/log_task_pose_"+_DataID+".txt");
	_OutRecord_velo.open(ros::package::getPath(std::string("dual_arm_control"))+"/Data/log_task_velo_"+_DataID+".txt");
	_OutRecord_efforts.open(ros::package::getPath(std::string("dual_arm_control"))+"/Data/log_robot_efforts_"+_DataID+".txt");

  if(!_OutRecord_pose.is_open() && !_OutRecord_velo.is_open() && !_OutRecord_efforts.is_open())
  {
    ROS_ERROR("[ObjectGrasping]: Cannot open output data files, the data_grasping directory might be missing");
    return false;
  }

	// signal(SIGINT,dual_arm_control::stopNode);
  	// 
	_w_H_eeStandby[LEFT]  = Utils<float>::pose2HomoMx(_x[LEFT],  _q[LEFT]);			// WITH EE pose wrt. the world TO BE CHANGED
	_w_H_eeStandby[RIGHT] = Utils<float>::pose2HomoMx(_x[RIGHT],  _q[RIGHT]);		// WITH EE pose wrt. the world

	// initialize the Free motion generator DS
	//------------------------------------------
	FreeMotionCtrl.init(_w_H_eeStandby, this->_gain_abs, this->_gain_rel);
	// initialize the cooperative controller
	//---------------------------------------
	CooperativeCtrl.init();
	// initialize desired pose for the object
	//----------------------------------------
	_w_H_Do = Utils<float>::pose2HomoMx(_xo, _qo);

	//
	Eigen::Vector3f releasePos    = Eigen::Vector3f(0.65, 0.00, 0.50);
	Eigen::Vector4f releaseOrient = Eigen::Vector4f(1.00, 0.00, 0.00, 0.00);

	Eigen::Vector3f releaseLinVel = Eigen::Vector3f(0.50, 0.00, 0.50);
	Eigen::Vector3f releaseAngVel = Eigen::Vector3f(0.00, 0.00, 0.00);
	Eigen::Vector3f restPos 	  	= Eigen::Vector3f(0.50, 0.15, 0.30);
	Eigen::Vector4f restOrient	  = Eigen::Vector4f(1.00, 0.00, 0.00, 0.00);
	// Current state of the robot End-Effector
	Eigen::Vector3f curPos      	= Eigen::Vector3f(0.45, 0.10, 0.30);
	Eigen::Vector4f curOrient			= Eigen::Vector4f(0.9239, 0.00, 0.3827, 0.00); // Eigen::Vector4d(1.00, 0.00, 0.00, 0.00);
	Eigen::Vector3f curLinVel 		= Eigen::Vector3f(0.00, 0.00, 0.00);
	Eigen::Vector3f curAngVel			= Eigen::Vector3f(0.00, 0.00, 0.00);

	//
	dsThrowing.init(dsThrowing.ds_param_, releasePos, releaseOrient, releaseLinVel, releaseAngVel, restPos, restOrient);

	// prepare for reading the keyboard keys
  	nonblock_2(1);  // 
	// /////////////////////////////////////////////////////////////////////////////////////////////
	_t0_run = ros::Time::now().toSec();
	//
	if (nh_.ok()) { 
		// Wait for poses being published
		ros::spinOnce();
		ROS_INFO("[dual_arm_control]: The object grabbing node is ready.");
		return true;
	}
	else {
		ROS_ERROR("[dual_arm_control]: The ros node has a problem.");
		return false;
	}
	
  return true;
}

// void dual_arm_control::stopNode(int sig)
// {
//   me->_stop = true;
// }

//
void dual_arm_control::run() 
{

	ROS_INFO("Running the dual_arm_control");

	bool stream = true;

	_t0_run = ros::Time::now().toSec();
	// --------------------------------------------------------
	// 
	// --------------------------------------------------------
	while (nh_.ok()) {
		//
		if(khbit_2() != 0){
	        char c = fgetc(stdin);
	        fflush(stdin);

	        switch(c){
				case 'q':{   
				            _goHome = !_goHome;
				            if(_goHome){_goToAttractors = true;}
				        }
				break;
				// control of the object (desired using keyboard)
				// position
				case 'a': _delta_pos(0) -= 0.01f; break;
				case 's': _delta_pos(0) += 0.01f; break;
				case 'd': _delta_pos(1) -= 0.01f; break;
				case 'f': _delta_pos(1) += 0.01f; break;
				case 'z': _delta_pos(2) -= 0.01f; break;
				case 'w': _delta_pos(2) += 0.01f; break;
				//orientation
				case 'h': _delta_ang(0) -= 0.05f; break;
				case 'j': _delta_ang(0) += 0.05f; break;
				case 'k': _delta_ang(1) -= 0.05f; break;
				case 'l': _delta_ang(1) += 0.05f; break;
				case 'm': _delta_ang(2) -= 0.05f; break;
				case 'i': _delta_ang(2) += 0.05f; break;  

				// user control of release or throwing (keyboard)
				case 'r': _releaseAndretract = !_releaseAndretract; break; 
				case 't': _isThrowing = !_isThrowing; break;  
				// case 'g': _goToAttractors = !_goToAttractors; break;  
				case 'g': {
						_goToAttractors = !_goToAttractors;
						if(_goToAttractors){
							_goHome			   = false;
							_releaseAndretract = false; 
						}
				}
				case 'p': _isPlacing = !_isPlacing; break;    
			}
	    }
	    // ----------------------------------------------------
		// get the first eigen value of the passive ds controller and Check for update of passive ds controller eigen value
		std::vector<float> param_values;
		ros::param::getCached("/iiwa1/CustomControllers/controllers/PassiveDS/params", param_values);	  _d1[LEFT]  = param_values[0];
		if(_d1[LEFT] <FLT_EPSILON) _d1[LEFT]   = 300.0f;
		ros::param::getCached("/iiwa_blue/CustomControllers/controllers/PassiveDS/params", param_values); _d1[RIGHT] = param_values[0];
		if(_d1[RIGHT]<FLT_EPSILON)  _d1[RIGHT] = 300.0f;

	    _mutex.lock();
			// update the poses of the robots and the object
			updatePoses();
			// compute generated desired motion and forces
			computeCommands();
			// publish the commands to be exectued
			publish_commands();
			//
			publishData();
			//
	    _mutex.unlock();

			ros::spinOnce();
			loop_rate_.sleep();
	}

	// Send zero command
	for(int k = 0; k < NB_ROBOTS; k++)
	{
		_vd[k].setZero();
		_omegad[k].setZero();
		_qd[k] = _q[k];
	}

	publish_commands();
	ros::spinOnce();
	loop_rate_.sleep();
	//
	nonblock_2(0);
	//
	_OutRecord_pose.close();
	_OutRecord_velo.close();
	_OutRecord_efforts.close();

	ros::shutdown();
}

void dual_arm_control::updatePoses()
{
	
	if(_initPoseCount < 20){
		_w_H_eeStandby[LEFT]  = _w_H_rb[LEFT]  * Utils<float>::pose2HomoMx(_xrbStandby[LEFT],  _qrbStandby[LEFT]);			// WITH EE pose wrt. the world TO BE CHANGED
		_w_H_eeStandby[RIGHT] = _w_H_rb[RIGHT] * Utils<float>::pose2HomoMx(_xrbStandby[RIGHT],  _qrbStandby[RIGHT]);		// WITH EE pose wrt. the world

		FreeMotionCtrl._w_H_eeStandby[LEFT]  = _w_H_eeStandby[LEFT];
		FreeMotionCtrl._w_H_eeStandby[RIGHT] = _w_H_eeStandby[RIGHT];
		_xDo    = Eigen::Vector3f(_xo(0), _xo(1), 0.50f);   // set attractor of lifting task
		_qDo    = _qo; 
		_w_H_Do = Utils<float>::pose2HomoMx(_xDo, _qDo);

		_initPoseCount ++;
	}
	// 
	if( _sensedContact && CooperativeCtrl._ContactConfidence == 1.0 && _xo(2) >= 0.98f*_xDo(2)) {
		// _isThrowing = true;
	}
	/////////////////////////////////////////////////////////////////////////////////////////////
	// desired tossing position with respect to the robot
	float dtoss_attrac  = 2.5f;
	float _toss_dir     = 0.0f*M_PI/4.0f;  // TO DO: should come from computed throwing_param 
	float _toss_angle   = M_PI/6.0f;  // TO DO: should come from computed throwing_param 
	// Test of throwing task
	if(_isThrowing){
		_delta_oDx   = 0.90f*_delta_oDx + 0.10f * dtoss_attrac;
		_delta_oDy   = 0.90f*_delta_oDy + 0.10f * dtoss_attrac * std::tan(_toss_dir);
		_delta_oDz   = 0.90f*_delta_oDz + 0.10f * dtoss_attrac * std::tan(_toss_angle);
		
		// update the attractor for tossing direction 
		_w_H_Do(0,3) = _xDo(0) + _delta_oDx;
		_w_H_Do(1,3) = _xDo(1) + _delta_oDy;
		_w_H_Do(2,3) = _xDo(2) + _delta_oDz;
		if(_w_H_o(0,3) >= 0.6f){     // specify a limit in x based on reachable space (TO DO: replace with learned function)
			_releaseAndretract = true;
			_isThrowing        = false;
			_w_H_Do(0,3) = _xDo(0);
			_w_H_Do(1,3) = _xDo(1);
			_w_H_Do(2,3) = _xDo(2);
			//
			_delta_oDx = 0.f;
			_delta_oDy = 0.f;
			_delta_oDz = 0.f;
		}
	}

	if(_isPlacing){
		_xDo    = Eigen::Vector3f(0.68f, 0.00f, 0.35f);   // set attractor of lifting task
		_w_H_Do = Utils<float>::pose2HomoMx(_xDo, _qDo);
		if((_w_H_o.block<3,1>(0,3)-_w_H_Do.block<3,1>(0,3)).norm()<=0.05){
			_releaseAndretract = true;
		}
		else{
			_releaseAndretract = false;
		}
	}

	/////////////////////////////////////////////////////////////////////////////////////////////
	// Update trajectory of the object
	// Update the object position or its desired position (attractor) through keyboard 
	if(_objCtrlKey && false){
		this->Keyboard_object_control(); 
	}
	else{
		this->Keyboard_reference_object_control();
	}

	// homogeneous transformations associated with the reaching task
	for(int k=0; k<NB_ROBOTS; k++){
		// _w_H_ee[k]  = _w_H_rb[k]  * Utils<float>::pose2HomoMx(_x[k],  _q[k]);		// with ee pose wrt. the robot base
		_w_H_ee[k]  = Utils<float>::pose2HomoMx(_x[k],  _q[k]);			// WITH EE pose wrt. the world
		_w_H_gp[k]  = _w_H_o * Utils<float>::pose2HomoMx(_xgp_o[k],  _qgp_o[k]);
		_n[k]       = _w_H_gp[k].block(0,0,3,3).col(2);
		_w_H_Dgp[k] = _w_H_gp[k];
		_err[k]     = (_w_H_ee[k].block(0,3,3,1)  - _w_H_gp[k].block(0,3,3,1)).norm();

		// _BasisQ[k]  = Utils<float>::create3dOrthonormalMatrixFromVector(_desVimp *_n[k]);

		if(CooperativeCtrl._ContactConfidence == 1.0){
			_o_H_ee[k]  = _w_H_o.inverse() * _w_H_ee[k];
			// _o_H_ee[k]  = Utils<float>::pose2HomoMx(_xgp_o[k],  _qgp_o[k]);
			_o_H_ee[k](1,3) *= 0.92f; 
			_w_H_Dgp[k]  = _w_H_Do * _o_H_ee[k];
			if(!_isThrowing){
				_w_H_Dgp[k].block(0,0,3,3)  = _w_H_Do.block(0,0,3,3) * Utils<float>::pose2HomoMx(_xgp_o[k],  _qgp_o[k]).block(0,0,3,3);
				}
		}
	}

	// Compute errors to object center position and dimension vector
	Eigen::Matrix4f le_H_re     =  _w_H_ee[LEFT].inverse() * _w_H_ee[RIGHT];
	Eigen::Matrix4f lgp_H_rgp   =  _w_H_gp[LEFT].inverse() * _w_H_gp[RIGHT];
	Eigen::Vector3f t_o_absEE   =   _w_H_o.block(0,3,3,1) - 0.5f*( _w_H_ee[LEFT].block(0,3,3,1) +  _w_H_ee[RIGHT].block(0,3,3,1));
	_eoD = fabs(le_H_re(2,3)) - fabs(lgp_H_rgp(2,3)); //(_xD-_xoD).dot(_xoD.normalized());
	_eoC = t_o_absEE.norm(); //(_xoC-_xC).norm();
  	//
	for(int k = 0; k <NB_ROBOTS; k++)
	{
		_normalForce[k] = fabs((_wRb[k]*_filteredWrench[k].segment(0,3)).dot(_n[k]));
	}
	// printing for analysis
	std::cout << "[dual_arm_control]: _w_H_o: \n" <<  _w_H_o << std::endl;
	std::cout << "[dual_arm_control]: _w_H_Do: \n" <<  _w_H_Do << std::endl;
	// std::cout << "[dual_arm_control]: _w_H_Dgp[LEFT]: \n" <<  _w_H_Dgp[LEFT] << std::endl;
	// std::cout << "[dual_arm_control]: _w_H_Dgp[RIGHT]: \n" <<  _w_H_Dgp[RIGHT] << std::endl;  
	// std::cout << "[dual_arm_control]: _xDo: \t" <<  _xDo.transpose() << std::endl;
	// std::cout << "[dual_arm_control]: _qDo: \n" <<  _qDo.transpose() << std::endl;
	std::cout << "[dual_arm_control]: _w_H_ee[LEFT]: \n" <<  _w_H_ee[0] << std::endl;
	std::cout << "[dual_arm_control]: _w_H_gp[LEFT]: \n" <<  _w_H_gp[0] << std::endl;
	std::cout << "[dual_arm_control]: _w_H_ee[RIGHT]: \n" << _w_H_ee[1] << std::endl;
	std::cout << "[dual_arm_control]: _w_H_gp[RIGHT]: \n" << _w_H_gp[1] << std::endl;
	// std::cout << "[dual_arm_control]: _w_H_eeStandby[LEFT]: \n" <<  _w_H_eeStandby[0] << std::endl;
	// std::cout << "[dual_arm_control]: _w_H_eeStandby[RIGHT]: \n" << _w_H_eeStandby[1] << std::endl;
	std::cout << "[dual_arm_control]:  ddddddddddddddddd  _eoD: \t" << _eoD << std::endl;
	std::cout << "[dual_arm_control]:  ddddddddddddddddd  _eoC: \t" << _eoC << std::endl;
	// std::cout << "[dual_arm_control]: _releaseAndretract: \n" <<  _releaseAndretract << std::endl;
	// std::cout << " ------------------------------------------------------------------- " << std::endl;
	// std::cout << "[dual_arm_control]: normal to Surface[LEFT]: \t" << _n[0].transpose() << std::endl;
	// std::cout << "[dual_arm_control]: normal to Surface[RIGHT]: \t" << _n[1].transpose() << std::endl;
	// std::cout << "[dual_arm_control]: _normalForce[LEFT]: \t" << _normalForce[0] << std::endl;
	// std::cout << "[dual_arm_control]: _normalForce[RIGHT]: \t" << _normalForce[1] << std::endl;
	std::cout << " ------------------------------------------------------------------- " << std::endl;
	std::cout << "[dual_arm_control]: FFFFFFF  filteredWrench[LEFT]: \t"  << _filteredWrench[0].transpose() << std::endl;
	std::cout << "[dual_arm_control]: FFFFFFF  filteredWrench[RIGHT]: \t" << _filteredWrench[1].transpose() << std::endl;
}
void dual_arm_control::computeCommands()
{
	// Update contact state
  updateContactState();
	// check contact
	_sensedContact = (fabs(_normalForce[LEFT]) >= _forceThreshold) && (fabs(_normalForce[RIGHT]) >= _forceThreshold) && (_c == 1.0f);
	//
	Vector6f desired_object_wrench_; 
	desired_object_wrench_.setZero();
	desired_object_wrench_.head(3) = -40.0f * (_w_H_o.block(0,3,3,1) - _w_H_Do.block(0,3,3,1)) - _objectMass * _gravity;
	//
	Eigen::Matrix3f R1[NB_ROBOTS];
	Eigen::Matrix3f R0[NB_ROBOTS];
	Eigen::Vector3f dirImp[NB_ROBOTS];

	float gamma_friction = M_PI/12.f;
	float rho = 0.10f;
	

	if(_goHome){
		// FreeMotionCtrl.computeCoordinatedMotion(_w_H_ee, _w_H_eeStandby, _w_H_o, _Vd_ee, _qd, true);
		FreeMotionCtrl.computeAsyncMotion(_w_H_ee, _w_H_eeStandby, _w_H_o, _Vd_ee, _qd, true);		
		for(int i=0; i<NB_ROBOTS; i++){
			_V_gpo[i].setZero();
			_fxc[i].setZero();
			//
			Utils<float>::Orthobasis(desired_object_wrench_.head(3), _n[i], R1[i], R0[i]);
			dirImp[i] = R0[i] * Eigen::Vector3f(rho*std::cos(gamma_friction), 0.0f, rho*std::sin(gamma_friction));
			_BasisQ[i]  = Utils<float>::create3dOrthonormalMatrixFromVector(dirImp[i]);
			// _E_xt_xd[i] = R0[i];
			// _BasisQ[i]  = Utils<float>::create3dOrthonormalMatrixFromVector(_desVimp *_n[i]);


			std::cout << "[dual_arm_control]:  ------------- dirImp[i]: \n" <<  dirImp[i].transpose() << std::endl;
			// std::cout << "[dual_arm_control]: ------------- R1[RIGHT]: \n" <<  R1[i] << std::endl;
		}
		_nu_Wr0 = _nu_Wr1 = 0.0f;
	}
	else 
	{
		if((ros::Time::now().toSec()-_t0_run) < 0.2f){
			FreeMotionCtrl.computeAsyncMotion(_w_H_ee, _w_H_gp, _w_H_o, _Vd_ee, _qd, true);
		}
		else
		{
			if(_releaseAndretract)
			{
				FreeMotionCtrl.computeReleaseAndRetractMotion(_w_H_ee, _w_H_Dgp,  _w_H_o, _Vd_ee, _qd, false);
			}
			else if(true && _sensedContact && CooperativeCtrl._ContactConfidence == 1.0f){
				// FreeMotionCtrl.computeConstrainedMotion(_w_H_ee, _w_H_Dgp,  _w_H_o, _Vd_ee, _qd, false);

				Eigen::Vector3f VdImp[NB_ROBOTS]; 
				VdImp[LEFT]  = _desVimp * _n[LEFT];
				VdImp[RIGHT] = _desVimp * _n[RIGHT];
				//
				Vector6f Vd_o = Eigen::VectorXf::Zero(6);

				// _w_H_Dgp[LEFT] = _w_H_gp[LEFT];
				// _w_H_Dgp[RIGHT] = _w_H_gp[RIGHT];
				FreeMotionCtrl.dual_arm_motion(_w_H_ee,  _Vee, _w_H_Dgp,  _w_H_o, _w_H_Do, Vd_o, _BasisQ, VdImp, false, 2, _Vd_ee, _qd, _release_flag);
				float via_height = 0.2f;
				if(_isPlacing){
							FreeMotionCtrl.generatePlacingMotion(_w_H_ee, _w_H_Dgp,  _w_H_o, _w_H_Do, via_height, _Vd_ee, _qd, false);
				}
				if(_isThrowing){
				  _desVtoss   = 0.90f*_desVtoss + 0.10f * _v_max;
				  // _Vd_ee[LEFT].head(3)  = _Vd_ee[LEFT].head(3)/(_Vd_ee[LEFT].head(3).norm()+1e-10)  *  _desVtoss;
  				// _Vd_ee[RIGHT].head(3) = _Vd_ee[RIGHT].head(3)/(_Vd_ee[RIGHT].head(3).norm()+1e-10)*  _desVtoss;
  				//
  				// Vd_o
  				// Call this function and get the desired acceleration or velocity if is2ndOrder is true or not
					Vector6f Vd_o  = dsThrowing.apply(_xo, _qo, _vo, Eigen::Vector3f(0.0f, 0.0f, 0.0f));  // Function to call in a loop
					// Call this to get the release flag
					_releaseAndretract = dsThrowing.get_release_flag();

  				//
  				FreeMotionCtrl.dual_arm_motion(_w_H_ee,  _Vee, _w_H_Dgp,  _w_H_o, _w_H_Do, Vd_o, _BasisQ, VdImp, false, 3, _Vd_ee, _qd, _release_flag);
  				//
				}
			}
			else
			{
				// FreeMotionCtrl.computeCoordinatedMotion(_w_H_ee, _w_H_gp, _w_H_o, _Vd_ee, _qd, false);  // nominal
				//
				// Eigen::Matrix3f BasisQ[NB_ROBOTS];
				Eigen::Vector3f VdImp[NB_ROBOTS]; 
				VdImp[LEFT]  = _desVimp * _n[LEFT];
				VdImp[RIGHT] = _desVimp * _n[RIGHT];
				//
				Vector6f Vd_o = Eigen::VectorXf::Zero(6);
				//
				FreeMotionCtrl.dual_arm_motion(_w_H_ee,  _Vee, _w_H_gp,  _w_H_o, _w_H_Do, Vd_o, _BasisQ, VdImp, false, 1, _Vd_ee, _qd, _release_flag);
			
					Eigen::Vector3f error_p_abs     = _w_H_o.block(0,3,3,1) - 0.5f*( _w_H_ee[LEFT].block(0,3,3,1) +  _w_H_ee[RIGHT].block(0,3,3,1));
					Eigen::Vector3f o_error_pos_abs = _w_H_o.block<3,3>(0,0).transpose() * error_p_abs;
			    Eigen::Vector3f o_error_pos_abs_paral = Eigen::Vector3f(o_error_pos_abs(0), 0.0f, o_error_pos_abs(2));
			    float cp_ap = Utils<float>::computeCouplingFactor(o_error_pos_abs_paral, 50.0f, 0.15f, 1.0f, true);  // 50.0f, 0.05f, 2.8f
				  //  	// create impact in the normal direction
				  // _Vd_ee[LEFT].head(3)  = _Vd_ee[LEFT].head(3)  + _n[LEFT]  * cp_ap  * _desVimp;
  				// _Vd_ee[RIGHT].head(3) = _Vd_ee[RIGHT].head(3) + _n[RIGHT] * cp_ap  * _desVimp;
			}
		}
		// keep the current orientation if not going to the attractor
		if(!_goToAttractors){
			_qd[0]    = _q[0];
			_qd[1]    = _q[1];
		}
		// compute the object's grasp points velocity
		getGraspPointsVelocity();
		//
		//
	  CooperativeCtrl.check_contact_proximity(_w_H_ee, _w_H_gp);
	  // Compute desired contact force profile
	  // -------------------------------------
	  // using QP based wrench generation
	  if(_qp_wrench_generation){
	  	CooperativeCtrl.computeControlWrench(_w_H_o, _w_H_ee, _w_H_gp, desired_object_wrench_);
	  }
	  else{
	  	// using Predefined normal forcve
	  	CooperativeCtrl.getPredefinedContactForceProfile(_goHome, _contactState, _w_H_o, _w_H_ee, _w_H_gp);
	  }	
	  
	  // conatct depend function between [0 1]
	  if(_sensedContact && CooperativeCtrl._ContactConfidence == 1.0f)
	  {
	  	_nu_Wr0 = 0.80f*_nu_Wr0 + 0.20f;
	  	_nu_Wr1 = 0.92f*_nu_Wr1 + 0.08f;
	  }
	  else
	  {
	  	_nu_Wr0 = 0.0f;
	  	_nu_Wr1 = 0.0f;
	  }

	  // applied force in velocity space
	  for(int i=0; i<NB_ROBOTS; i++){
	  	_fxc[i] = 1.0f/_d1[i] * CooperativeCtrl._f_applied[i].head(3);
	  }
	} 
	  // Extract linear velocity commands and desired axis angle command
		prepareCommands(_Vd_ee, _qd, _V_gpo);

		if(_releaseAndretract){
			_fxc[0].setZero();
			_fxc[1].setZero();
			_nu_Wr0 = 0.0f;
	  		_nu_Wr1 = 0.0f;
		}
		//
		if(_goToAttractors)
		{
			_applyVelo = 1.f;
		}
		else
		{
			_applyVelo = 0.0f;
			_qd[LEFT]  = _q[LEFT];
			_qd[RIGHT] = _q[RIGHT];
		}

	  // set the command to send
	  for(int i=0; i<NB_ROBOTS; i++)
	  {
		// _vd[i]  = (1.0f-_nu_Wr0) *_vd[i] + _nu_Wr0 * (_fxc[i]);
	  	// _vd[i]  = _applyVelo * _vd[i] + _nu_Wr0 * _fxc[i];
	  	_vd[i] = (1.0f - _nu_Wr0) * _applyVelo *_vd[i] + _nu_Wr0 * (_fxc[i] + 1.0f * _nu_Wr1 * _vd[i]);
		// _vd[i]  = _vd[i] + (_fxc[i] + 0.0f*_vd[i]);
		std::cout << " tttttttttttttttttttttttttttttttttttttttttttttttttttttt " <<  std::endl;
	  }

	std::cout << "[dual_arm_control]: _Vd_ee[LEFT]:  \t" << _Vd_ee[LEFT].transpose() << std::endl;
	std::cout << "[dual_arm_control]: _Vd_ee[RIGHT]: \t" << _Vd_ee[RIGHT].transpose() << std::endl;
	std::cout << " vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv " <<  std::endl;
	std::cout << "[dual_arm_control]: _vd[LEFT]:  \t" << _vd[LEFT].transpose() << std::endl;
	std::cout << "[dual_arm_control]: _vd[RIGHT]: \t" << _vd[RIGHT].transpose() << std::endl;
	std::cout << " ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ " <<  std::endl;
	std::cout << " COMPUTED HAND WRENCH _fxc  LEFT \t " << _fxc[LEFT].transpose() << std::endl;
	std::cout << " COMPUTED HAND WRENCH _fxc RIGHT \t " << _fxc[RIGHT].transpose() << std::endl;
}
void dual_arm_control::publish_commands()
{
	//
	geometry_msgs::Pose vel_quat[NB_ROBOTS];
	//
	for(int k=0; k<NB_ROBOTS; k++)
	{
		_pubVelo[k].data.clear();
		_pubVelo[k].data.push_back(_aad[k](0));		// axis angle pose_x	
		_pubVelo[k].data.push_back(_aad[k](1));		// axis angle pose_y
		_pubVelo[k].data.push_back(_aad[k](2));		// axis angle pose_z
		_pubVelo[k].data.push_back(_vd[k](0));		// linear velocity v_x
		_pubVelo[k].data.push_back(_vd[k](1));		// linear velocity v_y
		_pubVelo[k].data.push_back(_vd[k](2));		// linear velocity v_z
	  	// desired velocity
		vel_quat[k].position.x		= _vd[k](0);
		vel_quat[k].position.y		= _vd[k](1);
		vel_quat[k].position.z		= _vd[k](2);
		// desired pose
		vel_quat[k].orientation.w	= _qd[k](0);
		vel_quat[k].orientation.x	= _qd[k](1);
		vel_quat[k].orientation.y	= _qd[k](2);
		vel_quat[k].orientation.z	= _qd[k](3);
	}
	//
	_pub_ts_commands[LEFT].publish(_pubVelo[LEFT]);
	_pub_ts_commands[RIGHT].publish(_pubVelo[RIGHT]);

	_pubDesiredVel_Quat[LEFT].publish(vel_quat[LEFT]);
	_pubDesiredVel_Quat[RIGHT].publish(vel_quat[RIGHT]);
}

// current_object_pose_world_callback
// -------------------------------------
void dual_arm_control::objectPoseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	_xo << msg->position.x, 	msg->position.y, 	msg->position.z;
	// filtered object position
	SGF::Vec temp(3);
    _xo_filtered->AddData(_xo);
    _xo_filtered->GetOutput(0,temp);
    _xo = temp;
    _xo_filtered->GetOutput(1,temp);
    _vo = temp;

	_qo << msg->orientation.w, 	msg->orientation.x, msg->orientation.y, msg->orientation.z;
	_w_H_o = Utils<float>::pose2HomoMx(_xo, _qo);
}

void dual_arm_control::updateBasePoseCallback(const geometry_msgs::Pose::ConstPtr& msg, int k)
{
	Eigen::Vector4f q;
	_w_H_rb[k].block(0,3,3,1) << msg->position.x, 	msg->position.y, 	msg->position.z;
	q << msg->orientation.w, 	msg->orientation.x, msg->orientation.y, msg->orientation.z;
	_w_H_rb[k].block(0,0,3,3) = Utils<float>::quaternionToRotationMatrix(q);
}

void dual_arm_control::updateEEPoseCallback(const geometry_msgs::Pose::ConstPtr& msg, int k)
{
	// Update end effecotr pose (position+orientation)
	_x[k] << 	msg->position.x,	msg->position.y, 	msg->position.z;						// position
	_q[k] << 	msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;	// orientation
	//
	_wRb[k] = Utils<float>::quaternionToRotationMatrix(_q[k]);
	_x[k]   = _x[k]+_toolOffsetFromEE[k]*_wRb[k].col(2);
}

void dual_arm_control::updateEETwistCallback(const geometry_msgs::Twist::ConstPtr& msg, int k)
{
  _v[k] << msg->linear.x, msg->linear.y, msg->linear.z;
  _w[k] << msg->angular.x, msg->angular.y, msg->angular.z;
	_Vee[k].head(3)  = _v[k];
	_Vee[k].tail(3)  = _w[k];
}

void dual_arm_control::updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg, int k)
{
	Eigen::Matrix<float,6,1> raw;
	raw(0) = msg->wrench.force.x;
	raw(1) = msg->wrench.force.y;
	raw(2) = msg->wrench.force.z;
	raw(3) = msg->wrench.torque.x;
	raw(4) = msg->wrench.torque.y;
	raw(5) = msg->wrench.torque.z;

  // _wrench[k] = raw -_wrenchBias[k];
  // _filteredWrench[k] = _filteredForceGain*_filteredWrench[k]+(1.0f-_filteredForceGain)*_wrench[k];
  // if(!_wrenchBiasOK[k] && _firstRobotPose[k])
  if(!_wrenchBiasOK[k])
  {
    Eigen::Vector3f loadForce = _wRb[k].transpose()*_toolMass*_gravity;
    _wrenchBias[k].segment(0,3) -= loadForce;
    _wrenchBias[k].segment(3,3) -= _toolComPositionFromSensor.cross(loadForce);
    _wrenchBias[k] += raw; 
    _wrenchCount[k]++;
    if(_wrenchCount[k]==NB_FT_SENSOR_SAMPLES)
    {
      _wrenchBias[k] /= NB_FT_SENSOR_SAMPLES;
      _wrenchBiasOK[k] = true;
      std::cerr << "[dual_arm_control]: Bias " << k << ": " <<_wrenchBias[k].transpose() << std::endl;
    }
  }

  // if(_wrenchBiasOK[k] && _firstRobotPose[k])
  if(_wrenchBiasOK[k])
  // if(true)
  {
    _wrench[k] = raw -_wrenchBias[k];
    Eigen::Vector3f loadForce = _wRb[k].transpose()*_toolMass*_gravity;
    _wrench[k].segment(0,3) -= loadForce;
    _wrench[k].segment(3,3) -= _toolComPositionFromSensor.cross(loadForce);
    _filteredWrench[k] = _filteredForceGain*_filteredWrench[k]+(1.0f-_filteredForceGain)*_wrench[k];
  }
}

void dual_arm_control::updateRobotWrenchLeft(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
	Eigen::Matrix<float,6,1> raw;
	raw(0) = msg->wrench.force.x;
	raw(1) = msg->wrench.force.y;
	raw(2) = msg->wrench.force.z;
	raw(3) = msg->wrench.torque.x;
	raw(4) = msg->wrench.torque.y;
	raw(5) = msg->wrench.torque.z;

	_wrench[LEFT] = raw -_wrenchBias[LEFT];
	_filteredWrench[LEFT] = _filteredForceGain*_filteredWrench[LEFT]+(1.0f-_filteredForceGain)*_wrench[LEFT];
	// _filteredWrench[LEFT] = raw;
}

void dual_arm_control::updateRobotWrenchRight(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
	Eigen::Matrix<float,6,1> raw;
	raw(0) = msg->wrench.force.x;
	raw(1) = msg->wrench.force.y;
	raw(2) = msg->wrench.force.z;
	raw(3) = msg->wrench.torque.x;
	raw(4) = msg->wrench.torque.y;
	raw(5) = msg->wrench.torque.z;

	_wrench[RIGHT] = raw -_wrenchBias[RIGHT];
	_filteredWrench[RIGHT] = _filteredForceGain*_filteredWrench[RIGHT]+(1.0f-_filteredForceGain)*_wrench[RIGHT];
	// _filteredWrench[RIGHT] = raw;
}

void dual_arm_control::updateContactState()
{
  for(int k = 0; k < NB_ROBOTS; k++)
  {
    if(_normalForceWindow[k].size()<MOVING_FORCE_WINDOW_SIZE)
    {
      _normalForceWindow[k].push_back(_normalForce[k]);
      _normalForceAverage[k] = 0.0f;
    }
    else
    {
      _normalForceWindow[k].pop_front();
      _normalForceWindow[k].push_back(_normalForce[k]);
      _normalForceAverage[k] = 0.0f;
      for(int m = 0; m < MOVING_FORCE_WINDOW_SIZE; m++)
      {
        _normalForceAverage[k]+=_normalForceWindow[k][m];
      }
      _normalForceAverage[k] /= MOVING_FORCE_WINDOW_SIZE;
    }
  }
  // if(_normalForceAverage[LEFT] > 2.5f && _normalForceAverage[RIGHT] > 2.5f &&  _eoD < 0.05f && _eoC < 0.1f)
  if(_normalForceAverage[LEFT] > 2.0f && _normalForceAverage[RIGHT] > 2.5f &&  _eoD < 0.05f && _eoC < 0.05f)
  {
    _contactState = CONTACT;
    _c = 1.0f;
  }
  // else if(!(_normalForceAverage[LEFT] > 2.5f && _normalForceAverage[RIGHT] > 2.5f) && _eoD < 0.05f && _eoC < 0.1f)
  else if(!(_normalForceAverage[LEFT] > 2.0f && _normalForceAverage[RIGHT] > 2.5f) && _eoD < 0.05f && _eoC < 0.05f)
  {
    _contactState = CLOSE_TO_CONTACT;
    _c = 0.0f;
  }
  else
  {
    _contactState = NO_CONTACT;
    _c = 0.0f;
  }

  std::cerr << "[dual_arm_control]: contact state: " << (int) _contactState << " c: " << _c << std::endl;
  std::cerr << "[dual_arm_control]: _normalForceAverage[LEFT]: " << _normalForceAverage[LEFT] << std::endl;
  std::cerr << "[dual_arm_control]: _normalForceAverage[RIGHT]: " << _normalForceAverage[RIGHT] << std::endl;
}


void dual_arm_control::prepareCommands(Vector6f Vd_ee[], Eigen::Vector4f qd[], Vector6f V_gpo[])
{
	Eigen::Matrix<float,3,1> axis_d[NB_ROBOTS];	float angle_d[NB_ROBOTS];
  for(int i=0; i<NB_ROBOTS; i++)
  {
  	_vd[i]      = Vd_ee[i].head(3)  + _V_gpo[i].head(3);  
  	_omegad[i]  = Vd_ee[i].tail(3)  + _V_gpo[i].tail(3);   
  	Utils<float>::quaternionToAxisAngle(qd[i], axis_d[i], angle_d[i]);
  	_aad[i] = angle_d[i] * axis_d[i];
  }
}


void dual_arm_control::getGraspPointsVelocity()
{
  //velocity of grasp points on the object
  for(int i=0; i<NB_ROBOTS; i++)
  {
    Eigen::Vector3f t = _w_H_o.block<3,3>(0,0) * _xgp_o[i];
    Eigen::Matrix3f skew_Mx_gpo;
    skew_Mx_gpo <<   0.0f,  -t(2),     t(1),
                     t(2),   0.0f,    -t(0),
                    -t(1),   t(0),     0.0f;             
    // velocity
    _V_gpo[i].head(3) = _vo - skew_Mx_gpo * _wo;
    _V_gpo[i].tail(3) = _wo;
    //
    _V_gpo[i].head(3) *= 0.0f;
    _V_gpo[i].tail(3) *= 0.0f;
  }
}

void dual_arm_control::Keyboard_reference_object_control()   // control of attractor position through keyboad
{
    _w_H_Do(0,3) += _delta_pos(0);
    _w_H_Do(1,3) += _delta_pos(1);
    _w_H_Do(2,3) += _delta_pos(2);
    _delta_pos(0) = 0.0;
    _delta_pos(1) = 0.0;
    _delta_pos(2) = 0.0;
}

// //
void dual_arm_control::Keyboard_object_control()		// control of object position through keyboad
{
	// object2grasp.States_Object.pose.head(3) = ioSM->w_H_absF.block<3,3>(0,0) * init_obj_aF + ioSM->w_H_absF.block<3,1>(0,3); //
	_w_H_o(0,3) += _delta_pos(0);
	_w_H_o(1,3) += _delta_pos(1);
	_w_H_o(2,3) += _delta_pos(2); // _delta_ang

  Eigen::Vector3f ang_o = Utils<float>::getEulerAnglesXYZ_FixedFrame(_w_H_o.block<3,3>(0,0));
  // 
  ang_o += _delta_ang;
  _w_H_o.block<3,3>(0,0) = Utils<float>::eulerAnglesToRotationMatrix(	ang_o(0), ang_o(1), ang_o(2));

}

void dual_arm_control::publishData()
{
  for(int k = 0; k < NB_ROBOTS; k++)
  {
    
    // Publish desired twist
    geometry_msgs::Twist msgDesiredTwist;
    msgDesiredTwist.linear.x  = _vd[k](0);
    msgDesiredTwist.linear.y  = _vd[k](1);
    msgDesiredTwist.linear.z  = _vd[k](2);
    // Convert desired end effector frame angular velocity to world frame
    msgDesiredTwist.angular.x = _omegad[k](0);
    msgDesiredTwist.angular.y = _omegad[k](1);
    msgDesiredTwist.angular.z = _omegad[k](2);
    _pubDesiredTwist[k].publish(msgDesiredTwist);
    // Publish desired orientation
    geometry_msgs::Quaternion msgDesiredOrientation;
    msgDesiredOrientation.w = _qd[k](0);
    msgDesiredOrientation.x = _qd[k](1);
    msgDesiredOrientation.y = _qd[k](2);
    msgDesiredOrientation.z = _qd[k](3);
    _pubDesiredOrientation[k].publish(msgDesiredOrientation);
    // filtered wrench
    geometry_msgs::WrenchStamped msgFilteredWrench;
    msgFilteredWrench.header.frame_id = "world";
    msgFilteredWrench.header.stamp = ros::Time::now();
    msgFilteredWrench.wrench.force.x  = _filteredWrench[k](0);
    msgFilteredWrench.wrench.force.y  = _filteredWrench[k](1);
    msgFilteredWrench.wrench.force.z  = _filteredWrench[k](2);
    msgFilteredWrench.wrench.torque.x = _filteredWrench[k](3);
    msgFilteredWrench.wrench.torque.y = _filteredWrench[k](4);
    msgFilteredWrench.wrench.torque.z = _filteredWrench[k](5);
    _pubFilteredWrench[k].publish(msgFilteredWrench);

    std_msgs::Float64 msg;
    msg.data = _normalForce[k];
    _pubNormalForce[k].publish(msg); 
    //
    msg.data = _err[k];
		_pubDistAttractorEe[k].publish(msg);
		geometry_msgs::Pose msgPose;
		msgPose.position.x    = _w_H_gp[k](0,3);
		msgPose.position.y    = _w_H_gp[k](1,3);
		msgPose.position.z    = _w_H_gp[k](2,3);
		Eigen::Matrix3f Rgr   = _w_H_gp[k].block(0,0,3,3); 
		Eigen::Quaternionf qgr(Rgr);
		msgPose.orientation.x = qgr.x();
		msgPose.orientation.y = qgr.y();
		msgPose.orientation.z = qgr.z();
		msgPose.orientation.w = qgr.w();

		_pubAttractor[k].publish(msgPose);
  }
}

void dual_arm_control::saveData()
{
	_OutRecord_pose		<< _x[LEFT].transpose() << "	" << _q[LEFT].transpose() << "	";
	_OutRecord_pose   << _x[RIGHT].transpose() << "	" << _q[RIGHT].transpose() << "	";
	_OutRecord_pose   << _xo.transpose() << "	" << _qo.transpose() << std::endl;

	_OutRecord_velo		<< _Vd_ee[LEFT].transpose() << "	" << _Vd_ee[RIGHT].transpose() << "	";
	_OutRecord_velo		<< _v[LEFT].transpose() << "  " << _w[LEFT].transpose() << " ";
	_OutRecord_velo		<< _v[RIGHT].transpose() << "  " << _w[RIGHT].transpose() << std::endl;

	_OutRecord_efforts	<< _filteredWrench[LEFT].transpose() << "  " << _filteredWrench[RIGHT].transpose() << std::endl;
}