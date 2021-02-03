
#include "dual_arm_control.h"
#include "Utils.hpp"

using namespace std;
using namespace Eigen;

dual_arm_control::dual_arm_control(ros::NodeHandle &n, double frequency, 	std::string topic_pose_object_,
																																					std::string topic_pose_robot_base[],
																																					std::string topic_pose_robot_ee[],
																																					std::string topic_ee_commands[],
																																					std::string topic_sub_ForceTorque_Sensor[])	: nh_(n)
																																																											, loop_rate_(frequency)
{
  //
  // me = this;
  _stop = false;
  //
	_topic_pose_object = topic_pose_object_;
	memcpy(_topic_pose_robot_base, &topic_pose_robot_base[0], NB_ROBOTS * sizeof *topic_pose_robot_base); 
	memcpy(_topic_pose_robot_ee, &topic_pose_robot_ee[0], NB_ROBOTS * sizeof *topic_pose_robot_ee); 
	memcpy(_topic_ee_commands, &topic_ee_commands[0], NB_ROBOTS * sizeof *topic_ee_commands); 
	memcpy(_topic_subForceTorqueSensor, &topic_sub_ForceTorque_Sensor[0], NB_ROBOTS * sizeof *topic_sub_ForceTorque_Sensor); 

	//
	_objectMass = 1.0f;
	_objectDim << 0.20f, 0.20f, 0.20f;
	_toolOffsetFromEE = 0.13f;
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
    _w_H_gp[k].setConstant(0.0f);
    // object
    _xgp_o[k].setConstant(0.0f);
    _qgp_o[k].setConstant(0.0f);
    _V_gpo[k].setConstant(0.0f);
    _o_H_ee[k].setIdentity();
    _w_H_Dgp[k].setIdentity();

    // desired values
    _Vd_ee[k].setZero();
    _xd[k].setZero();
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
  }
  // 
  _vo.setZero();
	_wo.setZero();
  _xo.setZero(); 
  _xDo.setZero(); 
	_qo  << 1.0f, 0.0f, 0.0f, 0.0f;
	_qDo << 1.0f, 0.0f, 0.0f, 0.0f;
	_w_H_o  = Utils<float>::pose2HomoMx(_xo, _qo);	
	_w_H_Do = Utils<float>::pose2HomoMx(_xDo, _qDo);	
	_objecPoseCount = 0;

  // object desired grasping points
  _xgp_o[0] << 0.0f, -_objectDim(1)/2.0f,  -0.01f;  	// left   // _xgp_o[0] << 0.0f, -_objectDim(1)/1.8f, 0.0f;  // left
	_xgp_o[1] << 0.0f,  _objectDim(1)/2.0f,  -0.01f; 		// right 	// _xgp_o[1] << 0.0f,  _objectDim(1)/1.8f,  0.0f; // right

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
	_goHome = false;

  // coordination velocity
	_v_abs.setZero();
	_w_abs.setZero();
	_v_rel.setZero();
	_w_rel.setZero();
	_v_max = 0.7f;     // velocity limits
	_w_max = 2.0f;     // velocity limits
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
	_gain_abs.topLeftCorner(3,3)     = Eigen::Vector3f(2.0f, 2.0f, 2.0f).asDiagonal();
	_gain_abs.bottomRightCorner(3,3) = Eigen::Vector3f(1.0f, 1.0f, 1.0f).asDiagonal();
	_gain_rel.setZero();
	_gain_rel.topLeftCorner(3,3)     = Eigen::Vector3f(4.0f, 4.0f, 4.0f).asDiagonal();
	_gain_rel.bottomRightCorner(3,3) = Eigen::Vector3f(1.5f, 1.5f, 1.5f).asDiagonal();

	_reachable = 1.0f;
	_targetForce = 20.0f;
	_filteredForceGain = 0.9f;
	_sensedContact     = false;
	_forceThreshold    = 4.0f;
	_nuWrench 				 = 0.0f;
	//
	// _t0_run = ros::Time::now().toSec();

}
//
dual_arm_control::~dual_arm_control(){}

//
bool dual_arm_control::init()
{
	// Subscribers
	//------------
	_sub_object_pose 			= nh_.subscribe(_topic_pose_object,1, &dual_arm_control::objectPoseCallback, this, ros::TransportHints().reliable().tcpNoDelay());
	_sub_base_pose[LEFT] 	= nh_.subscribe<geometry_msgs::Pose>(_topic_pose_robot_base[LEFT], 1, boost::bind(&dual_arm_control::updateBasePoseCallback,this,_1,LEFT), 
																		 																																							ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	_sub_ee_pose[LEFT] 		= nh_.subscribe<geometry_msgs::Pose>(_topic_pose_robot_ee[LEFT], 1, boost::bind(&dual_arm_control::updateEEPoseCallback,this,_1,LEFT), 
																		 																																							ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	_subForceTorqueSensor[LEFT] = nh_.subscribe<geometry_msgs::WrenchStamped>(_topic_subForceTorqueSensor[LEFT], 1, boost::bind(&dual_arm_control::updateRobotWrench,this,_1,LEFT),
																																																									ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

	// _subForceTorqueSensor[LEFT]	= nh_.subscribe(_topic_subForceTorqueSensor[LEFT],1, &dual_arm_control::updateRobotWrenchLeft, this, ros::TransportHints().reliable().tcpNoDelay());

	_sub_base_pose[RIGHT] = nh_.subscribe<geometry_msgs::Pose>(_topic_pose_robot_base[RIGHT], 1, boost::bind(&dual_arm_control::updateBasePoseCallback,this,_1,RIGHT), 
																		 																																							ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	_sub_ee_pose[RIGHT]   = nh_.subscribe<geometry_msgs::Pose>(_topic_pose_robot_ee[RIGHT], 1, boost::bind(&dual_arm_control::updateEEPoseCallback,this,_1,RIGHT), 
																		 																																							ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	_subForceTorqueSensor[RIGHT] = nh_.subscribe<geometry_msgs::WrenchStamped>(_topic_subForceTorqueSensor[RIGHT], 1, boost::bind(&dual_arm_control::updateRobotWrench,this,_1,RIGHT),
																																																									ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	// _subForceTorqueSensor[RIGHT]	= nh_.subscribe(_topic_subForceTorqueSensor[RIGHT],1, &dual_arm_control::updateRobotWrenchRight, this, ros::TransportHints().reliable().tcpNoDelay());
	// Publishers:
	//-------------
	_pub_ts_commands[LEFT]    = nh_.advertise<std_msgs::Float64MultiArray>(_topic_ee_commands[LEFT], 1);
	_pub_ts_commands[RIGHT]   = nh_.advertise<std_msgs::Float64MultiArray>(_topic_ee_commands[RIGHT], 1);

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
	//
	// define a desired pose for the object
	//--------------------------------------
	// _xDo    = Eigen::Vector3f(_xo(0), _xo(1), 0.60f);
	// _qDo    = _qo; 
	_w_H_Do = Utils<float>::pose2HomoMx(_xo, _qo);
	// /////////////////////////////////////////////////////////////////////////////////////////////
	_t0_run = ros::Time::now().toSec();

	//
	if (nh_.ok()) 
	{ 
		// Wait for poses being published
		ros::spinOnce();
		ROS_INFO("[dual_arm_control]: The object grabbing node is ready.");
		return true;
	}
	else 
	{
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

	while (nh_.ok()) 
	{
		// get the first eigen value of the passive ds controller and Check for update of passive ds controller eigen value
		std::vector<float> param_values;
		ros::param::getCached("/iiwa1/CustomControllers/controllers/PassiveDS/params", param_values);			_d1[LEFT]  = param_values[0];
		if(_d1[LEFT] <FLT_EPSILON) _d1[LEFT]   = 150.0f;
		ros::param::getCached("/iiwa_blue/CustomControllers/controllers/PassiveDS/params", param_values); _d1[RIGHT] = param_values[0];
    if(_d1[RIGHT]<FLT_EPSILON)  _d1[RIGHT] = 150.0f;

		// update the poses of the robots and the object
		updatePoses();
		// compute generated desired motion and forces
		computeCommands();
		// publish the commands to be exectued
    publish_commands();
    //
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

}

void dual_arm_control::updatePoses()
{
	// homogeneous transformations associated with the reaching task
  _w_H_o 				= Utils<float>::pose2HomoMx(_xo, _qo);
  for(int k=0; k<NB_ROBOTS; k++)
  {
  	// _w_H_ee[k]  = _w_H_rb[k]  * Utils<float>::pose2HomoMx(_x[k],  _q[k]);		// with ee pose wrt. the robot base
	  _w_H_ee[k]  = Utils<float>::pose2HomoMx(_x[k],  _q[k]);			// WITH EE pose wrt. the world
	  _w_H_gp[k]  = _w_H_o * Utils<float>::pose2HomoMx(_xgp_o[k],  _qgp_o[k]);
	  _n[k]       = _w_H_gp[k].block(0,0,3,3).col(2);
	  _w_H_Dgp[k] = _w_H_gp[k];
	  //
	  _err[k]  = (_w_H_ee[k].block(0,3,3,1)  - _w_H_gp[k].block(0,3,3,1)).norm();
  }

  if(CooperativeCtrl._ContactConfidence == 1.0)
  {
   	_w_H_Do 			= Utils<float>::pose2HomoMx(_xDo, _qDo);
   	for(int k=0; k<NB_ROBOTS; k++)
  	{
  		_o_H_ee[k]  = _w_H_o.inverse() * _w_H_ee[k];
	  	// _o_H_ee[k]  = Utils<float>::pose2HomoMx(_xgp_o[k],  _qgp_o[k]);
	  	_o_H_ee[k](1,3) *= 0.95f; 
	  	_w_H_Dgp[k]  = _w_H_Do * _o_H_ee[k];
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

  std::cout << "[dual_arm_control]: _w_H_o: \n" <<  _w_H_o << std::endl;
  std::cout << "[dual_arm_control]: _w_H_Do: \n" <<  _w_H_Do << std::endl;
  std::cout << "[dual_arm_control]: _w_H_Dgp[LEFT]: \n" <<  _w_H_Dgp[LEFT] << std::endl;
  std::cout << "[dual_arm_control]: _w_H_Dgp[RIGHT]: \n" <<  _w_H_Dgp[RIGHT] << std::endl;

  std::cout << "[dual_arm_control]: _xDo: \t" <<  _xDo.transpose() << std::endl;
  std::cout << "[dual_arm_control]: _qDo: \n" <<  _qDo.transpose() << std::endl;
	// std::cout << "[dual_arm_control]: _w_H_ee[LEFT]: \n" <<  _w_H_ee[0] << std::endl;
	// std::cout << "[dual_arm_control]: _w_H_ee[RIGHT]: \n" << _w_H_ee[1] << std::endl;
	// std::cout << "[dual_arm_control]: _w_H_gp[LEFT]: \n" <<  _w_H_gp[0] << std::endl;
	// std::cout << "[dual_arm_control]: _w_H_gp[RIGHT]: \n" << _w_H_gp[1] << std::endl;

	std::cout << "[dual_arm_control]:  ddddddddddddddddd  _eoD: \t" << _eoD << std::endl;
	std::cout << "[dual_arm_control]:  ddddddddddddddddd  _eoC: \t" << _eoC << std::endl;

	std::cout << " ------------------------------------------------------------------- " << std::endl;
	std::cout << "[dual_arm_control]: normal to Surface[LEFT]: \t" << _n[0].transpose() << std::endl;
	std::cout << "[dual_arm_control]: normal to Surface[RIGHT]: \t" << _n[1].transpose() << std::endl;
	std::cout << " ------------------------------------------------------------------- " << std::endl;

	std::cout << "[dual_arm_control]: FFFFFFF  filteredWrench[LEFT]: \t"  << _filteredWrench[0].transpose() << std::endl;
	std::cout << "[dual_arm_control]: FFFFFFF  filteredWrench[RIGHT]: \t" << _filteredWrench[1].transpose() << std::endl;

	std::cout << "[dual_arm_control]: DESIRED FORCE[LEFT]: \t"  << _Fd[0] << std::endl;
	std::cout << "[dual_arm_control]: DESIRED FORCE[RIGHT]: \t" << _Fd[1] << std::endl;
}
void dual_arm_control::computeCommands()
{
	// Update contact state
  updateContactState();
	// check contact
	_sensedContact = (fabs(_filteredWrench[LEFT](2)) >= _forceThreshold) && (fabs(_filteredWrench[RIGHT](2)) >= _forceThreshold);
	//
	if((ros::Time::now().toSec()-_t0_run) < 0.2f)
	{
		// define a desired pose for the object
		// _xDo    = Eigen::Vector3f(_xo(0), _xo(1), 0.60f);
		// _qDo    = _qo; 
		// _w_H_Do = Utils<float>::pose2HomoMx(_xDo, _qDo);
		//
		FreeMotionCtrl.computeAsyncMotion(_w_H_ee, _w_H_gp, _Vd_ee, _qd);
	}
	else
	{
		if(true && _sensedContact && CooperativeCtrl._ContactConfidence == 1.0f)
		// if(_sensedContact)
		{
			// FreeMotionCtrl.computeCoordinatedMotion(_w_H_ee, _w_H_gp, _Vd_ee, _qd);
			FreeMotionCtrl.computeCoordinatedMotion(_w_H_ee, _w_H_Dgp, _Vd_ee, _qd);
			// FreeMotionCtrl.computeConstrainedMotion(_w_H_ee, _w_H_Dgp, _Vd_ee, _qd);
		}
		else
		{
			FreeMotionCtrl.computeCoordinatedMotion(_w_H_ee, _w_H_gp, _Vd_ee, _qd);
		}
		
	}
	// compute the object's grasp points velocity
	getGraspPointsVelocity();

	// Extract linear velocity commands and desired axis angle command
	prepareCommands(_Vd_ee, _qd, _V_gpo);
  //
  CooperativeCtrl.check_contact_proximity(_w_H_ee, _w_H_gp);
  // Compute desired contact force profile
  CooperativeCtrl.getPredefinedContactForceProfile(_goHome, _contactState, _w_H_o, _w_H_ee, _w_H_gp);
  // getPredefinedContactForceProfile(bool goHome, ContactState contactState, Eigen::Matrix4f w_H_o, Eigen::Matrix4f w_H_ee[], Eigen::Matrix4f w_H_cp[])

  if(CooperativeCtrl._ContactConfidence == 1.0f)
  {
  	_nuWrench = 0.80f*_nuWrench + 0.20f;
  }
  else
  {
  	_nuWrench = 0.0f;
  }
	  //
	  // for(int i=0; i<NB_ROBOTS; i++)
	  // {
		 //  //
		 //  _fxc[i] = _Fd[i]/_d1[i] * _n[i];
		 //  //
		 //  _vd[i]  = _vd[i] + _fxc[i];
	  // }

  Vector6f desired_object_wrench_; 
  desired_object_wrench_.setZero();
  desired_object_wrench_.head(3) = -20.0f * (_w_H_o.block(0,3,3,1) - _w_H_Do.block(0,3,3,1)) - _objectMass * _gravity;

  // CooperativeCtrl.computeControlWrench(_w_H_o, _w_H_ee, _w_H_gp, desired_object_wrench_);
  //
  for(int i=0; i<NB_ROBOTS; i++)
  {
	  _fxc[i] = 1.0f/_d1[i] * CooperativeCtrl._f_applied[i].head(3);
	  _vd[i]  = (1.0f-_nuWrench) *_vd[i] + _nuWrench * (_fxc[i] + 0.3f*_vd[i]);
	  // _vd[i]  = _vd[i] + (_fxc[i] + 0.0f*_vd[i]);
  }

  std::cout << "[dual_arm_control]: _Vd_ee[LEFT]:  \t" << _Vd_ee[LEFT].transpose() << std::endl;
  std::cout << "[dual_arm_control]: _Vd_ee[RIGHT]: \t" << _Vd_ee[RIGHT].transpose() << std::endl;
  std::cout << " vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv " <<  std::endl;
  std::cout << "[dual_arm_control]: _vd[LEFT]:  \t" << _vd[LEFT].transpose() << std::endl;
  std::cout << "[dual_arm_control]: _vd[RIGHT]: \t" << _vd[RIGHT].transpose() << std::endl;
  std::cout << " ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ " <<  std::endl;
}
void dual_arm_control::publish_commands()
{
	//
	for(int k=0; k<NB_ROBOTS; k++)
	{
		_pubVelo[k].data.clear();
	  // _pubVelo[k].data.push_back(_omegad[k](0));		// angular velocity omega_x	
	  // _pubVelo[k].data.push_back(_omegad[k](1));		// angular velocity omega_y
	  // _pubVelo[k].data.push_back(_omegad[k](2));		// angular velocity omega_z
	  _pubVelo[k].data.push_back(_aad[k](0));	// axis angle pose_x	
	  _pubVelo[k].data.push_back(_aad[k](1));	// axis angle pose_y
	  _pubVelo[k].data.push_back(_aad[k](2));	// axis angle pose_z
	  _pubVelo[k].data.push_back(_vd[k](0));		// linear velocity v_x
	  _pubVelo[k].data.push_back(_vd[k](1));		// linear velocity v_y
	  _pubVelo[k].data.push_back(_vd[k](2));		// linear velocity v_z
	}
	//
	_pub_ts_commands[LEFT].publish(_pubVelo[LEFT]);
	_pub_ts_commands[RIGHT].publish(_pubVelo[RIGHT]);
}

// void dual_arm_control::getPredefinedContactForceProfile()
// {
//   if(_goHome)
//   {
//     _Fd[LEFT]  = 0.0f;
//     _Fd[RIGHT] = 0.0f;
//   }
//   else
//   {
//     if(_contactState==CONTACT)
//     {
//       _Fd[LEFT]  = _targetForce;
//       _Fd[RIGHT] = _targetForce;
//     }
//     else if(_contactState==CLOSE_TO_CONTACT)
//     {
//       _Fd[LEFT] =  3.0f;
//       _Fd[RIGHT] = 3.0f;
//     }
//     else
//     {
//       _Fd[LEFT]  = 0.0f;
//       _Fd[RIGHT] = 0.0f;
//     }
//   }
// }

// current_object_pose_world_callback
// -------------------------------------
void dual_arm_control::objectPoseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	_xo << msg->position.x, 	msg->position.y, 	msg->position.z;
  _qo << msg->orientation.w, 	msg->orientation.x, msg->orientation.y, msg->orientation.z;

  if(_objecPoseCount < 5)
  {
  	_xDo    = Eigen::Vector3f(_xo(0), _xo(1), 0.60f);
		_qDo    = _qo; 
		_objecPoseCount ++;
  }
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
  _x[k]   = _x[k]+_toolOffsetFromEE*_wRb[k].col(2);
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
  // if(_wrenchBiasOK[k])
  if(true)
  {
    _wrench[k] = raw -_wrenchBias[k];
    Eigen::Vector3f loadForce = _wRb[k].transpose()*_toolMass*_gravity;
    _wrench[k].segment(0,3) -= loadForce;
    _wrench[k].segment(3,3) -= _toolComPositionFromSensor.cross(loadForce);
    _filteredWrench[k] = _filteredForceGain*_filteredWrench[k]+(1.0f-_filteredForceGain)*_wrench[k];
  }

  std::cout << "[dual_arm_control]: FFFFFFFF filteredWrench[" << k << "]: \t"  << _filteredWrench[k].transpose() << std::endl;
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

  // _wrench[LEFT] = raw -_wrenchBias[LEFT];
  // _filteredWrench[LEFT] = _filteredForceGain*_filteredWrench[LEFT]+(1.0f-_filteredForceGain)*_wrench[LEFT];
  _filteredWrench[LEFT] = raw;

  std::cout << "[dual_arm_control]: FFFFFFFF filteredWrench[" << LEFT << "]: \t"  << _filteredWrench[LEFT].transpose() << std::endl;
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

  // _wrench[RIGHT] = raw -_wrenchBias[RIGHT];
  // _filteredWrench[RIGHT] = _filteredForceGain*_filteredWrench[RIGHT]+(1.0f-_filteredForceGain)*_wrench[RIGHT];
  _filteredWrench[RIGHT] = raw;

  std::cout << "[dual_arm_control]: FFFFFFFF filteredWrench[" << RIGHT << "]: \t"  << _filteredWrench[RIGHT].transpose() << std::endl;
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

  if(_normalForceAverage[LEFT] > 3.0f && _normalForceAverage[RIGHT] > 3.0f &&  _eoD < 0.05f && _eoC < 0.1f)
  {
    _contactState = CONTACT;
    _c = 1.0f;
  }
  else if(!(_normalForceAverage[LEFT] > 3.0f && _normalForceAverage[RIGHT] > 3.0f) && _eoD < 0.05f && _eoC < 0.1f)
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
  }
}