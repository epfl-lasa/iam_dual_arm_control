
#include "dual_arm_control.h"
#include "Utils.h"

using namespace std;
using namespace Eigen;

dual_arm_control::dual_arm_control(ros::NodeHandle &n, double frequency, 	std::string topic_pose_object_,
																																					std::string topic_pose_robot_base_left_,
																																					std::string topic_pose_robot_ee_left_,
																																					std::string topic_ee_commands_left_,
																																					std::string topic_pose_robot_base_right_,
																																					std::string topic_pose_robot_ee_right_,
																																					std::string topic_ee_commands_right_)	: nh_(n)
																																																								, loop_rate_(frequency)
{
  //
  // me = this;
  _stop = false;
  //
	_topic_pose_object   					  = topic_pose_object_;
	_topic_pose_robot_base_left   	= topic_pose_robot_base_left_;				
	_topic_pose_robot_ee_left   		= topic_pose_robot_ee_left_;			
	_topic_ee_commands_left   			= topic_ee_commands_left_;		
	_topic_pose_robot_base_right   	= topic_pose_robot_base_right_;				
	_topic_pose_robot_ee_right   		= topic_pose_robot_ee_right_; 			
	_topic_ee_commands_right   			= topic_ee_commands_right_;		

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
    _x[k].setConstant(0.0f);
    _q[k].setConstant(0.0f);
    _d1[k] = 1.0f;
    _wRb[k].setIdentity();
    
    _xd[k].setConstant(0.0f);
    _vd[k].setConstant(0.0f);
    _omegad[k].setConstant(0.0f);
    _qd[k].setConstant(0.0f);
    _aad[k].setZero();
    _Vd_ee[k].setZero();
    
    _firstRobotPose[k] = false;
    _firstRobotTwist[k] = false;
    _firstWrenchReceived[k] = false;
    _wrenchCount[k] = 0;
    _normalForce[k] = 0.0f;
    _normalForceAverage[k] = 0.0f;
    _wrenchBiasOK[k] = false;

    _w_H_ee[k].setConstant(0.0f);
    _w_H_eeStandby[k].setConstant(0.0f);
    _w_H_rb[k].setIdentity();
    _w_H_gp[k].setConstant(0.0f);
    _xgp_o[k].setConstant(0.0f);
    _qgp_o[k].setConstant(0.0f);
    _V_gpo[k].setConstant(0.0f);

    _fxc[k].setZero();
		_Fd[k] = 0.0f;
		_err[k] = 1.0f;
		_filteredWrench[k].setZero();
		_wrench[k].setZero();
		_wrenchBias[k].setZero();

	  _o_H_ee[k].setIdentity();
	  _w_H_Dgp[k].setIdentity();

    //
    pubVelo[k].data.clear();
    pubVelo[k].data.push_back(0.0);		// axis angle poses _x	
    pubVelo[k].data.push_back(0.0);		// axis angle poses _y
    pubVelo[k].data.push_back(0.0);		// axis angle poses _z
    pubVelo[k].data.push_back(0.0);		// linear velocity v_x
    pubVelo[k].data.push_back(0.0);		// linear velocity v_y
    pubVelo[k].data.push_back(0.0);		// linear velocity v_z

  }
  //
	_eD = 0.0f;
	_eoD = 0.0f;
	_eC = 0.0f;
	_eoC = 0.0f;
	_c = 0.0f;
	_goHome = false;
	_nuWrench = 0.0f;

  // 
  _xDo.setZero(); 
	_qDo.setZero();
	_w_H_Do.setZero();
	//
	// _xgp_o[0] << 0.0f, -_objectDim(1)/2.0f, 0.0f;  // left
	// _xgp_o[1] << 0.0f,  _objectDim(1)/2.0f,  0.0f; // right
	_xgp_o[0] << 0.0f, -_objectDim(1)/1.8f, 0.0f;  // left
	_xgp_o[1] << 0.0f,  _objectDim(1)/1.8f,  0.0f; // right

	Eigen::Matrix3f o_R_gpl, o_R_gpr;		o_R_gpl.setZero();		o_R_gpr.setZero();	
	//
	o_R_gpl(0,0) =  1.0f;	o_R_gpl(2,1) = -1.0f; 	o_R_gpl(1,2) = 1.0f;
	o_R_gpr(0,0) = 1.0f;	o_R_gpr(2,1) =  1.0f; 	o_R_gpr(1,2) =-1.0f;
	// o_R_gpr(0,0) = -1.0f;	o_R_gpr(2,1) = -1.0f; 	o_R_gpr(1,2) =-1.0f; 
	// o_R_gpr(2,0) = -1.0f;	o_R_gpr(0,1) =  1.0f; 	o_R_gpr(1,2) =-1.0f; 
	_n[LEFT]  = o_R_gpl.col(2);
	_n[RIGHT] = o_R_gpr.col(2);

	_qgp_o[0] = Utils<float>::rotationMatrixToQuaternion(o_R_gpl); //
	_qgp_o[1] = Utils<float>::rotationMatrixToQuaternion(o_R_gpr); //

	_xo.setConstant(0.0f);
	_qo << 1.0f, 0.0f, 0.0f, 0.0f;
	_w_H_o = Utils<float>::pose2HomoMx(_xo, _qo);	

	_vo.setConstant(0.0f);
	_wo.setConstant(0.0f);

	_v_abs.setZero();
	_w_abs.setZero();
	_v_rel.setZero();
	_w_rel.setZero();

	_ep_abs.setConstant(0.0f);
	_eo_abs.setConstant(0.0f);
	_ep_rel.setConstant(0.0f);
	_eo_rel.setConstant(0.0f);

	_gain_abs.setZero();
	_gain_abs.topLeftCorner(3,3)     = Eigen::Vector3f(2.0f, 2.0f, 2.0f).asDiagonal();
	_gain_abs.bottomRightCorner(3,3) = Eigen::Vector3f(1.0f, 1.0f, 1.0f).asDiagonal();
	_gain_rel.setZero();
	_gain_rel.topLeftCorner(3,3)     = Eigen::Vector3f(4.0f, 4.0f, 4.0f).asDiagonal();
	_gain_rel.bottomRightCorner(3,3) = Eigen::Vector3f(1.5f, 1.5f, 1.5f).asDiagonal();

	reachable_p = 1.0f;
	reachable_o = 1.0f;
	_v_max = 0.7f;
	_w_max = 2.0f;
	_targetForce = 20.0f;
	_filteredForceGain = 0.9f;
	_sensedContact     = false;
	_forceThreshold    = 5.0f;
	t0_run = ros::Time::now().toSec();

}

bool dual_arm_control::init()
{
	// Subscribers
	sub_object_pose 	= nh_.subscribe(_topic_pose_object,1, &dual_arm_control::objectPoseCallback, this, ros::TransportHints().reliable().tcpNoDelay());
	sub_left_base_pose 	= nh_.subscribe<geometry_msgs::Pose>(_topic_pose_robot_base_left, 1, boost::bind(&dual_arm_control::updateBasePoseCallback,this,_1,LEFT), 
																		 							ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	sub_left_ee_pose 	= nh_.subscribe<geometry_msgs::Pose>(_topic_pose_robot_ee_left, 1, boost::bind(&dual_arm_control::updateEEPoseCallback,this,_1,LEFT), 
																		 							ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	_subForceTorqueSensor[LEFT] = nh_.subscribe<geometry_msgs::WrenchStamped>("/iiwa1/iiwa1_FTS_topic", 1, boost::bind(&dual_arm_control::updateRobotWrench,this,_1,LEFT),
																									ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

	sub_right_base_pose = nh_.subscribe<geometry_msgs::Pose>(_topic_pose_robot_base_right, 1, boost::bind(&dual_arm_control::updateBasePoseCallback,this,_1,RIGHT), 
																		 							ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	sub_right_ee_pose   = nh_.subscribe<geometry_msgs::Pose>(_topic_pose_robot_ee_right, 1, boost::bind(&dual_arm_control::updateEEPoseCallback,this,_1,RIGHT), 
																		 							ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

	_subForceTorqueSensor[RIGHT] = nh_.subscribe<geometry_msgs::WrenchStamped>("/iiwa_blue/iiwa_blue_FTS_topic", 1, boost::bind(&dual_arm_control::updateRobotWrench,this,_1,RIGHT),
																									ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  // sub_left_joint_states	= nh_.subscribe("/iiwa1/joint_states", 200, &dual_arm_control::larmJointStatesCallback, this, ros::TransportHints().reliable().tcpNoDelay());
	// sub_right_joint_states  = nh_.subscribe("/iiwa_blue/joint_states", 200, &dual_arm_control::rarmJointStatesCallback, this, ros::TransportHints().reliable().tcpNoDelay());
	// names of topics to be published
	// Publishers:
	pub_left_ts_commands    = nh_.advertise<std_msgs::Float64MultiArray>(_topic_ee_commands_left, 1);
	pub_right_ts_commands   = nh_.advertise<std_msgs::Float64MultiArray>(_topic_ee_commands_right, 1);

	//
	// signal(SIGINT,dual_arm_control::stopNode);
  //
	_w_H_eeStandby[LEFT]  = Utils<float>::pose2HomoMx(_x[LEFT],  _q[LEFT]);			// WITH EE pose wrt. the world TO BE CHANGED
	_w_H_eeStandby[RIGHT] = Utils<float>::pose2HomoMx(_x[RIGHT],  _q[RIGHT]);		// WITH EE pose wrt. the world
	
	// initialize the Free motion generator DS
	FreeMotionCtrl.init(_w_H_eeStandby, this->_gain_abs, this->_gain_rel);
	// initialize the cooperative controller
	CooperativeCtrl.init();
	//
	_xDo = _xo;
	_xDo(2) = 0.60f;
	_qDo = _qo; 
	_w_H_Do = Utils<float>::pose2HomoMx(_xDo, _qDo);


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

dual_arm_control::~dual_arm_control(){}

// void dual_arm_control::stopNode(int sig)
// {
//   me->_stop = true;
// }

//
void dual_arm_control::run() 
{

	ROS_INFO("Running the dual_arm_control");

	bool stream = true;

	t0_run = ros::Time::now().toSec();

	while (nh_.ok()) 
	{
		// get the first eigen value of the passive ds controller
		// Check for update of passive ds controller eigen value
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
	// homogeneous transformation of the EE
  // _w_H_ee[LEFT]  = _w_H_rb[LEFT]  * Utils<float>::pose2HomoMx(_x[LEFT],  _q[LEFT]);		// with ee pose wrt. the robot base
  // _w_H_ee[RIGHT] = _w_H_rb[RIGHT] * Utils<float>::pose2HomoMx(_x[RIGHT], _q[RIGHT]);		// with ee pose wrt. the robot base
  _w_H_ee[LEFT]  = Utils<float>::pose2HomoMx(_x[LEFT],  _q[LEFT]);			// WITH EE pose wrt. the world
  _w_H_ee[RIGHT] = Utils<float>::pose2HomoMx(_x[RIGHT], _q[RIGHT]);			// WITH EE pose wrt. the world

  // world transformation of grasp points
  _w_H_o 				 = Utils<float>::pose2HomoMx(_xo, _qo);	
  _w_H_gp[LEFT]  = _w_H_o * Utils<float>::pose2HomoMx(_xgp_o[LEFT],  _qgp_o[LEFT]);
  _w_H_gp[RIGHT] = _w_H_o * Utils<float>::pose2HomoMx(_xgp_o[RIGHT], _qgp_o[RIGHT]);
  _n[LEFT]   =  _w_H_gp[LEFT].block(0,0,3,3).col(2);
  _n[RIGHT]  =  _w_H_gp[RIGHT].block(0,0,3,3).col(2);

  _w_H_Dgp[LEFT]  = _w_H_gp[LEFT];
  _w_H_Dgp[RIGHT] = _w_H_gp[RIGHT];

  _w_H_Do = Utils<float>::pose2HomoMx(_xDo, _qDo);
  if(CooperativeCtrl._ContactConfidence == 1.0)
  {
  	_o_H_ee[LEFT]  = _w_H_o.inverse() * _w_H_ee[LEFT];
  	_o_H_ee[RIGHT] = _w_H_o.inverse() * _w_H_ee[RIGHT];
  	// _o_H_ee[LEFT]  = Utils<float>::pose2HomoMx(_xgp_o[LEFT],  _qgp_o[LEFT]);
  	// _o_H_ee[RIGHT] = Utils<float>::pose2HomoMx(_xgp_o[RIGHT], _qgp_o[RIGHT]);
  	_o_H_ee[LEFT](1,3)  *= 0.90f; 
		_o_H_ee[RIGHT](1,3) *= 0.90f;

  	_w_H_Dgp[LEFT]  = _w_H_Do * _o_H_ee[LEFT];
  	_w_H_Dgp[RIGHT] = _w_H_Do * _o_H_ee[RIGHT];
  }

  _err[LEFT]  = (_w_H_ee[LEFT].block(0,3,3,1)  - _w_H_gp[LEFT].block(0,3,3,1)).norm();
  _err[RIGHT] = (_w_H_ee[RIGHT].block(0,3,3,1) - _w_H_gp[RIGHT].block(0,3,3,1)).norm();

  // Compute errors to object center position and dimension vector
  Eigen::Matrix4f le_H_re   =  _w_H_ee[LEFT].inverse() * _w_H_ee[RIGHT];
  Eigen::Vector3f t_o_absEE =   _w_H_o.block(0,3,3,1) - 0.5f*( _w_H_ee[LEFT].block(0,3,3,1) +  _w_H_ee[RIGHT].block(0,3,3,1));
  _eoD = fabs(le_H_re(2,3)); //(_xD-_xoD).dot(_xoD.normalized());
  _eoC = t_o_absEE.norm(); //(_xoC-_xC).norm();

  // /////////////////////////////////////////////////////////////////////////////////////
  // =====================================
  //velocity of grasp points on the object
  // =====================================
  // /////////////////////////////////////////////////////////////////////////////////////
  Matrix6f veloTwistMx = Eigen::MatrixXf::Identity(6,6);
  Eigen::Vector3f t[NB_ROBOTS]; 
  Eigen::Matrix4f o_H_gpo[NB_ROBOTS];

  for(int i=0; i<NB_ROBOTS; i++)
  {
    t[i] = _w_H_o.block<3,3>(0,0) * _xgp_o[i];
    Eigen::Matrix3f skew_Mx_gpo;
    skew_Mx_gpo <<      0.0f,  -t[i](2),     t[i](1),
                     t[i](2),      0.0f,    -t[i](0),
                    -t[i](1),   t[i](0),        0.0f;             
    // velocity
    _V_gpo[i].head(3) = _vo - skew_Mx_gpo * _wo;
    _V_gpo[i].tail(3) = _wo;
  }

  //
  for(int k = 0; k <NB_ROBOTS; k++)
  {
    _normalForce[k] = fabs((_wRb[k]*_filteredWrench[k].segment(0,3)).dot(_n[k]));
  }

  std::cout << "[dual_arm_control]: _w_H_o: \n" <<  _w_H_o << std::endl;
	// std::cout << "[dual_arm_control]: _w_H_ee[LEFT]: \n" <<  _w_H_ee[0] << std::endl;
	// std::cout << "[dual_arm_control]: _w_H_ee[RIGHT]: \n" << _w_H_ee[1] << std::endl;
	// std::cout << "[dual_arm_control]: _w_H_gp[LEFT]: \n" <<  _w_H_gp[0] << std::endl;
	// std::cout << "[dual_arm_control]: _w_H_gp[RIGHT]: \n" << _w_H_gp[1] << std::endl;

	std::cout << "[dual_arm_control]: _eoD: \t" << _eoD << std::endl;
	std::cout << "[dual_arm_control]: _eoC: \t" << _eoC << std::endl;

	std::cout << "[dual_arm_control]: normal to Surface[LEFT]: \t" << _n[0].transpose() << std::endl;
	std::cout << "[dual_arm_control]: normal to Surface[RIGHT]: \t" << _n[1].transpose() << std::endl;

	std::cout << "[dual_arm_control]: filteredWrench[LEFT]: \t"  << _filteredWrench[0].transpose() << std::endl;
	std::cout << "[dual_arm_control]: filteredWrench[RIGHT]: \t" << _filteredWrench[1].transpose() << std::endl;

	std::cout << "[dual_arm_control]: DESIRED FORCE[LEFT]: \t"  << _Fd[0] << std::endl;
	std::cout << "[dual_arm_control]: DESIRED FORCE[RIGHT]: \t" << _Fd[1] << std::endl;
}
void dual_arm_control::computeCommands()
{
	//
	// Update contact state
  updateContactState();
	// check contact
	_sensedContact = (fabs(_filteredWrench[LEFT](2)) >= _forceThreshold) || (fabs(_filteredWrench[RIGHT](2)) >= _forceThreshold);

	if((ros::Time::now().toSec()-t0_run) < 0.2f)
	{
		_xDo    = _xo;
		_xDo(2) = 0.60f;
		_qDo    = _qo; 
		_w_H_Do = Utils<float>::pose2HomoMx(_xDo, _qDo);
		//
		FreeMotionCtrl.computeAsyncMotion(_w_H_ee, _w_H_gp, _Vd_ee, _qd);
	}
	else
	{
		if(CooperativeCtrl._ContactConfidence == 1.0f)
		// if(_sensedContact)
		{
			// FreeMotionCtrl.computeCoordinatedMotion(_w_H_ee, _w_H_gp, _Vd_ee, _qd);
			// FreeMotionCtrl.computeCoordinatedMotion(_w_H_ee, _w_H_Dgp, _Vd_ee, _qd);
			FreeMotionCtrl.computeConstrainedMotion(_w_H_ee, _w_H_Dgp, _Vd_ee, _qd);

		}
		else
		{
			FreeMotionCtrl.computeCoordinatedMotion(_w_H_ee, _w_H_gp, _Vd_ee, _qd);
		}
		
	}
	//
	Eigen::Matrix<float,3,1> axis_d[NB_ROBOTS];	float angle_d[NB_ROBOTS];
  for(int i=0; i<NB_ROBOTS; i++)
  {
  	_vd[i]      = _Vd_ee[i].head(3)  + _V_gpo[i].head(3);  
  	_omegad[i]  = _Vd_ee[i].tail(3)  + _V_gpo[i].tail(3);   
  	Utils<float>::quaternionToAxisAngle(_qd[i], axis_d[i], angle_d[i]);
  	_aad[i] = angle_d[i] * axis_d[i];
  }
  //
  CooperativeCtrl.check_contact_proximity(_w_H_ee, _w_H_gp);
  // Compute desired contact force profile
  // computeDesiredContactForceProfile();

  if(CooperativeCtrl._ContactConfidence == 1.0f)
  {
  	_nuWrench = 0.80f*_nuWrench + 0.20f;
  }
  else
  {
  	_nuWrench = 0.0f;
  }
	  // //
	  // for(int i=0; i<NB_ROBOTS; i++)
	  // {
		// // force test
	  // 	// if(_err[LEFT]< 0.04f &&  _err[RIGHT]< 0.04f)
	  // 	// CooperativeCtrl.check_contact_proximity(_w_H_ee, _w_H_gp);

	  // 	if(CooperativeCtrl._ContactConfidence == 1.0f)
	  // 	// if(_sensedContact)
	  // 	{
		 //  	_Fd[i] = _targetForce;
		 //  }
		 //  // else if(CooperativeCtrl._ContactConfidence == 1.0)
		 //  // {
		 //  // 	_Fd[i] = 10.0f;
		 //  // }
		 //  else
		 //  {
		 //  	_Fd[i] = 0.0f;
		 //  }
	  // }
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

  CooperativeCtrl.computeControlWrench(_w_H_o, _w_H_ee, _w_H_gp, desired_object_wrench_);
  //
  for(int i=0; i<NB_ROBOTS; i++)
  {
	  //
	  _fxc[i] = 1.0f/_d1[i] * CooperativeCtrl._f_applied[i].head(3);
	  //
	  _vd[i]  = (1.0f-_nuWrench) *_vd[i] + _nuWrench * (_fxc[i] + 0.30f*_vd[i]);
  }


  std::cout << "[dual_arm_control]: _Vd_ee[LEFT]:  \n" << _Vd_ee[LEFT].transpose() << std::endl;
  std::cout << "[dual_arm_control]: _Vd_ee[RIGHT]: \n" << _Vd_ee[RIGHT].transpose() << std::endl;
}
void dual_arm_control::publish_commands()
{
	//
	for(int k=0; k<NB_ROBOTS; k++)
	{
		pubVelo[k].data.clear();
	  // pubVelo[k].data.push_back(_omegad[k](0));		// angular velocity omega_x	
	  // pubVelo[k].data.push_back(_omegad[k](1));		// angular velocity omega_y
	  // pubVelo[k].data.push_back(_omegad[k](2));		// angular velocity omega_z

	  pubVelo[k].data.push_back(_aad[k](0));	// axis angle pose_x	
	  pubVelo[k].data.push_back(_aad[k](1));	// axis angle pose_y
	  pubVelo[k].data.push_back(_aad[k](2));	// axis angle pose_z
	  pubVelo[k].data.push_back(_vd[k](0));		// linear velocity v_x
	  pubVelo[k].data.push_back(_vd[k](1));		// linear velocity v_y
	  pubVelo[k].data.push_back(_vd[k](2));		// linear velocity v_z
	}
	//
	pub_left_ts_commands.publish(pubVelo[LEFT]);
	pub_right_ts_commands.publish(pubVelo[RIGHT]);
}

void dual_arm_control::computeDesiredContactForceProfile()
{
  if(_goHome)
  {
    _Fd[LEFT]  = 0.0f;
    _Fd[RIGHT] = 0.0f;
  }
  else
  {
    if(_contactState==CONTACT)
    {
      _Fd[LEFT]  = _targetForce;
      _Fd[RIGHT] = _targetForce;
    }
    else if(_contactState==CLOSE_TO_CONTACT)
    {
      _Fd[LEFT] =  3.0f;
      _Fd[RIGHT] = 3.0f;
    }
    else
    {
      _Fd[LEFT]  = 0.0f;
      _Fd[RIGHT] = 0.0f;
    }
  }
}

// current_object_pose_world_callback
// -------------------------------------
void dual_arm_control::objectPoseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	_xo << msg->position.x, 	msg->position.y, 	msg->position.z;
  _qo << msg->orientation.w, 	msg->orientation.x, msg->orientation.y, msg->orientation.z;
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