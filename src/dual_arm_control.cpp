
#include "iam_dual_arm_control/dual_arm_control.h"
#include "iam_dual_arm_control/Utils.hpp"
#include "iam_dual_arm_control/keyboard.h"
#include <chrono>
#include <ctime>

using namespace std;
using namespace Eigen;

const static IOFormat CSVFormat(FullPrecision, DontAlignCols, ", ", "\n");

// ---------------------------------------------------------------------

dual_arm_control::dual_arm_control(	ros::NodeHandle &n, double frequency, 	//std::string dataID,
									std::string topic_pose_object_,
									std::string topic_pose_robot_base[],
									std::string topic_pose_robot_ee[],
									std::string topic_ee_commands[],
									std::string topic_sub_ForceTorque_Sensor[])	: nh_(n)
																				, loop_rate_(frequency)
																				, _dt(1.0f/frequency)
{
	// me = this;
	_cycle_count = 0;
	_stop = false;
	//
	_topic_pose_object = topic_pose_object_;
	memcpy(_topic_pose_robot_base, &topic_pose_robot_base[0], NB_ROBOTS * sizeof *topic_pose_robot_base); 
	memcpy(_topic_pose_robot_ee, &topic_pose_robot_ee[0], NB_ROBOTS * sizeof *topic_pose_robot_ee); 
	memcpy(_topic_ee_commands, &topic_ee_commands[0], NB_ROBOTS * sizeof *topic_ee_commands); 
	memcpy(_topic_subForceTorqueSensor, &topic_sub_ForceTorque_Sensor[0], NB_ROBOTS * sizeof *topic_sub_ForceTorque_Sensor); 
	//
	_gravity << 0.0f, 0.0f, -9.80665f;
	_objectMass = 1.0f;
	_objectDim << 0.26f, 0.26f, 0.26f;
	_toolOffsetFromEE[0] = 0.115f; //0.17f;
	_toolOffsetFromEE[1] = 0.115f; //0.15f;
	_toolMass = 0.2f;    // TO CHANGE !!!!
	_toolComPositionFromSensor << 0.0f,0.0f,0.035f;
	//
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
		_dirImp[k].setZero();
		_VdImpact[k].setZero();
		//
		_joints_positions[k].setZero();
		_joints_velocities[k].setZero();
		_joints_accelerations[k].setZero();
		_joints_torques[k].setZero();
	}
	// 
	// Filtered variable (SG)
	_xo_filtered 			  = std::make_unique<SGF::SavitzkyGolayFilter>(3,3,6,_dt);
	_sgf_ddq_filtered_l = std::make_unique<SGF::SavitzkyGolayFilter>(7,3,6,_dt); // dim, order. window lenght
	_sgf_ddq_filtered_r = std::make_unique<SGF::SavitzkyGolayFilter>(7,3,6,_dt); // dim, order. window lenght
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
	_desired_object_wrench.setZero();

	// object desired grasping points
	_xgp_o[0] << 0.0f, -_objectDim(1)/2.0f,  -0.10f;  // left   
	_xgp_o[1] << 0.0f,  _objectDim(1)/2.0f,  -0.10f; 	// right 	

	Eigen::Matrix3f o_R_gpl;	o_R_gpl.setZero();		
	Eigen::Matrix3f o_R_gpr;	o_R_gpr.setZero();	
	o_R_gpl(0,0) =  1.0f;	o_R_gpl(2,1) = -1.0f; 	o_R_gpl(1,2) = 1.0f;
	o_R_gpr(0,0) =  1.0f;	o_R_gpr(2,1) =  1.0f; 	o_R_gpr(1,2) =-1.0f;
	// o_R_gpr(0,0) = -1.0f;	o_R_gpr(2,1) = -1.0f; 	o_R_gpr(1,2) =-1.0f; 
	// o_R_gpr(2,0) = -1.0f;	o_R_gpr(0,1) =  1.0f; 	o_R_gpr(1,2) =-1.0f; 
	_qgp_o[0] = Utils<float>::rotationMatrixToQuaternion(o_R_gpl); //
	_qgp_o[1] = Utils<float>::rotationMatrixToQuaternion(o_R_gpr); //
	_Vd_o.setZero();

	// normal to contact surfaces
	_n[LEFT]  = o_R_gpl.col(2);
	_n[RIGHT] = o_R_gpr.col(2);

	_c   = 0.0f;
	_v_max = 2.00f;     // velocity limits
	_w_max = 4.0f;     // velocity limits
	// coordination errors
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
	_gain_abs.topLeftCorner(3,3)     = 1.0f * Eigen::Vector3f(1.5f, 1.5f, 1.5f).asDiagonal();  
	_gain_abs.bottomRightCorner(3,3) = 1.0f * Eigen::Vector3f(1.0f, 1.0f, 1.0f).asDiagonal();  
	_gain_rel.setZero();
	_gain_rel.topLeftCorner(3,3)     = 1.f*Eigen::Vector3f(2.0f, 2.0f, 2.0f).asDiagonal();  
	_gain_rel.bottomRightCorner(3,3) = 1.f*Eigen::Vector3f(1.0f, 1.0f, 1.0f).asDiagonal();  

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
	_desVtoss   = 0.5f;
	_applyVelo  = 0.0f;
	_desVimp    = 0.5f;
	_desVreach  = 1.0f;
	_refVreach  = 0.0f;
	_delta_Imp  = 0.0f;
	_delta_Toss = 0.0f;

	_objCtrlKey 	= true;
	_goHome 			= true;
	_goToAttractors 	= true;     // true
	_releaseAndretract 	= false;
	_isThrowing 		= false;
	_isPlacing          = false;
	// _t0_run = ros::Time::now().toSec();
	_release_flag = false;
	_startlogging = false;
	_isPickupSet  = false;
	_dualTaskSelector = 1;
	_old_dual_method  = true;
}
//
dual_arm_control::~dual_arm_control(){}

//
bool dual_arm_control::init()
{
	//-------------
	// Subscribers
	//------------
	_sub_object_pose 			 				= nh_.subscribe(_topic_pose_object,1, &dual_arm_control::objectPoseCallback, this, ros::TransportHints().reliable().tcpNoDelay());
	_sub_base_pose[LEFT] 	 	 			= nh_.subscribe<geometry_msgs::Pose>(_topic_pose_robot_base[LEFT], 1, boost::bind(&dual_arm_control::updateBasePoseCallback,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	_sub_ee_pose[LEFT] 			 			= nh_.subscribe<geometry_msgs::Pose>(_topic_pose_robot_ee[LEFT], 1, boost::bind(&dual_arm_control::updateEEPoseCallback,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	_sub_ee_velo[LEFT] 			 			= nh_.subscribe<geometry_msgs::Twist>("/simo_track/robot_left/ee_velo", 1, boost::bind(&dual_arm_control::updateEETwistCallback,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	_subForceTorqueSensor[LEFT] 	= nh_.subscribe<geometry_msgs::WrenchStamped>(_topic_subForceTorqueSensor[LEFT], 1, boost::bind(&dual_arm_control::updateRobotWrench,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	// _subForceTorqueSensor[LEFT]= nh_.subscribe(_topic_subForceTorqueSensor[LEFT],1, &dual_arm_control::updateRobotWrenchLeft, this, ros::TransportHints().reliable().tcpNoDelay());
	_sub_joint_states[LEFT]	 			= nh_.subscribe<sensor_msgs::JointState>("/iiwa1/joint_states", 1, boost::bind(&dual_arm_control::updateRobotStatesLeft,this,_1), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

	_sub_base_pose[RIGHT] 		 		= nh_.subscribe<geometry_msgs::Pose>(_topic_pose_robot_base[RIGHT], 1, boost::bind(&dual_arm_control::updateBasePoseCallback,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	_sub_ee_pose[RIGHT]   		 		= nh_.subscribe<geometry_msgs::Pose>(_topic_pose_robot_ee[RIGHT], 1, boost::bind(&dual_arm_control::updateEEPoseCallback,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	_sub_ee_velo[RIGHT] 		 			= nh_.subscribe<geometry_msgs::Twist>("/simo_track/robot_right/ee_velo", 1, boost::bind(&dual_arm_control::updateEETwistCallback,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	_subForceTorqueSensor[RIGHT]	= nh_.subscribe<geometry_msgs::WrenchStamped>(_topic_subForceTorqueSensor[RIGHT], 1, boost::bind(&dual_arm_control::updateRobotWrench,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	// _subForceTorqueSensor[RIGHT]	= nh_.subscribe(_topic_subForceTorqueSensor[RIGHT],1, &dual_arm_control::updateRobotWrenchRight, this, ros::TransportHints().reliable().tcpNoDelay());
	_sub_joint_states[RIGHT]	 		= nh_.subscribe<sensor_msgs::JointState>("/iiwa_blue/joint_states", 1, boost::bind(&dual_arm_control::updateRobotStatesRight,this,_1), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	
	//-------------
	// Publishers:
	//-------------
	_pub_ts_commands[LEFT]    		= nh_.advertise<std_msgs::Float64MultiArray>(_topic_ee_commands[LEFT], 1);														// commands
	_pubDesiredOrientation[LEFT]	= nh_.advertise<geometry_msgs::Quaternion>("/dual_arm_control/robot_left/desired/ee_orientation", 1);
	_pubFilteredWrench[LEFT] 	  	= nh_.advertise<geometry_msgs::WrenchStamped>("/dual_arm_control/robot_left/filteredWrenchLeft", 1);
	_pubNormalForce[LEFT] 		  	= nh_.advertise<std_msgs::Float64>("/dual_arm_control/robot_left/normalForceLeft", 1);

	_pub_ts_commands[RIGHT]   		= nh_.advertise<std_msgs::Float64MultiArray>(_topic_ee_commands[RIGHT], 1);														// commands
	_pubDesiredOrientation[RIGHT]	= nh_.advertise<geometry_msgs::Quaternion>("/dual_arm_control/robot_right/desired/ee_orientation", 1);
	_pubFilteredWrench[RIGHT] 	 	= nh_.advertise<geometry_msgs::WrenchStamped>("/dual_arm_control/robot_right/filteredWrenc0hRight", 1);
	_pubNormalForce[RIGHT] 		 		= nh_.advertise<std_msgs::Float64>("/dual_arm_control/robot_right/normalForceRight", 1);

	// Desired command for the dual iiwa toolkit
	_pubDesiredVel_Quat[LEFT]   	= nh_.advertise<geometry_msgs::Pose>("/passive_control/iiwa1/vel_quat", 1);
	_pubDesiredVel_Quat[RIGHT]  	= nh_.advertise<geometry_msgs::Pose>("/passive_control/iiwa_blue/vel_quat", 1);

	// _pubDesiredTwist[LEFT] 		= nh_.advertise<geometry_msgs::Twist>("/dual_arm_control/robot_left/desired/ee_velocity", 1);  // /passive_control/iiwa1/des_twist
	// _pubDesiredTwist[RIGHT] 	 	= nh_.advertise<geometry_msgs::Twist>("/dual_arm_control/robot_right/desired/ee_velocity", 1);
	
	_pubDesiredTwist[LEFT] 		  	= nh_.advertise<geometry_msgs::Twist>("/passive_control/iiwa1/des_twist", 1);  // /
	_pubDesiredTwist[RIGHT] 	 		= nh_.advertise<geometry_msgs::Twist>("/passive_control/iiwa_blue/des_twist", 1);


	_pubDistAttractorEe[LEFT] 		= nh_.advertise<std_msgs::Float64>("/dual_arm_control/iiwa1/error", 1);     				// "/passive_control/iiwa1/error"
	_pubDistAttractorEe[RIGHT] 		= nh_.advertise<std_msgs::Float64>("/dual_arm_control/iiwa_blue/error", 1);   			// "/passive_control/iiwa_blue/error"
  _pubAttractor[LEFT] 					= nh_.advertise<geometry_msgs::Pose>("/dual_arm_control/iiwa1/attractor", 1);				// "/passive_control/iiwa1/attractor"
	_pubAttractor[RIGHT] 					= nh_.advertise<geometry_msgs::Pose>("/dual_arm_control/iiwa_blue/attractor", 1);		// "/passive_control/iiwa_blue/attractor"
	_pubNormLinVel[LEFT]					= nh_.advertise<std_msgs::Float64>("/dual_arm_control/iiwa_left/lin_vel_norm", 1);
	_pubNormLinVel[RIGHT]					= nh_.advertise<std_msgs::Float64>("/dual_arm_control/iiwa_right/lin_vel_norm", 1);
	//
	
	// initialize the tossing DS
	//---------------------------
	float param_toolOffset_left;
	float param_toolOffset_right;

	std::vector<float> param_releasePos;
	std::vector<float> param_releaseOrient;
	std::vector<float> param_releaseLinVel_dir;
	std::vector<float> param_releaseAngVel;
	std::vector<float> param_restPos;
	std::vector<float> param_restOrient;

	std::vector<float> param_stanbyPosition[NB_ROBOTS];
	std::vector<float> param_stanbyOrientation[NB_ROBOTS];
	std::vector<float> param_abs_gains;
	std::vector<float> param_rel_gains;

	std::vector<float> param_xDo_lifting;
	std::vector<float> param_qDo_lifting;
	std::vector<float> param_xDo_placing;
	std::vector<float> param_qDo_placing;
	bool modulated_reaching = true;
	bool isNorm_impact_vel  = false;

	bool gotParam = true;

	gotParam = gotParam && nh_.getParam("tool/offset2end_effector/left",  param_toolOffset_left);
	gotParam = gotParam && nh_.getParam("tool/offset2end_effector/right", param_toolOffset_right);
	gotParam = gotParam && nh_.getParam("desVimp", _desVimp);
	gotParam = gotParam && nh_.getParam("desVreach", _desVreach);
	gotParam = gotParam && nh_.getParam("tossing/desVtoss", _desVtoss);
	gotParam = gotParam && nh_.getParam("tossing/releasePos", param_releasePos);
	gotParam = gotParam && nh_.getParam("tossing/releaseOrient", param_releaseOrient);
	gotParam = gotParam && nh_.getParam("tossing/releaseLinVel_dir", param_releaseLinVel_dir);
	gotParam = gotParam && nh_.getParam("tossing/releaseAngVel", param_releaseAngVel);
	gotParam = gotParam && nh_.getParam("tossing/restPos", param_restPos);
	gotParam = gotParam && nh_.getParam("tossing/restOrient", param_restOrient);

	gotParam = gotParam && nh_.getParam("standby_pose/robot_left/position", param_stanbyPosition[LEFT]);
	gotParam = gotParam && nh_.getParam("standby_pose/robot_left/orientation", param_stanbyOrientation[LEFT]);
	gotParam = gotParam && nh_.getParam("standby_pose/robot_right/position", param_stanbyPosition[RIGHT]);
	gotParam = gotParam && nh_.getParam("standby_pose/robot_right/orientation", param_stanbyOrientation[RIGHT]);

	gotParam = gotParam && nh_.getParam("ds_absolute_gains", param_abs_gains);
	gotParam = gotParam && nh_.getParam("ds_relative_gains", param_rel_gains);
	gotParam = gotParam && nh_.getParam("impact_direction/friction_angle", _friction_angle);
	gotParam = gotParam && nh_.getParam("impact_direction/max_friction_angle", _max_friction_angle); 
	gotParam = gotParam && nh_.getParam("impact_direction/impact_dir_preset", _impact_dir_preset);
	gotParam = gotParam && nh_.getParam("dual_arm_task/dualTaskSelector", _dualTaskSelector); 
	gotParam = gotParam && nh_.getParam("dual_arm_task/lifting/position", param_xDo_lifting);
	gotParam = gotParam && nh_.getParam("dual_arm_task/lifting/orientation", param_qDo_lifting);
	gotParam = gotParam && nh_.getParam("dual_arm_task/placing/position", param_xDo_placing);
	gotParam = gotParam && nh_.getParam("dual_arm_task/placing/orientation", param_qDo_placing); 
	gotParam = gotParam && nh_.getParam("dual_arm_task/placing/height_via_point", _height_via_point); 
	gotParam = gotParam && nh_.getParam("dual_arm_task/old_dual_method", _old_dual_method); 
	gotParam = gotParam && nh_.getParam("dual_arm_task/modulated_reaching", modulated_reaching);  
	gotParam = gotParam && nh_.getParam("dual_arm_task/isNorm_impact_vel", isNorm_impact_vel);
	


	if (!gotParam) {
    ROS_ERROR("Couldn't the retrieve one or many parameters. ");
		return false;
  }

  _toolOffsetFromEE[0]  						= param_toolOffset_left;
  _toolOffsetFromEE[1]  						= param_toolOffset_right;
	_tossVar.release_position      		= Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(param_releasePos.data(), param_releasePos.size());
	_tossVar.release_orientation      = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(param_releaseOrient.data(), param_releaseOrient.size());
	_tossVar.release_linear_velocity  = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(param_releaseLinVel_dir.data(), param_releaseLinVel_dir.size());
	_tossVar.release_angular_velocity = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(param_releaseAngVel.data(), param_releaseAngVel.size());
	_tossVar.rest_position      			= Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(param_restPos.data(), param_restPos.size());
	_tossVar.rest_orientation      		= Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(param_restOrient.data(), param_restOrient.size());

	_xrbStandby[LEFT]  	= Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(param_stanbyPosition[LEFT].data(), param_stanbyPosition[LEFT].size());
	_qrbStandby[LEFT]  	= Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(param_stanbyOrientation[LEFT].data(), param_stanbyOrientation[LEFT].size());
	_xrbStandby[RIGHT] 	= Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(param_stanbyPosition[RIGHT].data(), param_stanbyPosition[RIGHT].size());
	_qrbStandby[RIGHT] 	= Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(param_stanbyOrientation[RIGHT].data(), param_stanbyOrientation[RIGHT].size());
	_gain_abs.diagonal()= Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(param_abs_gains.data(), param_abs_gains.size());
	_gain_rel.diagonal()= Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(param_rel_gains.data(), param_rel_gains.size());

	_xDo_lifting = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(param_xDo_lifting.data(), param_xDo_lifting.size());
	_qDo_lifting = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(param_qDo_lifting.data(), param_qDo_lifting.size());
	_xDo_placing = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(param_xDo_placing.data(), param_xDo_placing.size());
	_qDo_placing = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(param_qDo_placing.data(), param_qDo_placing.size());

	

	
 	// releaseLinVel = (_desVtoss/sqrt(2.0f))*Eigen::Vector3f(1.00, 0.00, 1.00);
 	_tossVar.release_linear_velocity = _desVtoss * _tossVar.release_linear_velocity.normalized();
 	//-------------------------------------------------------------------------------------------------
	// Motion and Force generation: DS
	//-------------------------------------------------------------------------------------------------
	_w_H_Do = Utils<float>::pose2HomoMx(_xo, _qo);
	_w_H_eeStandby[LEFT]  = Utils<float>::pose2HomoMx(_xrbStandby[LEFT],  _qrbStandby[LEFT]);			// WITH EE pose wrt. the world TO BE CHANGED
	_w_H_eeStandby[RIGHT] = Utils<float>::pose2HomoMx(_xrbStandby[RIGHT],  _qrbStandby[RIGHT]);		// WITH EE pose wrt. the world

	// initialize the Free motion generator DS
	//------------------------------------------
	FreeMotionCtrl.init(_w_H_eeStandby, this->_gain_abs, this->_gain_rel);

	// initialize the cooperative controller
	//---------------------------------------
	CooperativeCtrl.init();
	//
	// Object tossing DS
	//------------------
	// dsThrowing.init(dsThrowing.ds_param_, releasePos, releaseOrient, releaseLinVel, releaseAngVel, restPos, restOrient);
	dsThrowing.init(dsThrowing.ds_param_, _tossVar.release_position, _tossVar.release_orientation, 
									_tossVar.release_linear_velocity, _tossVar.release_angular_velocity,
									_tossVar.rest_position, _tossVar.rest_orientation);
	//
	dsThrowing.set_pickup_object_pose(_xo, _qo);
	Eigen::Vector3f new_toss_velocity = _desVtoss * (_tossVar.release_position -_xo).normalized();
	dsThrowing.set_toss_linear_velocity(new_toss_velocity);
	dsThrowing._refVtoss = _desVimp;
	//
	FreeMotionCtrl._desVreach = _desVreach;
	FreeMotionCtrl._refVreach[LEFT]  = _refVreach;
	FreeMotionCtrl._refVreach[RIGHT] = _refVreach;
	FreeMotionCtrl._modulated_reaching = modulated_reaching;
	FreeMotionCtrl._isNorm_impact_vel  = isNorm_impact_vel;
	FreeMotionCtrl._height_via_point = _height_via_point;

	//--------------------------------------------------------------------------------------------------
	// Data recording:
	//--------------------------------------------------------------------------------------------------
	// std::string path2Datafolder = ros::package::getPath(std::string("dual_arm_control")) +"/Data";
	datalog.datalog_init(ros::package::getPath(std::string("dual_arm_control")) +"/Data");

  // signal(SIGINT,dual_arm_control::stopNode);

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
	_t0_run = ros::Time::now().toSec();
	// --------------------------------------------------------
	// 
	// --------------------------------------------------------
	while (nh_.ok()) {
		//
		update_states_machines();
		//
		// get the first eigen value of the passive ds controller and Check for update of passive ds controller eigen value
		std::vector<float> param_values;
		ros::param::getCached("/iiwa1/CustomControllers/controllers/PassiveDS/params", param_values);	  _d1[LEFT]    = param_values[0];
		if(_d1[LEFT] <FLT_EPSILON) _d1[LEFT]   = 150.0f;
		ros::param::getCached("/iiwa_blue/CustomControllers/controllers/PassiveDS/params", param_values); _d1[RIGHT] = param_values[0];
		if(_d1[RIGHT]<FLT_EPSILON)  _d1[RIGHT] = 150.0f;

		_mutex.lock();
			// update the poses of the robots and the object
			updatePoses();
			// compute generated desired motion and forces
			computeCommands();
			// publish the commands to be exectued
			publish_commands();
			// publish data through topics for analysis 
			publishData();
			//
			if(_startlogging){
				saveData();
			}
			//
		_mutex.unlock();

		ros::spinOnce();
		loop_rate_.sleep();
		_cycle_count ++;    // counter the cycles
	}

	// Send zero command
	for(int k = 0; k < NB_ROBOTS; k++){
		_vd[k].setZero();
		_omegad[k].setZero();
		_qd[k] = _q[k];
	}
	publish_commands();
	//
	ros::spinOnce();
	loop_rate_.sleep();
	// close the data logging files
	datalog.Close_files();

	ros::shutdown();
}

void dual_arm_control::update_states_machines(){
	// ----------------------------------------------------
	keyboard::Keyboard::nonblock_2(1);
    if (keyboard::Keyboard::khbit_2() !=0) {
    	char c = fgetc(stdin);
        fflush(stdin);

			switch(c){
				case 'q':{   
					_goHome = !_goHome;
					if(_goHome){
						_goToAttractors = true;
						// _startlogging = false;
						// datalog.datalog_reset(ros::package::getPath(std::string("dual_arm_control")) +"/Data");
					}
					if(!_goHome){_startlogging  = true;}
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
				case 'r': _releaseAndretract = !_releaseAndretract; 
					break; 
				case 't': _isThrowing = !_isThrowing; 
									_dualTaskSelector = TOSSING; // toss
					break;  
				// case 'g': _goToAttractors = !_goToAttractors; break;  
				case 'g': {
									_goToAttractors = !_goToAttractors;
									if(_goToAttractors){
										_goHome			   = false;
										_releaseAndretract = false; 
									}
				}
					break;
				case 'p': _isPlacing = !_isPlacing; 
									_dualTaskSelector = PICK_AND_PLACE; 			// placing
					break;    
				// control of impact and tossing velocity
				case 'v': 
						_desVtoss -=0.05f; 
						if(_desVtoss < 0.2f) _desVtoss = 0.2f;
						dsThrowing.set_toss_linear_velocity(_desVtoss * _tossVar.release_linear_velocity.normalized());
					break;   
				case 'b':  
						_desVtoss +=0.05f; 
						if(_desVtoss > 2.0f) _desVtoss = 2.0f;
						dsThrowing.set_toss_linear_velocity(_desVtoss * _tossVar.release_linear_velocity.normalized());
					break;   
				case 'y': 
						_desVimp  -=0.05f;
					 	if(_desVimp < 0.05f) _desVimp = 0.05f;
					break;   
				case 'u': 
						_desVimp  +=0.05f;  
						if(_desVimp > 0.6f) _desVimp = 0.6f;
					break; 

				// reset the data logging 
				case 'c':
						_startlogging = false;
						datalog.datalog_reset(ros::package::getPath(std::string("dual_arm_control")) +"/Data");
					break;   
			}
		}
		keyboard::Keyboard::nonblock_2(0);
		// ----------------------------------------------------
}

void dual_arm_control::updatePoses()
{
	if(_initPoseCount < 100){
		_w_H_eeStandby[LEFT]  = _w_H_rb[LEFT]  * Utils<float>::pose2HomoMx(_xrbStandby[LEFT],  _qrbStandby[LEFT]);			// WITH EE pose wrt. the world TO BE CHANGED
		_w_H_eeStandby[RIGHT] = _w_H_rb[RIGHT] * Utils<float>::pose2HomoMx(_xrbStandby[RIGHT],  _qrbStandby[RIGHT]);		// WITH EE pose wrt. the world
		FreeMotionCtrl._w_H_eeStandby[LEFT]  = _w_H_eeStandby[LEFT];
		FreeMotionCtrl._w_H_eeStandby[RIGHT] = _w_H_eeStandby[RIGHT];
		//
		_xDo    = Eigen::Vector3f(_xo(0), _xo(1), _xDo_lifting(2));   // set attractor of lifting task   
		_qDo    = _qo; // _qDo_lifting
		_w_H_Do = Utils<float>::pose2HomoMx(_xDo, _qDo);

		_initPoseCount ++;
	}
	// 
	if( _sensedContact && CooperativeCtrl._ContactConfidence == 1.0 && _xo(2) >= 0.98f*_xDo(2)) {
		// _isThrowing = true;
	}
	/////////////////////////////////////////////////////////////////////////////////////////////
	if(_isPlacing){
		// _xDo    = _xDo_placing; //Eigen::Vector3f(0.65f, 0.155f, 0.35f);   // set attractor of placing task
		_w_H_Do = Utils<float>::pose2HomoMx(_xDo_placing, _qDo);
		if((_w_H_o.block<3,1>(0,3)-_xDo_placing).norm()<=0.05){
			_releaseAndretract = true;
		}
	}
	// // 
	// if(_isThrowing){
	// 	_xDo    = _tossVar.release_position ; //Eigen::Vector3f(0.65f, 0.155f, 0.35f);   // set attractor of placing task
	// 	_w_H_Do = Utils<float>::pose2HomoMx(_xDo, _qDo);
	// }

	/////////////////////////////////////////////////////////////////////////////////////////////
	// Update trajectory of the object
	// Update the object position or its desired position (attractor) through keyboard 
	if(_objCtrlKey && false){
		this->Keyboard_virtual_object_control(); 
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

		if(CooperativeCtrl._ContactConfidence == 1.0)
		{
			_o_H_ee[k]  = _w_H_o.inverse() * _w_H_ee[k];
			_o_H_ee[k](1,3) *= 0.95f; 
			_w_H_Dgp[k]  = _w_H_Do * _o_H_ee[k];
			// _w_H_Dgp[k]  = _w_H_o * _o_H_ee[k];
			if(!_isThrowing){
				_w_H_Dgp[k].block(0,0,3,3)  = _w_H_Do.block(0,0,3,3) * Utils<float>::pose2HomoMx(_xgp_o[k],  _qgp_o[k]).block(0,0,3,3);
				// _w_H_Dgp[k].block(0,0,3,3)  = _w_H_o.block(0,0,3,3) * Utils<float>::pose2HomoMx(_xgp_o[k],  _qgp_o[k]).block(0,0,3,3);
			}
			//
			// if(!_isPickupSet){
			// 	dsThrowing.set_pickup_object_pose(_xo, _qo);
			// 	_isPickupSet = true;
			// }
		}
	}
	// 
	if(!_isPickupSet){
		if((CooperativeCtrl._ContactConfidence == 1.0)){
			dsThrowing.set_pickup_object_pose(_xo, _qo);
			Eigen::Vector3f new_toss_velocity = _desVtoss * (_tossVar.release_position -_xo).normalized();
			dsThrowing.set_toss_linear_velocity(new_toss_velocity);
			_isPickupSet = true;
		}
		else{
			dsThrowing.set_pickup_object_pose(_xo, _qo);
		}
	}


	// Compute errors to object center position and dimension vector
	Eigen::Matrix4f le_H_re     =  _w_H_ee[LEFT].inverse() * _w_H_ee[RIGHT];
	Eigen::Matrix4f lgp_H_rgp   =  _w_H_gp[LEFT].inverse() * _w_H_gp[RIGHT];
	Eigen::Vector3f t_o_absEE   =  0.5f*( _w_H_gp[LEFT].block(0,3,3,1) +  _w_H_gp[RIGHT].block(0,3,3,1)) - 0.5f*( _w_H_ee[LEFT].block(0,3,3,1) +  _w_H_ee[RIGHT].block(0,3,3,1));
	_eoD = fabs(le_H_re(2,3)) - fabs(lgp_H_rgp(2,3)); //(_xD-_xoD).dot(_xoD.normalized());
	_eoC = t_o_absEE.norm(); //(_xoC-_xC).norm();
  	//
	// printing for analysis
	std::cout << "[dual_arm_control]: _w_H_ee[LEFT]: \n" <<  _w_H_ee[0] << std::endl;
	std::cout << "[dual_arm_control]: _w_H_gp[LEFT]: \n" << _w_H_gp[0] << std::endl;
	std::cout << "[dual_arm_control]: _w_H_ee[RIGHT]: \n" << _w_H_ee[1] << std::endl;
	std::cout << "[dual_arm_control]: _w_H_gp[RIGHT]: \n" << _w_H_gp[1] << std::endl;

	std::cout << "[dual_arm_control]: _w_H_o:  \n" <<  _w_H_o << std::endl;
	std::cout << "[dual_arm_control]: _w_H_Do: \n" <<  _w_H_Do << std::endl;
	std::cout << " ------------------------------------------------------------------- " << std::endl;
	std::cout << "[dual_arm_control]: FFFFFFF  filteredWrench[LEFT]: \t"  << _filteredWrench[0].transpose() << std::endl;
	std::cout << "[dual_arm_control]: FFFFFFF  filteredWrench[RIGHT]: \t" << _filteredWrench[1].transpose() << std::endl;
	std::cout << "[dual_arm_control]:  ddddddddddddddddd  _eoD: \t" << _eoD << std::endl;
	std::cout << "[dual_arm_control]:  ddddddddddddddddd  _eoC: \t" << _eoC << std::endl;
}

//
void dual_arm_control::computeCommands()
{
	// Update contact state
  updateContactState();
	//
	_desired_object_wrench.head(3) = -40.0f * (_w_H_o.block(0,3,3,1) - _w_H_Do.block(0,3,3,1)) - _objectMass * _gravity;

	if(_goHome)
	{
		FreeMotionCtrl.computeAsyncMotion(_w_H_ee, _w_H_eeStandby, _w_H_o, _Vd_ee, _qd, true);		
		_Vd_o  = dsThrowing.apply(_xo, _qo, _vo, Eigen::Vector3f(0.0f, 0.0f, 0.0f), 1);  							// 	Function to call in a loop
		Eigen::Vector3f obj_dir = this->get_object_desired_direction(_dualTaskSelector, _w_H_o.block(0,3,3,1));

		std::cout << "[dual_arm_control]: OBJECT MOTION DIRECTION : \t"  << obj_dir.transpose() << std::endl;

		for(int i=0; i<NB_ROBOTS; i++){
			_dirImp[i]  		 	= this->get_impact_direction(_Vd_o.head(3), _n[i], _friction_angle);			//  impact direction
			_VdImpact[LEFT]  	= _desVimp * _dirImp[LEFT]; 																							//	impact velocity [LEFT];
			_VdImpact[RIGHT]	= _desVimp * _dirImp[RIGHT]; 																							//	impact velocity [RIGHT];
			_BasisQ[i] 				= Utils<float>::create3dOrthonormalMatrixFromVector(_dirImp[i]);					//  Orthogonal Basis of Modulated Dual-arm DS
		}
		this->reset_variables();
	}
	else 
	{
		if((ros::Time::now().toSec()-_t0_run) < 0.4f){
			FreeMotionCtrl.computeAsyncMotion(_w_H_ee, _w_H_gp, _w_H_o, _Vd_ee, _qd, true);
		}
		else
		{
			if(_releaseAndretract) //  release_and_retract || release
			{
				FreeMotionCtrl.computeReleaseAndRetractMotion(_w_H_ee, _w_H_Dgp,  _w_H_o, _Vd_ee, _qd, true);
				_isThrowing 	= false;
				_isPlacing 		= false;
				_isPickupSet 	= false;
				_nu_Wr0 = _nu_Wr1 	= 0.0f;
				dsThrowing.reset_release_flag();
			}
			else if(true && _sensedContact && CooperativeCtrl._ContactConfidence == 1.0f)  // TODO : Replace by contact
			{
				_Vd_o  = dsThrowing.apply(_xo, _qo, _vo, Eigen::Vector3f(0.0f, 0.0f, 0.0f), 1);  // Function to call in a loop
				_releaseAndretract = dsThrowing.get_release_flag();

				
				if(_old_dual_method){
					FreeMotionCtrl.computeConstrainedMotion(_w_H_ee, _w_H_Dgp, _w_H_o, _Vd_ee, _qd, false);

					//
					std::cout << "[dual_arm_control]: IN CONTACT OLD METHOD  _w_H_Dgp LEFT : \n"  << _w_H_Dgp[0] << std::endl;
					std::cout << "[dual_arm_control]: IN CONTACT OLD METHOD  _w_H_Dgp RIGHT : \n"  << _w_H_Dgp[1] << std::endl;
					std::cout << "[dual_arm_control]: IN CONTACT OLD METHOD  _isThrowing : \n"  << _isThrowing << std::endl;
					//
					if(_isPlacing){
							Eigen::Matrix4f w_H_DesObj = Utils<float>::pose2HomoMx(_xDo_placing, _qDo_placing);
							_w_H_Dgp[LEFT].block(0,0,3,3)  = w_H_DesObj.block(0,0,3,3) * Utils<float>::pose2HomoMx(_xgp_o[LEFT],  _qgp_o[LEFT]).block(0,0,3,3);
							_w_H_Dgp[RIGHT].block(0,0,3,3)  = w_H_DesObj.block(0,0,3,3) * Utils<float>::pose2HomoMx(_xgp_o[RIGHT],  _qgp_o[RIGHT]).block(0,0,3,3);
							FreeMotionCtrl.generatePlacingMotion(_w_H_ee, _w_H_Dgp,  _w_H_o, w_H_DesObj, _height_via_point, _Vd_ee, _qd, false);
							if((_w_H_o.block<3,1>(0,3)-_xDo_placing).norm()<=0.05){
								_releaseAndretract = true;
							}
					}
					if(_isThrowing){
						Eigen::Matrix4f w_H_DesObj = Utils<float>::pose2HomoMx(_tossVar.release_position, _tossVar.release_orientation);
						_w_H_Dgp[LEFT]  = w_H_DesObj * _o_H_ee[LEFT];
						_w_H_Dgp[RIGHT] = w_H_DesObj * _o_H_ee[RIGHT];
						FreeMotionCtrl.computeConstrainedMotion(_w_H_ee, _w_H_Dgp, _w_H_o, _Vd_ee, _qd, false);
						//
						Eigen::Vector3f t_Do_absEE = (_tossVar.release_position + _tossVar.release_linear_velocity.normalized() * 0.05) - 0.5f * (_w_H_ee[LEFT].block(0, 3, 3, 1) + _w_H_ee[RIGHT].block(0, 3, 3, 1));
						float scale   = 80.f;
						float in_d    = 0.5f*(_w_H_ee[LEFT](0,3)+_w_H_ee[RIGHT](0,3));
						float sigmoid = 1.0f/(1+exp(-scale*(in_d- 0.41f)));
					  _Vd_ee[LEFT].head(3)  = _Vd_ee[LEFT].head(3)/( _Vd_ee[LEFT].head(3).norm() +1e-10) * sigmoid* _desVtoss;
	  				_Vd_ee[RIGHT].head(3) = _Vd_ee[RIGHT].head(3)/(_Vd_ee[RIGHT].head(3).norm()+1e-10) * sigmoid* _desVtoss;
	  				//
	  				if((_w_H_o.block<3,1>(0,3)-_tossVar.release_position).norm()<=0.035){
								_releaseAndretract = true;
						}
					}
				}
				else{
					_w_H_Dgp[LEFT].block(0,0,3,3)  = _w_H_o.block(0,0,3,3) * Utils<float>::pose2HomoMx(_xgp_o[LEFT],  _qgp_o[LEFT]).block(0,0,3,3);
					_w_H_Dgp[RIGHT].block(0,0,3,3) = _w_H_o.block(0,0,3,3) * Utils<float>::pose2HomoMx(_xgp_o[RIGHT],  _qgp_o[RIGHT]).block(0,0,3,3);
					if(dsThrowing.a_normal_> 0.90f){
						_w_H_Dgp[LEFT]  = _w_H_o * _o_H_ee[LEFT];
						_w_H_Dgp[RIGHT] = _w_H_o * _o_H_ee[RIGHT];
					}
					FreeMotionCtrl.dual_arm_motion(_w_H_ee,  _Vee, _w_H_Dgp,  _w_H_o, _w_H_Do, _Vd_o, _BasisQ, _VdImpact, false, _dualTaskSelector, _Vd_ee, _qd, _release_flag); // 0=reach, 1=pick, 2=toss, 3=pick_and_toss, 4=pick_and_place
				}
			}
			else  // Free-motion: reaching
			{
				if(_old_dual_method){
					FreeMotionCtrl.computeCoordinatedMotion2(_w_H_ee, _w_H_gp, _w_H_o, _Vd_ee, _qd, false);
					//
					Eigen::Vector3f error_p_abs     = _w_H_o.block(0,3,3,1) - 0.5f*( _w_H_ee[LEFT].block(0,3,3,1) +  _w_H_ee[RIGHT].block(0,3,3,1));
					Eigen::Vector3f o_error_pos_abs = _w_H_o.block<3,3>(0,0).transpose() * error_p_abs;
					Eigen::Vector3f o_error_pos_abs_paral = Eigen::Vector3f(o_error_pos_abs(0), 0.0f, o_error_pos_abs(2));
					float cp_ap = Utils<float>::computeCouplingFactor(o_error_pos_abs_paral, 50.0f, 0.17f, 1.0f, true);  // 50.0f, 0.05f, 2.8f
					// create impact in the normal direction
					_Vd_ee[LEFT].head(3)  = _Vd_ee[LEFT].head(3)  + _n[LEFT]  * cp_ap * 0.20f; //_desVimp;
					_Vd_ee[RIGHT].head(3) = _Vd_ee[RIGHT].head(3) + _n[RIGHT] * cp_ap * 0.20f; //_desVimp;
				}
				else{
					FreeMotionCtrl.dual_arm_motion(_w_H_ee,  _Vee, _w_H_gp,  _w_H_o, _w_H_Do, _Vd_o, _BasisQ, _VdImpact, false, 0, _Vd_ee, _qd, _release_flag);    // 0: reach
				}
				// if(fabs(_w_H_o(0,3) -_xDo_lifting(0)) <=0.05){
				// 				_releaseAndretract = true;
				// }
  			dsThrowing._refVtoss = _desVimp;
	 			_Vd_o.setZero();	// for data logging
			}
		}
		// compute the object's grasp points velocity
		getGraspPointsVelocity();
	  //
	  CooperativeCtrl.getAppliedWrenches(_goHome, _contactState, _w_H_o, _w_H_ee, _w_H_gp, _desired_object_wrench, _qp_wrench_generation);
	  // applied force in velocity space
	  for(int i=0; i<NB_ROBOTS; i++){
	  	_fxc[i] = 1.0f/_d1[i] * CooperativeCtrl._f_applied[i].head(3);
	  }
	} 
  // Extract linear velocity commands and desired axis angle command
	prepareCommands(_Vd_ee, _qd, _V_gpo);

	std::cout << "[dual_arm_control]: _w_H_o: \n" << _w_H_o << std::endl; 
	std::cout << "[dual_arm_control]: _w_H_Do: \n" <<  _w_H_Do << std::endl;
	std::cout << "[dual_arm_control]:  ------------- _sensedContact: \t" << _sensedContact << std::endl;
	std::cout << "[dual_arm_control]: _Vd_ee[LEFT]:  \t" << _Vd_ee[LEFT].transpose() << std::endl;
	std::cout << "[dual_arm_control]: _Vd_ee[RIGHT]: \t" << _Vd_ee[RIGHT].transpose() << std::endl;
	std::cout << " vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv " <<  std::endl;
	std::cout << "[dual_arm_control]: _vd[LEFT]:  \t" << _vd[LEFT].transpose() << std::endl;
	std::cout << "[dual_arm_control]: _vd[RIGHT]: \t" << _vd[RIGHT].transpose() << std::endl;
	std::cout << " ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ " <<  std::endl;
	std::cout << " COMPUTED HAND WRENCH _fxc  LEFT \t " << _fxc[LEFT].transpose() << std::endl;
	std::cout << " COMPUTED HAND WRENCH _fxc RIGHT \t " << _fxc[RIGHT].transpose() << std::endl; 

	std::cout << " [dual_arm_control]: _dirImp[LEFT] \t " << _dirImp[LEFT].transpose()   << " normal LEFT  \t " << _n[LEFT].transpose()<< std::endl;
	std::cout << " [dual_arm_control]: _dirImp[RIGHT] \t " << _dirImp[RIGHT].transpose() << " normal RIGHT \t " << _n[RIGHT].transpose()<< std::endl;

	std::cout << " EEEE----------- EEEPPP   _desVtoss IIIIIIII ----------- ONNNNNNNN \t " << _desVtoss << std::endl; 
	std::cout << " EEEE----------- EEEPPP   _desVimp  IIIIIIII ----------- ONNNNNNNN \t " <<  _desVimp <<  std::endl;
}

//
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
	_x[k] << 	msg->position.x,	msg->position.y, 	msg->position.z;														// position
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
    _wrench[k].head(3) = _wRb[k] * _wrench[k].head(3);
		_wrench[k].tail(3) = _wRb[k] * _wrench[k].tail(3);
    _filteredWrench[k] = _filteredForceGain*_filteredWrench[k]+(1.0f-_filteredForceGain)*_wrench[k];
    //
    _normalForce[k] = fabs((_filteredWrench[k].segment(0,3)).dot(_n[k]));
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
	_wrench[LEFT].head(3) = _wRb[LEFT] * _wrench[LEFT].head(3);
	_wrench[LEFT].tail(3) = _wRb[LEFT] * _wrench[LEFT].tail(3);
	_filteredWrench[LEFT] = _filteredForceGain*_filteredWrench[LEFT]+(1.0f-_filteredForceGain)*_wrench[LEFT];
	// _filteredWrench[LEFT] = raw;
	 _normalForce[LEFT] = fabs((_filteredWrench[LEFT].segment(0,3)).dot(_n[LEFT]));
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
	_wrench[RIGHT].head(3) = _wRb[RIGHT] * _wrench[RIGHT].head(3);
	_wrench[RIGHT].tail(3) = _wRb[RIGHT] * _wrench[RIGHT].tail(3);
	_filteredWrench[RIGHT] = _filteredForceGain*_filteredWrench[RIGHT]+(1.0f-_filteredForceGain)*_wrench[RIGHT];
	// _filteredWrench[RIGHT] = raw;
	 _normalForce[RIGHT] = fabs((_filteredWrench[RIGHT].segment(0,3)).dot(_n[RIGHT]));
}

void dual_arm_control::updateRobotStatesLeft(const sensor_msgs::JointState::ConstPtr &msg) {
	for(int i=0; i<_nb_joints[LEFT]; i++){
		_joints_positions[LEFT](i)	= (float) msg->position[i];
		_joints_velocities[LEFT](i)	= (float) msg->velocity[i];
		_joints_torques[LEFT](i)		= (float) msg->effort[i];
	}
	//
	SGF::Vec temp_acc(_nb_joints[LEFT]);
	_sgf_ddq_filtered_l->AddData(_joints_velocities[LEFT]);
  _sgf_ddq_filtered_l->GetOutput(1,temp_acc);
 	_joints_accelerations[LEFT]	= temp_acc.cast<float>();	

}

void dual_arm_control::updateRobotStatesRight(const sensor_msgs::JointState::ConstPtr &msg) {
	//
	for(int i=0; i<_nb_joints[RIGHT]; i++){
		_joints_positions[RIGHT](i)		= (float)msg->position[i];
		_joints_velocities[RIGHT](i)	= (float)msg->velocity[i];
		_joints_torques[RIGHT](i)			= (float)msg->effort[i];
	}
	//
	SGF::Vec temp_acc(_nb_joints[RIGHT]);
	_sgf_ddq_filtered_r->AddData(_joints_velocities[RIGHT]);
  _sgf_ddq_filtered_r->GetOutput(1,temp_acc);
 	_joints_accelerations[RIGHT]	= temp_acc.cast<float>();		
}

void dual_arm_control::updateContactState()
{
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
  // if(_normalForceAverage[LEFT] > 2.0f && _normalForceAverage[RIGHT] > 2.0f &&  _eoD < 0.05f && _eoC < 0.1f)
  // if (_normalForceAverage[LEFT] > _forceThreshold || _normalForceAverage[RIGHT] > _forceThreshold && (CooperativeCtrl._ContactConfidence == 1.0f)) 
  if ((_normalForceAverage[LEFT] > 2.0f || _normalForceAverage[RIGHT] > 2.0f) && _eoD < 0.05f && (_eoC < 0.05f || CooperativeCtrl._ContactConfidence == 1.0f))
  {
    _contactState = CONTACT;
    _c = 1.0f;
  }
  // else if(!(_normalForceAverage[LEFT] > 2.5f && _normalForceAverage[RIGHT] > 2.5f) && _eoD < 0.05f && _eoC < 0.1f)
  else if(!(_normalForceAverage[LEFT] > 2.0f && _normalForceAverage[RIGHT] > 2.0f) && _eoD < 0.05f && _eoC < 0.05f){
    _contactState = CLOSE_TO_CONTACT;
    _c = 0.0f;
  }
  else{
    _contactState = NO_CONTACT;
    _c = 0.0f;
  }
  // check contact
  _sensedContact = ((fabs(_normalForce[LEFT]) >= _forceThreshold) || (fabs(_normalForce[RIGHT]) >= _forceThreshold)) && (_c == 1.0f);
  // _sensedContact = (fabs(_normalForce[LEFT]) >= _forceThreshold) && (fabs(_normalForce[RIGHT]) >= _forceThreshold) && (_c == 1.0f);

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

  // --------------------------------------------------------
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

  if(_releaseAndretract){
			_fxc[0].setZero();
			_fxc[1].setZero();
			_nu_Wr0 = 0.0f;
	  	_nu_Wr1 = 0.0f;
		}
		//
		if(_goToAttractors){
			_applyVelo = 1.f;
		}
		else{
			_applyVelo = 0.0f;
			// keep the current orientation if not going to the attractor
			_qd[LEFT]  = _q[LEFT];
			_qd[RIGHT] = _q[RIGHT];
		}

	  // set the command to send
	  for(int i=0; i<NB_ROBOTS; i++){
	  	_vd[i] = _applyVelo *_vd[i] + _nu_Wr0 * _fxc[i] ;
	  }
}


void dual_arm_control::getGraspPointsVelocity()    // in object struct or object
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
void dual_arm_control::Keyboard_virtual_object_control()		// control of object position through keyboad
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
//
Eigen::Vector3f dual_arm_control::get_impact_direction(Eigen::Vector3f des_object_force, Eigen::Vector3f normal, float coeff_friction)
{
	//
	Eigen::Matrix3f R1;
	Eigen::Matrix3f R0;
	Utils<float>::Orthobasis(des_object_force, normal, R1, R0);
	// 
	float theta = 0.0f;
	if(this->_impact_dir_preset){
		theta 		= M_PI/180.f * coeff_friction;
	}
	else{
		float vzo = des_object_force.transpose() * R0.col(2);
		float vxo = des_object_force.transpose() * normal;
		theta 	  = std::atan2(vzo,vxo);
	}
	if(fabs(theta) >= M_PI/180.f * this->_max_friction_angle){
		theta = theta/fabs(theta) * M_PI/180.f *_max_friction_angle;
	}
	Eigen::Vector3f imp_dir = R0 * Eigen::Vector3f(std::cos(theta), 0.0f, std::sin(theta));

	return imp_dir.normalized();
}

void dual_arm_control::reset_variables(){

	for(int i=0; i<NB_ROBOTS; i++){
		_V_gpo[i].setZero();
		_fxc[i].setZero();
	}
	_releaseAndretract 	= false;
	_isThrowing 				= false;
	_isPlacing 					= false;
	_isPickupSet 				= false;
	_Vd_o.setZero();
	_nu_Wr0 = _nu_Wr1 	= 0.0f;
	this->_refVreach  	= 0.0f;
	FreeMotionCtrl._refVreach[LEFT]  = 0.0f;
	FreeMotionCtrl._refVreach[RIGHT] = 0.0f;
	dsThrowing._refVtoss = _desVimp;
	dsThrowing.reset_release_flag();
	//
	// _xDo    = Eigen::Vector3f(0.35f, 0.00f, 0.50f);   // set attractor of placing task
	_w_H_Do = Utils<float>::pose2HomoMx(_xDo_lifting, _qDo);
}

Eigen::Vector3f dual_arm_control::get_object_desired_direction(int task_type, Eigen::Vector3f object_pos){
	// 0=reach, 1=pick, 2=toss, 3=pick_and_toss, 4=pick_and_place
	Eigen::Vector3f des_object_pos = _xDo;
	switch(task_type){
		case 1:  des_object_pos = _xDo_lifting; 		break;
		case 2:  des_object_pos = dsThrowing.Xt_; 	break;
		case 3:  des_object_pos = dsThrowing.Xt_; 	break;
		case 4:  des_object_pos = 0.5f*(object_pos + _xDo_placing);
						 des_object_pos(2)=_xDo_placing(2) + _height_via_point; 		break;
		default: des_object_pos = _xDo; 						break;
	}
	//
	Eigen::Vector3f error_obj_pos = des_object_pos - object_pos;
	return error_obj_pos.normalized();
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
    // normal forces
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
		// norm of desired velocity
		std_msgs::Float64 msgVel;
    msgVel.data = _Vee[k].head(3).norm();
		_pubNormLinVel[k].publish(msgVel);
  }
}

void dual_arm_control::saveData()
{
	Eigen::Vector3f  xgrL = _w_H_gp[LEFT].block(0,3,3,1);
	Eigen::Vector3f  xgrR = _w_H_gp[RIGHT].block(0,3,3,1);
	Eigen::Vector4f  qgrL = Utils<float>::rotationMatrixToQuaternion(_w_H_gp[LEFT].block(0,0,3,3)); //
	Eigen::Vector4f  qgrR = Utils<float>::rotationMatrixToQuaternion(_w_H_gp[RIGHT].block(0,0,3,3)); //
	//
	Eigen::MatrixXf power_left   = _joints_torques[LEFT].transpose()  * _joints_velocities[LEFT];
	Eigen::MatrixXf power_right  = _joints_torques[RIGHT].transpose() * _joints_velocities[RIGHT];

	// To do: add desired object orientation
	// To do : grasping point after the ee pose
	// To o : add desired quaternion
	// if(_startlogging)
	// {
		datalog._OutRecord_pose			<< (float)(_cycle_count * _dt) << ", ";																																																						// cycle time
		datalog._OutRecord_pose			<< _x[LEFT].transpose().format(CSVFormat)  	<< " , " << _q[LEFT].transpose().format(CSVFormat) 	<< " , ";																					// left end-effector
		datalog._OutRecord_pose   	<< _x[RIGHT].transpose().format(CSVFormat) 	<< " , " << _q[RIGHT].transpose().format(CSVFormat) << " , ";																					// right end-effector
		datalog._OutRecord_pose   	<< _xo.transpose().format(CSVFormat) 			 	<< " , " << _qo.transpose().format(CSVFormat) 			<< " , ";																					// object
		datalog._OutRecord_pose   	<< _w_H_Do(0,3) << " , " << _w_H_Do(1,3) 		<< " , " << _w_H_Do(2,3) << " , ";																																		// desired object
		datalog._OutRecord_pose   	<< xgrL.transpose().format(CSVFormat) 			<< " , " << qgrL.transpose().format(CSVFormat) 			<< " , ";																					// left  grasping point
		datalog._OutRecord_pose   	<< xgrR.transpose().format(CSVFormat) 			<< " , " << qgrR.transpose().format(CSVFormat) 			<< " , ";																					// right grasping point
		datalog._OutRecord_pose   	<< _tossVar.release_position.transpose().format(CSVFormat) 	<< " , " << _tossVar.release_orientation.transpose().format(CSVFormat)   << " , ";			// release pose
		datalog._OutRecord_pose   	<< _tossVar.rest_position.transpose().format(CSVFormat) 			<< " , " << _tossVar.rest_orientation.transpose().format(CSVFormat) 			<< std::endl;	// rest pose 
		
		datalog._OutRecord_velo			<< (float)(_cycle_count * _dt) << ", ";
		datalog._OutRecord_velo			<< _Vd_ee[LEFT].transpose().format(CSVFormat) << " , " << _Vd_ee[RIGHT].transpose().format(CSVFormat) << " , ";
		datalog._OutRecord_velo			<< _Vee[LEFT].transpose().format(CSVFormat) 	<< " , " << _Vee[RIGHT].transpose().format(CSVFormat) 	<< " , ";
		datalog._OutRecord_velo			<< _vd[LEFT].transpose().format(CSVFormat) 		<< " , " << _vd[RIGHT].transpose().format(CSVFormat) 		<< " , ";
		datalog._OutRecord_velo			<< _omegad[LEFT].transpose().format(CSVFormat)<< " , " << _omegad[RIGHT].transpose().format(CSVFormat)<< " , ";
		datalog._OutRecord_velo			<< _vo.transpose().format(CSVFormat) 					<< " , " << _wo.transpose().format(CSVFormat) 					<< " , ";
		datalog._OutRecord_velo			<< _Vd_o.transpose().format(CSVFormat) << " , ";
		datalog._OutRecord_velo			<< _tossVar.release_linear_velocity.transpose().format(CSVFormat) << " , " <<  _tossVar.release_angular_velocity .transpose().format(CSVFormat) << " , "<< std::endl;
	
			
		datalog._OutRecord_efforts	<< (float)(_cycle_count * _dt) << ", ";
		datalog._OutRecord_efforts	<< _filteredWrench[LEFT].transpose().format(CSVFormat)  << " , ";
		datalog._OutRecord_efforts	<< _filteredWrench[RIGHT].transpose().format(CSVFormat) << " , ";
		datalog._OutRecord_efforts  << CooperativeCtrl._f_applied[LEFT].transpose().format(CSVFormat) << " , ";
		datalog._OutRecord_efforts  << CooperativeCtrl._f_applied[RIGHT].transpose().format(CSVFormat) << std::endl;
	
		datalog._OutRecord_tasks		<< (float)(_cycle_count * _dt) << ", ";
		datalog._OutRecord_tasks   	<< _desVimp << " , " << _desVtoss << " , "; 
		datalog._OutRecord_tasks   	<< _goHome 	<< " , " << _goToAttractors << " , " << _releaseAndretract << " , " << _isThrowing << " , " << _isPlacing << " , " << _c << " , "; 			//CooperativeCtrl._ContactConfidence << " , ";
		datalog._OutRecord_tasks   	<< FreeMotionCtrl.a_proximity_ << " , " << FreeMotionCtrl.a_normal_ << " , " << FreeMotionCtrl.a_tangent_ << " , " << FreeMotionCtrl.a_release_ << " , " << FreeMotionCtrl.a_retract_ << " , ";
		datalog._OutRecord_tasks   	<< dsThrowing.a_proximity_ << " , " << dsThrowing.a_normal_  << " , " << dsThrowing.a_tangent_<< " , " << dsThrowing.a_toss_  << std::endl;
		// 
		datalog._OutRecord_jts_states << (float)(_cycle_count * _dt) << ", ";
		datalog._OutRecord_jts_states << _joints_positions[LEFT].transpose().format(CSVFormat)  		<< " , " << _joints_positions[RIGHT].transpose().format(CSVFormat)  		<< " , ";
		datalog._OutRecord_jts_states << _joints_velocities[LEFT].transpose().format(CSVFormat)  		<< " , " << _joints_velocities[RIGHT].transpose().format(CSVFormat)  		<< " , ";
		datalog._OutRecord_jts_states << _joints_accelerations[LEFT].transpose().format(CSVFormat)  << " , " << _joints_accelerations[RIGHT].transpose().format(CSVFormat)  << " , " ;
		datalog._OutRecord_jts_states << _joints_torques[LEFT].transpose().format(CSVFormat)   			<< " , " <<  _joints_torques[RIGHT].transpose().format(CSVFormat) 			<< " , " ;
		datalog._OutRecord_jts_states << power_left(0,0)   << " , " <<  power_right(0,0) << std::endl;
	// }

}	