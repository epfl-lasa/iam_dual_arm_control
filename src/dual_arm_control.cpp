
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
	// _objectMass = 1.0f;
	// _objectDim << 0.20f, 0.20f, 0.20f;
	// _objectDim << 0.13f, 0.14f, 0.15f;
	// _objectDim << 0.26f, 0.26f, 0.26f;
	// _objectDim << 0.19f, 0.19f, 0.19f;
	// _objectDim << 0.27f, 0.37f, 0.24f;
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
		_dirImp[k].setZero();
		_VdImpact[k].setZero();
		//
		_joints_positions[k].setZero();
		_joints_velocities[k].setZero();
		_joints_accelerations[k].setZero();
		_joints_torques[k].setZero();

		_VEE_oa[k].setZero();
	}
	// 
	// Filtered variable (SG)
	// _xo_filtered 			  = std::make_unique<SGF::SavitzkyGolayFilter>(3,3,6,_dt);
	// _qo_filtered				= std::make_unique<SGF::SavitzkyGolayFilter>(4,3,10,_dt);

	_sgf_ddq_filtered_l = std::make_unique<SGF::SavitzkyGolayFilter>(7,3,6,_dt); // dim, order. window lenght
	_sgf_ddq_filtered_r = std::make_unique<SGF::SavitzkyGolayFilter>(7,3,6,_dt); // dim, order. window lenght

	// _xt_filtered 			  = std::make_unique<SGF::SavitzkyGolayFilter>(3,3,10,_dt);
	//
	// _vo.setZero();
	// _wo.setZero();
	// _Vo.setZero();
	// _xo.setZero(); 
	// _xDo.setZero(); 
	// _qo  << 1.0f, 0.0f, 0.0f, 0.0f;
	// _qDo << 1.0f, 0.0f, 0.0f, 0.0f;
	// _w_H_o  = Utils<float>::pose2HomoMx(_xo, _qo);	
	// _w_H_Do = Utils<float>::pose2HomoMx(_xDo, _qDo);	
	// _Vd_o.setZero();

	_initPoseCount  = 0;
	_objecPoseCount = 0;
	_desired_object_wrench.setZero();

	// object desired grasping points
	Eigen::Matrix3f o_R_gpl;	o_R_gpl.setZero();		
	Eigen::Matrix3f o_R_gpr;	o_R_gpr.setZero();	
	o_R_gpl(0,0) =  1.0f;	o_R_gpl(2,1) = -1.0f; 	o_R_gpl(1,2) = 1.0f;
	o_R_gpr(0,0) =  1.0f;	o_R_gpr(2,1) =  1.0f; 	o_R_gpr(1,2) =-1.0f;
	// o_R_gpr(0,0) = -1.0f;	o_R_gpr(2,1) = -1.0f; 	o_R_gpr(1,2) =-1.0f; 
	// o_R_gpr(2,0) = -1.0f;	o_R_gpr(0,1) =  1.0f; 	o_R_gpr(1,2) =-1.0f; 
	// _qgp_o[0] = Utils<float>::rotationMatrixToQuaternion(o_R_gpl); //
	// _qgp_o[1] = Utils<float>::rotationMatrixToQuaternion(o_R_gpr); //
	//
	int sgf_p[3];
	int sgf_o[3];
	sgf_p[3] = 3; 	sgf_p[3] = 3; 	sgf_p[3] = 6;
	sgf_o[3] = 4; 	sgf_o[3] = 3; 	sgf_o[3] = 10;

	// object_to_grasp
	object_.init_object(sgf_p, sgf_o, _dt, o_R_gpl, o_R_gpr);

	// target
	target_.init_target(3,3,10,_dt);


	// _xt.setZero();
	// _qt.setZero();
	// _vt.setZero();
	// _wt.setZero();
	// _qt  << 1.0f, 0.0f, 0.0f, 0.0f;
	// _x_pickup.setZero();
	// _x_intercept.setZero();
	// _xt_state2go.setZero();

	// // normal to contact surfaces
	// _n[LEFT]  = o_R_gpl.col(2);
	// _n[RIGHT] = o_R_gpr.col(2);

	_c   = 0.0f;
	_v_max = 1.50f;     // velocity limits
	_w_max = 4.0f;     // velocity limits
	// coordination errors
	// _xoC.setZero();
	// _xoD.setZero();
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
	_gain_rel.topLeftCorner(3,3)     = 1.0f*Eigen::Vector3f(2.0f, 2.0f, 2.0f).asDiagonal();  
	_gain_rel.bottomRightCorner(3,3) = 1.0f*Eigen::Vector3f(1.0f, 1.0f, 1.0f).asDiagonal();  

	_reachable 		   = 1.0f;
	_targetForce 	   = 10.0f;
	_filteredForceGain = 0.9f;
	_sensedContact     = false;
	_forceThreshold    = 2.0f;
	_nu_Wr0 		   = 0.0f;
	_nu_Wr1 		   = 0.0f;
	_qp_wrench_generation = false;
	//
	// Stanby pose
	_xrbStandby[LEFT]   = Vector3f(0.35f,  0.20f, 0.80f); // relative to the robot base
	_xrbStandby[RIGHT]  = Vector3f(0.35f, -0.20f, 0.80f); // relative to the robot base
	_qrbStandby[LEFT]  << 0.3827f, -0.9239f, 0.0f, 0.0f;
	_qrbStandby[RIGHT] << 0.3827f,  0.9239f, 0.0f, 0.0f;

	_delta_pos.setZero();
	_delta_ang.setZero();
	_filt_delta_ang.setZero();
	_filt_delta_ang_mir.setZero();
	_delta_rel_pos.setZero();

	_delta_oDx  = 0.0f;
	_delta_oDy  = 0.0f;
	_delta_oDz  = 0.0f;
	_desVtoss   = 0.5f;
	_applyVelo  = 0.0f;
	_desVimp    = 0.5f;
	_desVreach  = 0.75f;
	_refVreach  = 0.0f;
	_delta_Imp  = 0.0f;
	_delta_Toss = 0.0f;

	_objCtrlKey 		= true;
	_goHome 				= true;
	_goToAttractors 		= true;     // true
	_releaseAndretract 	= false;
	_isThrowing 		= false;
	_isPlacing     	= false;
	_isPlaceTossing = false;
	// _t0_run = ros::Time::now().toSec();
	_release_flag 	= false;
	_startlogging 	= false;
	_isPickupSet  	= false;
	_dualTaskSelector = 1;
	_old_dual_method  = true;
	_mode_conveyor_belt = 0;
	_desSpeed_conveyor_belt = 200;
	_nominalSpeed_conveyor_belt = 200;
	_magniture_pert_conveyor_belt = 100;
	_dual_PathLen_AvgSpeed.setZero();
	_hasCaughtOnce 		= false;
	_isIntercepting 	= false;
	_isDisturbTarget 	= false;
	_beta_vel_mod 		= 1.0f;
	_initSpeedScaling = 1.0f; 	//0.75; 	//
	_trackingFactor 	= 0.40f; 	//0.17f; 	// 0.35f better
	_delta_tracking 	= 0.0f;
	_winLengthAvgSpeedEE = 20;
	// _winCounterAvgSpeedEE = 0;
	_isSimulation 		= true;
	_adaptationActive = false;
	_isTargetFixed 		= true;
	userSelect_ 			= true;

	_feasibleAlgo = false;
	_pickupBased  = true;
	_trackTargetRotation = true;
	_isMotionTriggered =false;
	_isRatioFactor = false;
	_tol_attractor = 0.07f;
	_switchSlopeAdapt = 100.0f;
	_beta_vel_mod_unfilt = 1.0f;
	_time2intercept_tgt = 0.0f;
	_time2intercept_bot = 0.0f;

	_updatePathEstim  	= false;
	_counter_monocycle 	= 0;
	_counter_pickup 	 	= 0;
	_dxEE_dual_avg 	 		= 0.f;
	_dxEE_dual_avg_pcycle = 0.f;
	_dxEE_dual_avg_0 		= 0.f;
	_Del_xEE_dual_avg 	= 0.f;
	_xEE_dual.setZero();
	_xEE_dual_0.setZero(); 
	_dual_angular_limit.setZero();
}
//
dual_arm_control::~dual_arm_control(){}

//
bool dual_arm_control::init()
{
	
	// initialize the tossing DS
	//---------------------------
	float param_toolOffset_real_left;
	float param_toolOffset_real_right;
	float param_toolOffset_sim_left;
	float param_toolOffset_sim_right;

	std::string param_object_name;
	float param_objectMass;
	std::vector<float> param_objectDim;
	std::vector<float> param_graspOffset_L;
	std::vector<float> param_graspOffset_R;

	std::vector<float> param_releasePos;
	std::vector<float> param_releaseOrient;
	std::vector<float> param_releaseLinVel_dir;
	std::vector<float> param_releaseAngVel;
	std::vector<float> param_restPos;
	std::vector<float> param_restOrient;
	std::vector<float> param_dual_angular_limit;

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
	bool isQP_wrench_generation  = false;
	std::string param_damping_topic_CustomCtrl_left  = "/iiwa1/CustomControllers/controllers/PassiveDS/params";
	std::string param_damping_topic_CustomCtrl_right = "/iiwa_blue/CustomControllers/controllers/PassiveDS/params";
	std::string param_damping_topic_TorqueCtrl_left  = "/iiwa1/control/lambda_Pos";
	std::string param_damping_topic_TorqueCtrl_right = "/iiwa_blue/control/lambda_Pos";

	bool gotParam = true;

	while(!nh_.getParam("dual_system/simulation", _isSimulation)){ROS_INFO("Waitinng for param: dual_system/simulation ");}
	while(!nh_.getParam("dual_system/passiveDS/dampingTopic/CustomController/left", param_damping_topic_CustomCtrl_left)){ROS_INFO("Waitinng for param : CustomController/left");}
	while(!nh_.getParam("dual_system/passiveDS/dampingTopic/CustomController/right", param_damping_topic_CustomCtrl_right)){ROS_INFO("Waitinng for param : CustomController/right");}
	while(!nh_.getParam("dual_system/passiveDS/dampingTopic/TorqueController/left", param_damping_topic_TorqueCtrl_left)){ROS_INFO("Waitinng for param : TorqueController/left");}
	while(!nh_.getParam("dual_system/passiveDS/dampingTopic/TorqueController/right", param_damping_topic_TorqueCtrl_right)){ROS_INFO("Waitinng for param : TorqueController/right");}
	while(!nh_.getParam("dual_system/tool/offset2end_effector/real/left",  param_toolOffset_real_left)){ROS_INFO("Waitinng for param:  offset2end_effector/real/left");}
	while(!nh_.getParam("dual_system/tool/offset2end_effector/real/right", param_toolOffset_real_right)){ROS_INFO("Waitinng for param: offset2end_effector/real/right ");}
	while(!nh_.getParam("dual_system/tool/offset2end_effector/sim/left",  param_toolOffset_sim_left)){ROS_INFO("Waitinng for param: offset2end_effector/sim/left ");}
	while(!nh_.getParam("dual_system/tool/offset2end_effector/sim/right", param_toolOffset_sim_right)){ROS_INFO("Waitinng for param: offset2end_effector/sim/right ");}

	while(!nh_.getParam("object/name", param_object_name)){ROS_INFO("Waitinng for param: object/name");}
	while(!nh_.getParam("object/" + param_object_name + "/graspOffset_L", param_graspOffset_L)){ROS_INFO("Waitinng for param: object/graspOffset_L ");}
	while(!nh_.getParam("object/" + param_object_name + "/graspOffset_R", param_graspOffset_R)){ROS_INFO("Waitinng for param: object/graspOffset_R ");}
	while(!nh_.getParam("object/" + param_object_name + "/dimension", param_objectDim)){ROS_INFO("Waitinng for param: object dimension ");}
	while(!nh_.getParam("object/" + param_object_name + "/mass", param_objectMass)){ROS_INFO("Waitinng for param: object mass ");}

	while(!nh_.getParam("dual_arm_task/coordination/ds_absolute_gains", param_abs_gains)){ROS_INFO("Waitinng for param: ds_absolute_gains ");}
	while(!nh_.getParam("dual_arm_task/coordination/ds_relative_gains", param_rel_gains)){ROS_INFO("Waitinng for param: ds_relative_gains ");}

	while(!nh_.getParam("dual_arm_task/reach_to_grasp/impact/desVimp", _desVimp)){ROS_INFO("Waitinng for param: impact/desVimp");}
	while(!nh_.getParam("dual_arm_task/reach_to_grasp/desVreach", _desVreach)){ROS_INFO("Waitinng for param: reach_to_grasp/desVreach");}
	while(!nh_.getParam("dual_arm_task/reach_to_grasp/impact/impact_direction/friction_angle", _friction_angle)){ROS_INFO("Waitinng for param: impact_direction/friction_angl");}
	while(!nh_.getParam("dual_arm_task/reach_to_grasp/impact/impact_direction/max_friction_angle", _max_friction_angle)){ROS_INFO("Waitinng for param: impact_direction/max_friction_angle");}
	while(!nh_.getParam("dual_arm_task/reach_to_grasp/impact/impact_direction/impact_dir_preset", _impact_dir_preset)){ROS_INFO("Waitinng for param: impact_direction/impact_dir_preset");}

	while(!nh_.getParam("dual_arm_task/standby_pose/robot_left/position", param_stanbyPosition[LEFT])){ROS_INFO("Waitinng for param: robot_left/position ");}
	while(!nh_.getParam("dual_arm_task/standby_pose/robot_left/orientation", param_stanbyOrientation[LEFT])){ROS_INFO("Waitinng for param: robot_left/orientation ");}
	while(!nh_.getParam("dual_arm_task/standby_pose/robot_right/position", param_stanbyPosition[RIGHT])){ROS_INFO("Waitinng for param: robot_right/position ");}
	while(!nh_.getParam("dual_arm_task/standby_pose/robot_right/orientation", param_stanbyOrientation[RIGHT])){ROS_INFO("Waitinng for param: robot_right/orientation ");}

	while(!nh_.getParam("dual_arm_task/lifting/position", param_xDo_lifting)){ROS_INFO("Waitinng for param: lifting/position");}
	while(!nh_.getParam("dual_arm_task/lifting/orientation", param_qDo_lifting)){ROS_INFO("Waitinng for param: lifting/orientation");}
	while(!nh_.getParam("dual_arm_task/lifting/increment_lift_pos", _increment_lift_pos)){ROS_INFO("Waitinng for param: increment_lift_pos");}

	while(!nh_.getParam("dual_arm_task/placing/position", param_xDo_placing)){ROS_INFO("Waitinng for param: placing/position");}
	while(!nh_.getParam("dual_arm_task/placing/orientation", param_qDo_placing)){ROS_INFO("Waitinng for param: placing/orientation");}
	while(!nh_.getParam("dual_arm_task/placing/height_via_point", _height_via_point)){ROS_INFO("Waitinng for param: placing/height_via_point");}

	while(!nh_.getParam("dual_arm_task/tossing/desVtoss", _desVtoss)){ROS_INFO("Waitinng for param: tossing/desVtoss ");}
	while(!nh_.getParam("dual_arm_task/tossing/releasePos", param_releasePos)){ROS_INFO("Waitinng for param: tossing/releasePos ");}
	while(!nh_.getParam("dual_arm_task/tossing/releaseOrient", param_releaseOrient)){ROS_INFO("Waitinng for param: tossing/releaseOrien ");}
	while(!nh_.getParam("dual_arm_task/tossing/releaseLinVel_dir", param_releaseLinVel_dir)){ROS_INFO("Waitinng for param: tossing/releaseLinVel_dir ");}
	while(!nh_.getParam("dual_arm_task/tossing/releaseAngVel", param_releaseAngVel)){ROS_INFO("Waitinng for param: tossing/releaseAngVel ");}
	while(!nh_.getParam("dual_arm_task/tossing/restPos", param_restPos)){ROS_INFO("Waitinng for param: tossing/restPos ");}
	while(!nh_.getParam("dual_arm_task/tossing/restOrient", param_restOrient)){ROS_INFO("Waitinng for param: tossing/restOrient ");}
	while(!nh_.getParam("dual_arm_task/tossing/increment_release_pos", _increment_release_pos)){ROS_INFO("Waitinng for param: tossing/increment_release_pos ");}  
	while(!nh_.getParam("dual_arm_task/tossing/dual_angular_limit", param_dual_angular_limit)){ROS_INFO("Waitinng for param: tossing/param_dual_angular_limit ");}

	while(!nh_.getParam("dual_arm_task/dualTaskSelector", _dualTaskSelector)){ROS_INFO("Waitinng for param: dualTaskSelector");}
	while(!nh_.getParam("dual_arm_task/old_dual_method", _old_dual_method)){ROS_INFO("Waitinng for param:  old_dual_method");}
	while(!nh_.getParam("dual_arm_task/modulated_reaching", modulated_reaching)){ROS_INFO("Waitinng for param:  modulated_reaching");}
	while(!nh_.getParam("dual_arm_task/isNorm_impact_vel", isNorm_impact_vel)){ROS_INFO("Waitinng for param:  isNorm_impact_vel");}
	while(!nh_.getParam("dual_arm_task/isQP_wrench_generation", isQP_wrench_generation)){ROS_INFO("Waitinng for param:  isQP_wrench_generation");}
	while(!nh_.getParam("dual_arm_task/objCtrlKey", _objCtrlKey)){ROS_INFO("Waitinng for param:  objCtrlKey");}
	while(!nh_.getParam("dual_arm_task/isTargetFixed", _isTargetFixed)){ROS_INFO("Waitinng for param:  isTargetFixed");} 
	while(!nh_.getParam("dual_arm_task/userSelect", userSelect_)){ROS_INFO("Waitinng for param:  userSelect");}

	while(!nh_.getParam("conveyor_belt/control_mode", _ctrl_mode_conveyor_belt)){ROS_INFO("Waitinng for param: conveyor_belt/control_mode ");}
	while(!nh_.getParam("conveyor_belt/nominal_speed", _nominalSpeed_conveyor_belt)){ROS_INFO("Waitinng for param: conveyor_belt/nominal_speed");}
	while(!nh_.getParam("conveyor_belt/magnitude_perturbation", _magniture_pert_conveyor_belt)){ROS_INFO("Waitinng for param: conveyor_belt/magnitude_perturbation");}


	if (!gotParam) {
    ROS_ERROR("Couldn't the retrieve one or many parameters. ");
		return false;
  }
  	
	std::vector<float> param_damp_l;
	std::vector<float> param_damp_r;
	ros::param::getCached(param_damping_topic_TorqueCtrl_left,  param_damp_l);
	ros::param::getCached(param_damping_topic_TorqueCtrl_right, param_damp_r);
	if((!param_damp_l.empty()) && (!param_damp_r.empty())){ // nonEmpty
		_dsDampingTopic[LEFT]  = param_damping_topic_TorqueCtrl_left;
		_dsDampingTopic[RIGHT] = param_damping_topic_TorqueCtrl_left;
	} 
	else{
		_dsDampingTopic[LEFT]  = param_damping_topic_CustomCtrl_left;
		_dsDampingTopic[RIGHT] = param_damping_topic_CustomCtrl_right;
	}

	object_._objectMass 					  					= param_objectMass;
	object_._objectDim 						  					= Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(param_objectDim.data(), param_objectDim.size());
	Eigen::Vector3f graspOffset_L    	= Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(param_graspOffset_L.data(), param_graspOffset_L.size());
	Eigen::Vector3f graspOffset_R    	= Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(param_graspOffset_R.data(), param_graspOffset_R.size());
	
	_toolOffsetFromEE[0]  			  		= _isSimulation ? param_toolOffset_sim_left :  param_toolOffset_real_left;
	_toolOffsetFromEE[1]  			  		= _isSimulation ? param_toolOffset_sim_right :  param_toolOffset_real_right;
	_tossVar.release_position      	  = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(param_releasePos.data(), param_releasePos.size());
	_tossVar.release_orientation      = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(param_releaseOrient.data(), param_releaseOrient.size());
	_tossVar.release_linear_velocity  = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(param_releaseLinVel_dir.data(), param_releaseLinVel_dir.size());
	_tossVar.release_angular_velocity = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(param_releaseAngVel.data(), param_releaseAngVel.size());
	_tossVar.rest_position      	  	= Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(param_restPos.data(), param_restPos.size());
	_tossVar.rest_orientation      	  = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(param_restOrient.data(), param_restOrient.size());

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
	_dual_angular_limit = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(param_dual_angular_limit.data(), param_dual_angular_limit.size());
	

	// relative grasping positons
	_xgp_o[0] = Eigen::Vector3f(0.0f, -object_._objectDim(1)/2.0f,  0.0f) + graspOffset_L;   // left  
	_xgp_o[1] = Eigen::Vector3f(0.0f,  object_._objectDim(1)/2.0f,  0.0f) + graspOffset_R; 	 // right 

	// conveyor belt
	_desSpeed_conveyor_belt = _nominalSpeed_conveyor_belt;
	//
	Eigen::Matrix3f RDo_lifting = Utils<float>::quaternionToRotationMatrix(_qDo_lifting);
	_filt_delta_ang = Utils<float>::getEulerAnglesXYZ_FixedFrame(RDo_lifting);
	_filt_delta_ang_mir = Utils<float>::getEulerAnglesXYZ_FixedFrame(RDo_lifting);

	// ======================================================================================================
	// ROS TOPICS
	// ======================================================================================================
	// Subscribers
	//------------
	std::string topic_pose_target = "/simo_track/target_pose";

	_sub_object_pose 			 				= nh_.subscribe(_topic_pose_object,1, &dual_arm_control::objectPoseCallback, this, ros::TransportHints().reliable().tcpNoDelay());
	_sub_base_pose[LEFT] 	 	 			= nh_.subscribe<geometry_msgs::Pose>(_topic_pose_robot_base[LEFT], 1, 
																								boost::bind(&dual_arm_control::updateBasePoseCallback,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	_sub_ee_pose[LEFT] 			 			= nh_.subscribe<geometry_msgs::Pose>(_topic_pose_robot_ee[LEFT], 1, 
																								boost::bind(&dual_arm_control::updateEEPoseCallback,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	_sub_ee_velo[LEFT] 			 			= nh_.subscribe<geometry_msgs::Twist>("/simo_track/robot_left/ee_velo", 1, 
																								boost::bind(&dual_arm_control::updateEETwistCallback,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	_subForceTorqueSensor[LEFT] 	= nh_.subscribe<geometry_msgs::WrenchStamped>(_topic_subForceTorqueSensor[LEFT], 1, 
																								boost::bind(&dual_arm_control::updateRobotWrench,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	_sub_joint_states[LEFT]	 			= nh_.subscribe<sensor_msgs::JointState>("/iiwa1/joint_states", 1, 
																								boost::bind(&dual_arm_control::updateRobotStatesLeft,this,_1), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	_sub_base_pose[RIGHT] 		 		= nh_.subscribe<geometry_msgs::Pose>(_topic_pose_robot_base[RIGHT], 1, 
																								boost::bind(&dual_arm_control::updateBasePoseCallback,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	_sub_ee_pose[RIGHT]   		 		= nh_.subscribe<geometry_msgs::Pose>(_topic_pose_robot_ee[RIGHT], 1, 
																								boost::bind(&dual_arm_control::updateEEPoseCallback,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	_sub_ee_velo[RIGHT] 		 			= nh_.subscribe<geometry_msgs::Twist>("/simo_track/robot_right/ee_velo", 1, 
																								boost::bind(&dual_arm_control::updateEETwistCallback,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	_subForceTorqueSensor[RIGHT]	= nh_.subscribe<geometry_msgs::WrenchStamped>(_topic_subForceTorqueSensor[RIGHT], 1, 
																								boost::bind(&dual_arm_control::updateRobotWrench,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	_sub_joint_states[RIGHT]	 		= nh_.subscribe<sensor_msgs::JointState>("/iiwa_blue/joint_states", 1, 
																								boost::bind(&dual_arm_control::updateRobotStatesRight,this,_1), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	_sub_target_pose 			 				= nh_.subscribe(topic_pose_target,1, &dual_arm_control::targetPoseCallback, this, ros::TransportHints().reliable().tcpNoDelay());
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
	_pubDesiredVel_Quat[LEFT]   	= nh_.advertise<geometry_msgs::Pose>("/dual_arm_control/iiwa1/vel_quat", 1);     // "/passive_control/iiwa1/vel_quat" 
	_pubDesiredVel_Quat[RIGHT]  	= nh_.advertise<geometry_msgs::Pose>("/dual_arm_control/iiwa_blue/vel_quat", 1);

	// _pubDesiredTwist[LEFT] 		= nh_.advertise<geometry_msgs::Twist>("/dual_arm_control/robot_left/desired/ee_velocity", 1);  // /passive_control/iiwa1/des_twist
	// _pubDesiredTwist[RIGHT] 	 	= nh_.advertise<geometry_msgs::Twist>("/dual_arm_control/robot_right/desired/ee_velocity", 1);
	
	_pubDesiredTwist[LEFT] 		  	= nh_.advertise<geometry_msgs::Twist>("/passive_control/iiwa1/des_twist", 1);  //  "/dual_arm_control/robot_right/desired/ee_velocity"
	_pubDesiredTwist[RIGHT] 	 		= nh_.advertise<geometry_msgs::Twist>("/passive_control/iiwa_blue/des_twist", 1);

	_pubDistAttractorEe[LEFT] 		= nh_.advertise<std_msgs::Float64>("/dual_arm_control/iiwa1/error", 1);     							// "/passive_control/iiwa1/error"
	_pubDistAttractorEe[RIGHT] 		= nh_.advertise<std_msgs::Float64>("/dual_arm_control/iiwa_blue/error", 1);   						// "/passive_control/iiwa_blue/error"
  _pubAttractor[LEFT] 					= nh_.advertise<geometry_msgs::Pose>("/dual_arm_control/iiwa1/attractor", 1);							// "/passive_control/iiwa1/attractor"
	_pubAttractor[RIGHT] 					= nh_.advertise<geometry_msgs::Pose>("/dual_arm_control/iiwa_blue/attractor", 1);					// "/passive_control/iiwa_blue/attractor"
	_pubNormLinVel[LEFT]					= nh_.advertise<std_msgs::Float64>("/dual_arm_control/iiwa_left/lin_vel_norm", 1);
	_pubNormLinVel[RIGHT]					= nh_.advertise<std_msgs::Float64>("/dual_arm_control/iiwa_right/lin_vel_norm", 1);
	//
	_pubAppliedWrench[LEFT]				= nh_.advertise<geometry_msgs::Wrench>("/dual_arm_control/robot_left/applied_wrench", 1);
	_pubAppliedWrench[RIGHT]			= nh_.advertise<geometry_msgs::Wrench>("/dual_arm_control/robot_right/applied_wrench", 1);
	//
	_pubApplied_fnornMoment[LEFT] = nh_.advertise<geometry_msgs::Wrench>("/passive_control/iiwa1/ext_nforce_moments", 1);
	_pubApplied_fnornMoment[RIGHT]= nh_.advertise<geometry_msgs::Wrench>("/passive_control/iiwa_blue/ext_nforce_moments", 1);

	_pubConveyorBeltMode 					= nh_.advertise<std_msgs::Int32>("/conveyor_belt/desired_mode", 1); 
	_pubConveyorBeltSpeed 				= nh_.advertise<std_msgs::Int32>("/conveyor_belt/desired_speed", 1); 


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
	FreeMotionCtrl._objectDim = object_._objectDim;
	FreeMotionCtrl._dt = _dt;

	// initialize the cooperative controller
	//---------------------------------------
	CooperativeCtrl.init();
	_qp_wrench_generation = isQP_wrench_generation;
	//
	// initialization of the toss task parameter estimator
	// ----------------------------------------------------
	std::string path2LearnedModelfolder = ros::package::getPath(std::string("dual_arm_control")) +"/LearnedModel/model1";
	std::string file_gmm[3];
	std::string dataType = "/throwingParam";
	//
	file_gmm[0]     = path2LearnedModelfolder + dataType + "_prio.txt";
	file_gmm[1]     = path2LearnedModelfolder + dataType + "_mu.txt";
	file_gmm[2]     = path2LearnedModelfolder + dataType + "_sigma.txt";

	tossParamEstimator.init(file_gmm, _tossVar.release_position, _tossVar.release_orientation, 
							_tossVar.release_linear_velocity, _tossVar.release_angular_velocity);
	//
	target_._xd_landing = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
	//
	tossParamEstimator.estimate_tossing_param(toss_task_param_estimator::PHYS_IDEAL, target_._xd_landing, _tossVar.release_position);

	// Object tossing DS
	//------------------
	target_._xd_landing  = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
	target_._x_intercept = target_._xd_landing;
	target_._x_pickup 	 = _xo;

	// _tossVar.release_position = _xDo_placing + Eigen::Vector3f(0.0, 0.0, _height_via_point);

	// if PRESET
	// dsThrowing.init(dsThrowing.ds_param_, releasePos, releaseOrient, releaseLinVel, releaseAngVel, restPos, restOrient);
	dsThrowing.init(dsThrowing.ds_param_, _tossVar.release_position, _tossVar.release_orientation, 
									_tossVar.release_linear_velocity, _tossVar.release_angular_velocity,
									_tossVar.rest_position, _tossVar.rest_orientation);
	//
	// IF AUTOMATICALLY DETERMINED (USING RELEASE POSE GENERATOR)
	// dsThrowing.init(dsThrowing.ds_param_, 
	// 				tossParamEstimator.get_release_position(), tossParamEstimator.get_release_orientation(),
	// 				tossParamEstimator.get_release_linear_velocity(), tossParamEstimator.get_release_angular_velocity(),
	// 				_tossVar.rest_position, _tossVar.rest_orientation);
	// _desVtoss = tossParamEstimator.get_release_linear_velocity().norm();
	//
	dsThrowing.set_pickup_object_pose(_xo, _qo);
	// Eigen::Vector3f new_toss_velocity = _desVtoss * (_tossVar.release_position -_xo).normalized();
	_tossVar.release_linear_velocity = _desVtoss * (_tossVar.release_position -_xo).normalized();
	dsThrowing.set_toss_linear_velocity(_tossVar.release_linear_velocity);
	dsThrowing._refVtoss = _desVimp;

	// release_pos.from_cartesian(_tossVar.release_position -_xDo_lifting);

	// For estimation of predicted robot path
	dsThrowingEstim = dsThrowing;
	//
	FreeMotionCtrl._desVreach 				 = _desVreach;
	FreeMotionCtrl._refVreach[LEFT]  	 = _refVreach;
	FreeMotionCtrl._refVreach[RIGHT] 	 = _refVreach;
	FreeMotionCtrl._modulated_reaching = modulated_reaching;
	FreeMotionCtrl._isNorm_impact_vel  = isNorm_impact_vel;
	FreeMotionCtrl._height_via_point   = _height_via_point;
	//
	FreeMotionCtrlEstim = FreeMotionCtrl;

	// // KF filter for the object and target
	// _xo_KF_filtered.init(_dt, Eigen::Vector2f(0.004, 0.1), 0.004, _xo);
	// _xo_KF_filtered.update(_xo);


	// _xt_KF_filtered.init(_dt, Eigen::Vector2f(0.004, 0.1), 0.004, _xt);
	// _xt_KF_filtered.update(_xt);
	// _xt_KF_filtered.get_estimate_velocity();

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

// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Run
// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void dual_arm_control::run() 
{
	ROS_INFO("Running the dual_arm_control");
	_t0_run = ros::Time::now().toSec();
	// --------------------------------------------------------
	// 
	// --------------------------------------------------------
	while (nh_.ok()) {
		//
		auto start = std::chrono::high_resolution_clock::now();
		// update
		update_states_machines();
		// get the first eigen value of the passive ds controller and its updated value
		get_pasive_ds_1st_damping();
		
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

		// Estimation of the running period 
		auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << " [dual_arm_control] RUNNING PERIOD ------------> : \t " << duration.count() << " ms" << std::endl;

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
// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
						_startlogging = false;
						// datalog.datalog_reset(ros::package::getPath(std::string("dual_arm_control")) +"/Data");
					}
					if(!_goHome){_startlogging  = true;}
				}
				break;
				// case 'g': _goToAttractors = !_goToAttractors; break;  
				case 'g': {_goToAttractors = !_goToAttractors;
								if(_goToAttractors){
									_goHome			   = false;
									_releaseAndretract = false; 
								}
						   }
					break;
				// control of the object (desired using keyboard)
				// position
				// case 'a': _delta_pos(0) -= 0.01f; break;
				// case 's': _delta_pos(0) += 0.01f; break;
				// case 'd': _delta_pos(1) -= 0.01f; break;
				// case 'f': _delta_pos(1) += 0.01f; break;
				// case 'z': _delta_pos(2) -= 0.01f; break;
				// case 'w': _delta_pos(2) += 0.01f; break;
				// position
				case 'a':
							if(_ctrl_mode_conveyor_belt){ _mode_conveyor_belt = 2;
																						publish_conveyor_belt_cmds();
																						_startlogging  = true;
							}
				      else if(_increment_release_pos) _delta_rel_pos(0) -= 0.025f;  //_delta_rel_pos(0)  -= 0.05f;  //[m]
				      else                       _delta_pos(0)      -= 0.01f; 
				  break;
				case 's':
							if(_ctrl_mode_conveyor_belt){ _mode_conveyor_belt = 0; 
																						publish_conveyor_belt_cmds();}
				      else if(_increment_release_pos) _delta_rel_pos(0)  += 0.025f; //_delta_rel_pos(0)  += 0.05f; //[m]
				      else                       _delta_pos(0)      += 0.01f; 
				  break;
				case 'd': 
							if(_ctrl_mode_conveyor_belt){ _mode_conveyor_belt = 1;
																						publish_conveyor_belt_cmds();}
				      else if(_increment_release_pos) _delta_rel_pos(1)  -= 5.0f; //[deg]   _delta_rel_pos(1)  -= 0.025f; //
				      else                       _delta_pos(1)      -= 0.01f; 
				  break;
				case 'f': 
				      if(_increment_release_pos) _delta_rel_pos(1)  += 5.0f; //[deg]   _delta_rel_pos(1)  += 0.025f; //
				      else                       _delta_pos(1)      += 0.01f; 
				  break;
				case 'z':
							if(_ctrl_mode_conveyor_belt) _trackingFactor -=0.01f; 
				      else if(_increment_release_pos) _delta_rel_pos(2) -= 5.0f; //[deg]   _delta_rel_pos(2)  -= 0.025f; //
				      else                       _delta_pos(2)      -= 0.01f; 
				  break;
				case 'w':
							if(_ctrl_mode_conveyor_belt) _trackingFactor +=0.01f;  
				      else if(_increment_release_pos) _delta_rel_pos(2)  += 5.0f; //[deg]   _delta_rel_pos(2)  += 0.025f; //
				      else                       _delta_pos(2)      += 0.01f; 
				  break;
				//orientation
				case 'h': 
								if(_ctrl_mode_conveyor_belt) _nominalSpeed_conveyor_belt -=50;  
								else
									_delta_ang(0) -= 0.05f; 
					break;
				case 'j': if(_ctrl_mode_conveyor_belt) _nominalSpeed_conveyor_belt +=50;  
								else
									_delta_ang(0) += 0.05f; 
					break;
				case 'k':
							if(_ctrl_mode_conveyor_belt) _adaptationActive = !_adaptationActive;
							else _delta_ang(1) -= 0.05f; 
					break;
				// case 'l': _delta_ang(1) += 0.05f; break;
				case 'm':
							if(_ctrl_mode_conveyor_belt) _magniture_pert_conveyor_belt -=50;
							else _delta_ang(2) -= 0.05f; 
					break; 
				case 'i':
							if(_ctrl_mode_conveyor_belt) _magniture_pert_conveyor_belt +=50;
							else _delta_ang(2) += 0.05f; break;  

				// user control of release or throwing (keyboard)
				case 'r': _releaseAndretract = !_releaseAndretract; 
					break; 

				case 'l': { _dualTaskSelector = PICK_AND_LIFT; 
										_hasCaughtOnce = false;}
					break;    

				case 't':{ _isThrowing = !_isThrowing; 
							if(_isThrowing){
								_dualTaskSelector = PICK_AND_TOSS; //TOSSING; // toss
								_hasCaughtOnce = false;
							}
							else if(!_isThrowing){
								_dualTaskSelector = PICK_AND_LIFT; // toss
							}			
						 }
					break;  
				case 'p': {_isPlacing = !_isPlacing; 
							if(_isPlacing){
								_dualTaskSelector = PICK_AND_PLACE; 
								_hasCaughtOnce = false;
							}
							else if(!_isPlacing){
								_dualTaskSelector = PICK_AND_LIFT; 
							}				
						  }
					break;  
				case 'o': {_isPlaceTossing = !_isPlaceTossing; 
								if(_isPlaceTossing){
									_dualTaskSelector = PLACE_TOSSING; 
									_hasCaughtOnce = false;
								}
								else if(!_isPlaceTossing){
									_dualTaskSelector = PICK_AND_LIFT; 
								}				
						  }
					break;      
				// control of impact and tossing velocity
				case 'v': 
						_desVtoss -=0.05f; 
						if(_desVtoss < 0.2f) _desVtoss = 0.2f;
						dsThrowing.set_toss_linear_velocity(_desVtoss * _tossVar.release_linear_velocity.normalized());
						dsThrowingEstim.set_toss_linear_velocity(_desVtoss * _tossVar.release_linear_velocity.normalized());
					break;   
				case 'b':  
						_desVtoss +=0.05f; 
						if(_desVtoss > 2.0f) _desVtoss = 2.0f;
						dsThrowing.set_toss_linear_velocity(_desVtoss * _tossVar.release_linear_velocity.normalized());
						dsThrowingEstim.set_toss_linear_velocity(_desVtoss * _tossVar.release_linear_velocity.normalized());
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

				// disturb the target speed
				case 'e':
					_isDisturbTarget = !_isDisturbTarget;
				break;

				// placing hight
				case 'x':
					if(_dualTaskSelector == PICK_AND_TOSS){
						_tossVar.release_position(1)-=0.01;
					}
					else{
						_xDo_placing(2)-= 0.01; 
					} 
				break;

				// placing hight
				case 'n':
					if(_dualTaskSelector == PICK_AND_TOSS){
						_tossVar.release_position(1)+=0.01;
					}
					else{
						_xDo_placing(2)+= 0.01; 
					} 
				break;

			}
		}
		keyboard::Keyboard::nonblock_2(0);
		// ----------------------------------------------------
}

void dual_arm_control::updatePoses()
{
	if(_initPoseCount < 100){
		_w_H_eeStandby[LEFT]  = _w_H_rb[LEFT]  * Utils<float>::pose2HomoMx(_xrbStandby[LEFT],  _qrbStandby[LEFT]);		// WITH EE pose wrt. the world TO BE CHANGED
		_w_H_eeStandby[RIGHT] = _w_H_rb[RIGHT] * Utils<float>::pose2HomoMx(_xrbStandby[RIGHT], _qrbStandby[RIGHT]);		// WITH EE pose wrt. the world
		FreeMotionCtrl._w_H_eeStandby[LEFT]  = _w_H_eeStandby[LEFT];
		FreeMotionCtrl._w_H_eeStandby[RIGHT] = _w_H_eeStandby[RIGHT];
		//
		_xDo    = Eigen::Vector3f(_xo(0), _xo(1), _xDo_lifting(2));   // set attractor of lifting task   
		_qDo    = _qo; 																								// _qDo_lifting
		_w_H_Do = Utils<float>::pose2HomoMx(_xDo, _qDo);
		target_._x_intercept = Eigen::Vector3f(_xo(0), 0.0, _xo(2));
		// for catching
		FreeMotionCtrl.set_virtual_object_frame(Utils<float>::pose2HomoMx(target_._x_intercept, _qo));

		_initPoseCount ++;
	}
	
	// Estimation of moving average EE and target speed
	// this->estimate_moving_average_ee_speed();
	// this->estimate_moving_average_target_velocity();

	// Update the object position or its desired position (attractor) through keyboard 
	if(_objCtrlKey){ this->Keyboard_virtual_object_control(); }
	else{ this->Keyboard_reference_object_control(); }
	//
	if(_increment_release_pos){
		this->update_release_position();
	}
	
	// homogeneous transformations associated with the reaching task
	for(int k=0; k<NB_ROBOTS; k++){
		_w_H_ee[k]  = Utils<float>::pose2HomoMx(_x[k],  _q[k]);			// WITH EE pose wrt. the world
		_w_H_gp[k]  = _w_H_o * Utils<float>::pose2HomoMx(_xgp_o[k],  _qgp_o[k]);
		_w_H_Dgp[k] = _w_H_Do * Utils<float>::pose2HomoMx(_xgp_o[k],  _qgp_o[k]);
		_n[k]       = _w_H_gp[k].block(0,0,3,3).col(2);
		_err[k]     = (_w_H_ee[k].block(0,3,3,1)  - _w_H_gp[k].block(0,3,3,1)).norm();
		_o_H_ee[k]  = _w_H_o.inverse() * _w_H_ee[k];

		if(CooperativeCtrl._ContactConfidence == 1.0)
		{
			_o_H_ee[k](1,3) *= 0.95f; 
			_w_H_Dgp[k]  = _w_H_Do * _o_H_ee[k];
			// _w_H_gp[k]  = _w_H_o * _o_H_ee[k];
		}
	}

}

//
void dual_arm_control::get_pasive_ds_1st_damping(){
	std::vector<float> param_values;
	ros::param::getCached(_dsDampingTopic[LEFT], param_values);	  _d1[LEFT] = param_values[0];
	if(_d1[LEFT] <FLT_EPSILON) _d1[LEFT]   = 150.0f;
	ros::param::getCached(_dsDampingTopic[RIGHT], param_values);  _d1[RIGHT] = param_values[0];
	if(_d1[RIGHT]<FLT_EPSILON)  _d1[RIGHT] = 150.0f;
}

// // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void dual_arm_control::computeCommands()
{
	// Update contact state
  updateContactState();
 	// ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  // self imposed limits on intercept region (placing on moving target)
  float x_t_min = 0.60f;  
  float x_t_max = 0.75f;    
  float y_t_min =-0.10f;     
  float y_t_max = 0.10f;   

	float beta_vel_mod_unfilt = 1.0f;

  // ---------------------------------------------------------------------------------------------------------------------------------------
  bool no_dual_mds_method = _old_dual_method;
	bool isContact          = true && _sensedContact && CooperativeCtrl._ContactConfidence == 1.0f;
	bool isPlacing          = _isPlacing || (_dualTaskSelector == PICK_AND_PLACE);
	bool isThrowing         = _isThrowing || (_dualTaskSelector == TOSSING) || (_dualTaskSelector == PICK_AND_TOSS);
	bool isPlaceTossing 		= _isPlaceTossing || (_dualTaskSelector == PLACE_TOSSING);
	bool isClose2Release    = (dsThrowing.a_tangent_> 0.99f);
	//
	bool isPlacingCommand 	= (_release_flag) || ((_w_H_o.block<3,1>(0,3)-_xDo_placing).norm()<=0.07); // 0.07 0.05
	bool isTossingCommand 	= (_release_flag) || ((_w_H_o.block<3,1>(0,3)-_tossVar.release_position).norm()<=0.035);
	//
	bool placing_done 			= (_release_flag) || ((_w_H_o.block<3,1>(0,3)-_xDo_placing).norm()<=0.08); //0.05
	bool placeTossing_done 	= (_release_flag) || (((_w_H_o.block<3,1>(0,3)-_tossVar.release_position).norm()<=0.07) 
																						|| ((_w_H_o.block<2,1>(0,3)-_xDo_placing.head(2)).norm() <= 0.05 ) );
	bool tossing_done 			= (_release_flag) || ( ((_w_H_o.block<3,1>(0,3)-_tossVar.release_position).norm()<=0.035) );
	bool isForceDetected 		= (_normalForceAverage[LEFT] > _forceThreshold || _normalForceAverage[RIGHT] > _forceThreshold);


	// ----------------------------------------------------------------------------------------------------------------------------------------
	// Intercept/ landing location
	// ----------------------------------------------------------------------------------------------------------------------------------------
	// limits for throwing object's yaw angle
	// compute intercept position limits
	Eigen::Vector3f x_origin= target_._x_pickup;
	x_origin = Eigen::Vector3f(0.35, 0, 0);
	Eigen::Vector3f x_i_min = this->compute_intercept_with_target(x_origin, target_._xt,  target_._vt, -_dual_angular_limit(2));
	Eigen::Vector3f x_i_max = this->compute_intercept_with_target(x_origin, target_._xt,  target_._vt,  _dual_angular_limit(2));

	// self imposed limits on intercept region (placing on moving target)
	float intercep_limits[4];
	intercep_limits[0] = 0.60f;  			// x_min
	intercep_limits[1] = 0.75f;  			// x_max
	intercep_limits[2] = x_i_min(1); 	// -0.10f;  // y_min
	intercep_limits[3] = x_i_max(1); 	//  0.10f;  // y_max

	std::cout << " CCCCCCCCCCCCCCCCCCCCCCC Intercept limits MIN \t " << x_i_min.transpose() << std::endl;
	std::cout << " CCCCCCCCCCCCCCCCCCCCCCC Intercept limits MAX \t " << x_i_max.transpose() << std::endl;

	// // // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// 		// // ---------------------------------------------------------------------------------------------------------------------------------
	// 		// // calibration to estimate the tracking Factor
	// 		// // ---------------------------------------------------------------------------------------------------------------------------------
	// 		// if(!_isPickupSet && !_releaseAndretract){
	// 		// 	// dxEE_dual_ = 0.5f*(VEE[LEFT] + VEE[RIGHT]);
	// 		// 	// _dxEE_dual_avg 		 += (0.5f*(_Vee[LEFT].head(3) + _Vee[RIGHT].head(3)).norm() - _dxEE_dual_avg_0)/(_counter_monocycle + 1);
	// 		// 	_dxEE_dual_avg 		 += (Utils<float>::get_abs_3d(_Vee, true).norm() - _dxEE_dual_avg_0)/(_counter_monocycle + 1);
	// 		// 	_xEE_dual 					= Utils<float>::get_abs_3d(_w_H_ee); //0.5f*(_w_H_ee[LEFT].block(0,3,3,1) + _w_H_ee[RIGHT].block(0,3,3,1));
	// 		// 	_Del_xEE_dual_avg  += (_xEE_dual- _xEE_dual_0).norm();
	// 		// 	//
	// 		// 	_dxEE_dual_avg_0 = _dxEE_dual_avg;
	// 		// 	_xEE_dual_0 		 = _xEE_dual;

	// 		// 	_updatePathEstim = false;
	// 		// 	_counter_monocycle ++;
	// 		// }
	// 		// // //
	// 		// // float new_trackingFactor = 1.0f;
	// 		// // if(_isPickupSet && !_updatePathEstim && _releaseAndretract){
	// 		// // 	//
	// 		// // 	_dxEE_dual_avg_pcycle += (_dxEE_dual_avg - _dxEE_dual_avg_pcycle)/(_counter_pickup + 1);
	// 		// // 	_dxEE_dual_avg_pcycle  = _dxEE_dual_avg;
	// 		// // 	//
	// 		// // 	new_trackingFactor = min((_dxEE_dual_avg_pcycle/(_dual_PathLen_AvgSpeed(1)+1e-5)), 1.0);
	// 		// // 	_trackingFactor 	 = _trackingFactor + (new_trackingFactor - _trackingFactor)/(_counter_pickup + 1);
	// 		// // 	//
	// 		// // 	_counter_monocycle = 0;
	// 		// // 	_counter_pickup ++;
	// 		// // 	_updatePathEstim = true;
	// 		// // }
	
	// // ----------------------------------------------------------------------------------------------------------------------------------------
	// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//
	// ===========================================================================================================
	// Application
	// ===========================================================================================================
	if( (!_releaseAndretract) && (fmod(_cycle_count, 20)==0)){
		_dual_PathLen_AvgSpeed = FreeMotionCtrlEstim.predictRobotTranslation( _w_H_ee, _w_H_gp,  _w_H_eeStandby, _w_H_o, _tossVar.release_position, _desVtoss, 0.05f, 0.100f, _initSpeedScaling);
	}
	// -------------------------------------------------------------
	Eigen::Vector2f Lp_Va_pred_bot = {_dual_PathLen_AvgSpeed(0), _trackingFactor *_dual_PathLen_AvgSpeed(1)}; 							
	Eigen::Vector2f Lp_Va_pred_tgt = tossParamEstimator.estimateTarget_SimpPathLength_AverageSpeed(target_._xt, target_._xd_landing, target_._vt); 	
	//
	float flytime_obj = 0.200f; //0.255f;
	float eps_den = 1e-6;
	float time2intercept_tgt = fabs(fabs(Lp_Va_pred_tgt(0) +eps_den - Lp_Va_pred_tgt(1)*flytime_obj)/(Lp_Va_pred_tgt(1)+eps_den));
	float time2intercept_bot = Lp_Va_pred_bot(0)/Lp_Va_pred_bot(1);
	//
	_time2intercept_tgt = 0.0f;
	_time2intercept_bot = 0.0f;
	// -------------------------------------------------------------

	// determine the desired landing position
	this->find_desired_landing_position(x_origin, isPlacing, isPlaceTossing, isThrowing); 	// ---> _xd_landing

	// this->set_2d_position_box_constraints(_xd_landing, intercep_limits);
	std::cout << " DDDDDDDDDDDDDDDD  XD LANDING IS : \t " << target_._xd_landing.transpose() << std::endl;

	// Estimate the target state to go
	this->estimate_target_state_to_go(Lp_Va_pred_bot, Lp_Va_pred_tgt, flytime_obj);  	// ---> _xt_state2go

	// set at pickup instant
	if(!_isPickupSet && !_releaseAndretract)
	{
		if(_sensedContact && (CooperativeCtrl._ContactConfidence == 1.0)){

			// Update intercept (desired landing) position
			this->update_intercept_position(flytime_obj, intercep_limits);    	// ---> _xd_landing
			// Determination of the release configuration
			this->find_release_configuration();	// ---> _tossVar.release_position  _tossVar.release_linear_velocity
			// set the release state and the object pickup position
			this->set_release_state();    			// <----- _tossVar.release_position  _tossVar.release_linear_velocity
			//
			_isPickupSet = true;
		}
		else{
			target_._x_pickup = _xo;
			dsThrowing.set_pickup_object_pose(target_._x_pickup, _qo);
		}
	}

	// Adaptation of the desired motion 
	this->compute_adaptation_factors(Lp_Va_pred_bot, Lp_Va_pred_tgt, flytime_obj);
	// // ===========================================================================================================
	// // ===========================================================================================================

	
	// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	if(_goHome)
		// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	{
		//
		FreeMotionCtrl.computeAsyncMotion(_w_H_ee, _w_H_eeStandby, _w_H_o, _Vd_ee, _qd, true);		
		_Vd_o  = dsThrowing.apply(_xo, _qo, _vo, Eigen::Vector3f(0.0f, 0.0f, 0.0f), 1);  												// 	Function to call in a loop
		Eigen::Vector3f obj_dir = this->get_object_desired_direction(_dualTaskSelector, _w_H_o.block(0,3,3,1));

		std::cout << "[dual_arm_control]: OBJECT MOTION DIRECTION : \t"  << obj_dir.transpose() << std::endl;
		//
		for(int i=0; i<NB_ROBOTS; i++){
			_dirImp[i]  		 	= this->get_impact_direction(_Vd_o.head(3), _n[i], _friction_angle);								//  impact direction
			_VdImpact[LEFT]  	= _desVimp * _dirImp[LEFT]; 																												//	impact velocity [LEFT];
			_VdImpact[RIGHT]	= _desVimp * _dirImp[RIGHT]; 																												//	impact velocity [RIGHT];
			_BasisQ[i] 				= Utils<float>::create3dOrthonormalMatrixFromVector(_dirImp[i]);										//  Orthogonal Basis of Modulated Dual-arm DS
		}
		// reset some controller variables
		this->reset_variables();
		if(_adaptationActive){
			_tossVar.release_position(1) = 0.0f;
			_xDo_placing(1) = 0.0f;
		}
		//
		// if( ((_initPoseCount > 50) && ((xt_bar - xt2go_bar).norm() < 0.04f)) && (!_hasCaughtOnce) ) 					// 0.80     // tossing isMotionTriggered
		// if((_initPoseCount > 50) && (fabs(_xo(1)-y_2_go) < 0.02f) && (!_hasCaughtOnce)) 											// 0.80   	// catching
		if(_isMotionTriggered && (!_hasCaughtOnce))
		{
			_goHome 				= false;
			_hasCaughtOnce 	= true;
			// _startlogging  = true;
		}
		//
		_dxEE_dual_avg 		= 0.0f; //  _dual_PathLen_AvgSpeed(1); //
		_Del_xEE_dual_avg = 0.0f;
	}
	else 
	{
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			if(_releaseAndretract) //  release_and_retract || release
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			{
				FreeMotionCtrl.computeReleaseAndRetractMotion(_w_H_ee, _w_H_Dgp,  _w_H_o, _Vd_ee, _qd, true);
				_isThrowing 		= false;
				_isPlacing 			= false;
				_isPickupSet 		= false;
				_isPlaceTossing = false;
				_nu_Wr0 = _nu_Wr1 = 0.0f;
				dsThrowing.reset_release_flag();
				_isIntercepting = false;
			}
			// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			else // reaching and constrained motion
			// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			{
	      if((isPlacing && isPlacingCommand) || (isThrowing && isTossingCommand)){
	        _releaseAndretract = true;
	      }

	      if(isContact){
	        _Vd_o  = dsThrowing.apply(_xo, _qo, _vo, Eigen::Vector3f(0.0f, 0.0f, 0.0f), 1);  // Function to call in a loop
	        _releaseAndretract = dsThrowing.get_release_flag();
	        			// Release and Retract condition
					//------------------------------
					if(  (isPlacing && placing_done) || (isPlaceTossing && placeTossing_done) || (isThrowing && tossing_done) ){
						_releaseAndretract = true;
					}
	      }
	      else
	      {
	    		dsThrowing._refVtoss = _desVimp;
	        _Vd_o.setZero();  // for data logging
	      }
	      // //
	      // if(isPlacing){ this->update_placing_position(y_t_min, y_t_max); }
	      // if(isPlacing || isPlaceTossing){ this->constrain_placing_position(x_t_min, x_t_max, y_t_min, y_t_max); }
	      // ------------------------------------------------------------------
	      // Desired object pose
				//--------------------
	      Eigen::Vector3f xDesTask = _xDo_lifting;
				Eigen::Vector4f qDesTask = _qDo_lifting;

				// desired task position and orientation vectors
				//----------------------------------------------
				if(isPlacing){
					xDesTask = _xDo_placing;
					qDesTask = _qDo_placing;
				}
				if(isPlaceTossing){
					xDesTask = _xDo_placing;
					qDesTask = _qDo_placing;
				}
				if(isThrowing){
					xDesTask = _tossVar.release_position;
					qDesTask = _tossVar.release_orientation;
				}
				// Target to object Orientation Adaptation
				// ----------------------------------------
				// if(_trackTargetRotation && !(isThrowing || isPlaceTossing)){  // isPlacing || 
				// 	this->mirror_target2object_orientation(_qt, qDesTask, _dual_angular_limit);
				// }
				if(_trackTargetRotation){
					this->mirror_target2object_orientation(target_._qt, qDesTask, _dual_angular_limit);
				}
				_w_H_Do = Utils<float>::pose2HomoMx(xDesTask, qDesTask);  //
				//
				// Desired pose of the grasping points
				//------------------------------------
				for(int k=0; k<NB_ROBOTS; k++){ 
					_w_H_Dgp[k].block(0,0,3,3)  = _w_H_Do.block(0,0,3,3) * Utils<float>::pose2HomoMx(_xgp_o[k], _qgp_o[k]).block(0,0,3,3);
				}

	      // ---------------------------------------------------------------
	      FreeMotionCtrl.getDesiredMotion(no_dual_mds_method,
		                                    isContact, 
		                                    isPlacing,
		                                    isThrowing,
		                                    isClose2Release,
		                                    _dualTaskSelector,
		                                    _w_H_ee, 
		                                    _xgp_o,
		                                    _qgp_o,
		                                    _o_H_ee,
		                                    _w_H_o, 
		                                    _w_H_Do,
		                                    _xDo_placing,
		                                    qDesTask, //_qDo_placing,
		                                    _tossVar.release_position,
		                                    qDesTask, //_tossVar.release_orientation,
		                                    _height_via_point,
		                                    _Vee,
		                                    _Vd_o,
		                                    _BasisQ,
		                                    _VdImpact,
		                                    _w_H_Dgp,
		                                    _Vd_ee, 
		                                    _qd, 
		                                    _release_flag);
	      // ----------------------------------------------------------------
			
			} // reaching and constrained motion

			///////////////////////////////////////////////////////////
			if(isPlacing || isThrowing || isPlaceTossing ){
				// force feedback to grab objects
				float f_gain = 0.02f;
				float abs_force_correction = _nu_Wr0 *f_gain * 0.5f*( (_filteredWrench[LEFT].segment(0,3)  - CooperativeCtrl._f_applied[LEFT].head(3)).dot(_n[LEFT])  
																		 + (_filteredWrench[RIGHT].segment(0,3) - CooperativeCtrl._f_applied[RIGHT].head(3)).dot(_n[RIGHT]) );   
				if(fabs(abs_force_correction)  > 0.2f){
				abs_force_correction = abs_force_correction/fabs(abs_force_correction) * 0.2f;
				}
				_Vd_ee[LEFT].head(3)  =  _Vd_ee[LEFT].head(3)  - 0.40*abs_force_correction * _n[LEFT];   // 0.6 (heavy objects)
				_Vd_ee[RIGHT].head(3) =  _Vd_ee[RIGHT].head(3) - 0.40*abs_force_correction * _n[RIGHT];  // 0.6 (heavy objects)
			}

			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Adaptation
			// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			//
			if(!isContact && (FreeMotionCtrl.a_proximity_ >= 0.2f)){
				beta_vel_mod_unfilt = 1.0;
			}
			// -----------------------------------------------------------------------------------------------------------------------------
			float fil_beta = 0.10;
			_beta_vel_mod = (1.f- fil_beta)*_beta_vel_mod + fil_beta * beta_vel_mod_unfilt;

			if((target_._vt.norm() >= 0.05 && (!_releaseAndretract) && (dsThrowing.a_proximity_<=0.99f))){
				_Vd_ee[LEFT].head(3)  *= _initSpeedScaling * ((float)_adaptationActive * _beta_vel_mod + (1. - (float)_adaptationActive));
				_Vd_ee[RIGHT].head(3) *= _initSpeedScaling * ((float)_adaptationActive * _beta_vel_mod + (1. - (float)_adaptationActive));
			}
			// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// compute the object's grasp points velocity
			// 
			getGraspPointsVelocity();
		  //
			// Desired object's task wrench
		  if(true){
		  	// _desired_object_wrench.head(3) = -40.0f * (_w_H_o.block(0,3,3,1) - _w_H_Do.block(0,3,3,1))- object_._objectMass * _gravity;
		  	_desired_object_wrench.head(3) = -12.64f * (_vo - FreeMotionCtrl.get_des_object_motion().head(3)) - object_._objectMass * _gravity;
		  }
		  else{
		  	float Damp1 = 0.5f*(_d1[LEFT]+_d1[RIGHT]);
		  	_desired_object_wrench.head(3) = 0.0*Damp1*(Utils<float>::get_abs_3d(_Vee, true) - Utils<float>::get_abs_3d(_Vd_ee, true)) - object_._objectMass * _gravity;
		  }
			_desired_object_wrench.tail(3) = -25.0f*(_wo - FreeMotionCtrl.get_des_object_motion().tail(3));

		  // Generate grasping force and apply it in velocity space
	  	//--------------------------------------------------------
		  CooperativeCtrl.getAppliedWrenches(_goHome, _contactState, _w_H_o, _w_H_ee, _w_H_gp, _desired_object_wrench, object_._objectMass, _qp_wrench_generation, isForceDetected);

		  // applied force in velocity space
		  for(int i=0; i<NB_ROBOTS; i++){
		  	_fxc[i] = 1.0f/_d1[i] * CooperativeCtrl._f_applied[i].head(3);
		  }
	}
	// compute the velocity to avoid EE collision
	FreeMotionCtrl.compute_EE_avoidance_velocity(_w_H_ee, _VEE_oa);

  // Extract linear velocity commands and desired axis angle command
	prepareCommands(_Vd_ee, _qd, _V_gpo);

	// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// control of conveyor belt speed
	// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	float omega_pert = 2.f*M_PI/1;
	float delta_omega_pert = 0.1f*(2.f*(float) std::rand()/RAND_MAX - 1.0f)*omega_pert;
	// if(_ctrl_mode_conveyor_belt && _isDisturbTarget){
	if(_ctrl_mode_conveyor_belt){
			_desSpeed_conveyor_belt = (int)(_nominalSpeed_conveyor_belt + (int)_isDisturbTarget * _magniture_pert_conveyor_belt * sin((omega_pert + delta_omega_pert) * _dt * _cycle_count));
	}
	

	std::cout << " MEASURED HAND WRENCH _filteredWrench  LEFT \t " << _filteredWrench[LEFT].transpose() << std::endl;
	std::cout << " MEASURED HAND WRENCH _filteredWrench RIGHT \t " << _filteredWrench[RIGHT].transpose() << std::endl; 

	std::cout << "[dual_arm_control]: _w_H_o: \n" << _w_H_o << std::endl; 
	std::cout << "[dual_arm_control]: _w_H_Do: \n" <<  _w_H_Do << std::endl;
	std::cout << "[dual_arm_control]: _w_H_t: \n" <<  Utils<float>::quaternionToRotationMatrix(target_._qt) << std::endl;
	std::cout << "[dual_arm_control]: _w_H_ee[LEFT]: \n" <<  _w_H_ee[0] << std::endl;
	std::cout << "[dual_arm_control]: _w_H_Dgp[LEFT]: \n" << _w_H_Dgp[0] << std::endl;
	std::cout << "[dual_arm_control]: _w_H_ee[RIGHT]: \n" << _w_H_ee[1] << std::endl;
	std::cout << "[dual_arm_control]: _w_H_Dgp[RIGHT]: \n" << _w_H_Dgp[1] << std::endl;

	std::cout << "[dual_arm_control]: 3D STATE 2 GO : \t"  << target_._xt_state2go.transpose() << std::endl;
	std::cout << "[dual_arm_control]:  ------------- _sensedContact: \t" << _sensedContact << std::endl;
	std::cout << "[dual_arm_control]: _Vd_ee[LEFT]:  \t" << _Vd_ee[LEFT].transpose() << std::endl;
	std::cout << "[dual_arm_control]: _Vd_ee[RIGHT]: \t" << _Vd_ee[RIGHT].transpose() << std::endl;
	std::cout << " vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv " <<  std::endl;
	std::cout << "[dual_arm_control]: _vd[LEFT]:  \t" << _vd[LEFT].transpose() << std::endl;
	std::cout << "[dual_arm_control]: _vd[RIGHT]: \t" << _vd[RIGHT].transpose() << std::endl;
	std::cout << " ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ " <<  std::endl;
	std::cout << " COMPUTED HAND WRENCH _fxc  LEFT \t " << _fxc[LEFT].transpose() << std::endl;
	std::cout << " COMPUTED HAND WRENCH _fxc RIGHT \t " << _fxc[RIGHT].transpose() << std::endl; 
	// std::cout << " [dual_arm_control]: _dirImp[LEFT] \t " << _dirImp[LEFT].transpose()   << " normal LEFT  \t " << _n[LEFT].transpose()<< std::endl;
	// std::cout << " [dual_arm_control]: _dirImp[RIGHT] \t " << _dirImp[RIGHT].transpose() << " normal RIGHT \t " << _n[RIGHT].transpose()<< std::endl;
	std::cout << " EEEE----------- EEEPPP   _desVtoss IIIIIIII ----------- ONNNNNNNN \t " << _desVtoss << std::endl; 
	std::cout << " EEEE----------- EEEPPP   _desVimp  IIIIIIII ----------- ONNNNNNNN \t " <<  _desVimp <<  std::endl; 
	// std::cout << " EEEE---- RELEASE POSITION  IIIIIIII ----------- ONNNNNNNN \t " <<  _tossVar.release_position.transpose() <<  std::endl;
	// std::cout << " EEEE---- RELEASE POSITION  SPHERICAL  -- r: \t " <<  release_pos.r ;
	// std::cout << " theta: \t " <<  180.f/M_PI * release_pos.theta;
	// std::cout << " phi: \t " <<  180.f/M_PI * release_pos.phi <<  std::endl;
	// std::cout << " EEEE- POSITION PLACING is  \t " << _xDo_placing.transpose() << std::endl; 
	// std::cout << " EEEE- POSITION LIFTING is  \t " << _xDo_lifting.transpose() << std::endl; 
	std::cout << " EEEE- OBJECT MASS is  \t " << object_._objectMass << std::endl;  
	std::cout << " CONVEYOR_BELT SPEED is  \t " << _desSpeed_conveyor_belt << std::endl;
	std::cout << " CONVEYOR_BELT DISTURBED is  \t " << _isDisturbTarget << std::endl;
	// std::cout << " CONVEYOR_BELT PERTURBATION is  \t " << (int) (_nominalSpeed_conveyor_belt + (int)_isDisturbTarget *_magniture_pert_conveyor_belt * sin((2.f*M_PI/1) * _dt * _cycle_count)) << std::endl;
	std::cout << " INTERCEPT STATUS is  \t " << _hasCaughtOnce << std::endl;

	std::cout << " HOME STATUS is --------------------------> : \t " << _goHome << std::endl; 
	std::cout << " RELEASE_AND_RETRACT STATUS is -----------> : \t " << _releaseAndretract << std::endl; 
	switch(_dualTaskSelector){
		case 0: std::cout << " DUAL_ARM MODE is -----------------------> : \t " << "REACHING-TO-GRASP" << std::endl; break;
		case 1: std::cout << " DUAL_ARM MODE is -----------------------> : \t " << "LIFTING" << std::endl; break;
		case 2: std::cout << " DUAL_ARM MODE is -----------------------> : \t " << "TOSSING" << std::endl; break;
		case 3: std::cout << " DUAL_ARM MODE is -----------------------> : \t " << "PICK_AND_TOSS" << std::endl; break;
		case 4: std::cout << " DUAL_ARM MODE is -----------------------> : \t " << "PICK_AND_PLACE" << std::endl; break;
		case 5: std::cout << " DUAL_ARM MODE is -----------------------> : \t " << "PLACE-TOSSING" << std::endl; break;
	}
	std::cout << " AAAA  TRACKING FACTOR AAAAA is  \t " << _trackingFactor << std::endl; 
	std::cout << " AAAA  ADAPTATION STATUS AAAAA is  -----------> : \t " << _adaptationActive << std::endl; 
	std::cout << " PPPPPPPPPPPPPPP _xDo_placing PPPPPPPPPP  is  -----------> : \t " << _xDo_placing.transpose() << std::endl; 
	std::cout << " TTTTTTTTTTTTTTT _xTarget TTTTTTTTTTT  is  -----------> : \t " << target_._xt.transpose() << std::endl; 
	//-------------------------------------
	
	// -------------------------------------------------------------------------------
	std::cout << " PPPPPPPPPPPPPPP _dual_PathLen PPPPPPPPPP  is  -----------> : \t " << _dual_PathLen_AvgSpeed(0) << std::endl; 
	std::cout << " PPPPPPPPPPPPPPP _dual_Path_AvgSpeed(1) PPPPPPPPPP  is  -----------> : \t " << _dual_PathLen_AvgSpeed(1) << std::endl; 
	std::cout << " PPPPPPPPPPPPPPP _Del_xEE_dual_avg PPPPPPPPPP  is  -----------> : \t " << _Del_xEE_dual_avg << std::endl; 
	std::cout << " PPPPPPPPPPPPPPP _dxEE_dual_avg PPPPPPPPPP  is  -----------> : \t " << _dxEE_dual_avg_pcycle << std::endl;  
	std::cout << " PPPPPPPPPPPPPPP _counter_pickup PPPPPPPPPP  is  -----------> : \t " << _counter_pickup << std::endl;

	
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void dual_arm_control::prepareCommands(Vector6f Vd_ee[], Eigen::Vector4f qd[], Vector6f V_gpo[])
{
	Eigen::Matrix<float,3,1> axis_d[NB_ROBOTS];	float angle_d[NB_ROBOTS];
  for(int i=0; i<NB_ROBOTS; i++)
  {
  	// _vd[i]      = Vd_ee[i].head(3)  + _V_gpo[i].head(3);  
  	// _omegad[i]  = Vd_ee[i].tail(3)  + _V_gpo[i].tail(3);  
  	Vector6f VdEE  = _tcp_W_EE[i].inverse() * (Vd_ee[i]  + _V_gpo[i]);
  	//
  	_vd[i]     = VdEE.head(3) + 1.0*_VEE_oa[i].head(3);
		_omegad[i] = VdEE.tail(3);

  	Utils<float>::quaternionToAxisAngle(qd[i], axis_d[i], angle_d[i]);
  	_aad[i] = angle_d[i] * axis_d[i];
  }

  // --------------------------------------------------------
  if(_goToAttractors && _sensedContact && CooperativeCtrl._ContactConfidence == 1.0f)
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
	  	_vd[i] = _applyVelo *_vd[i] + _nu_Wr0 * _fxc[i];
	  }
}

//
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
    _V_gpo[i].head(3) = _vo - 0*skew_Mx_gpo * _wo;
    _V_gpo[i].tail(3) = 0*_wo;
    //
    _V_gpo[i].head(3) *= 0.0f;
    _V_gpo[i].tail(3) *= 0.0f;
  }
}

void dual_arm_control::Keyboard_reference_object_control()   // control of attractor position through keyboad
{
  // _w_H_Do(0,3) += _delta_pos(0);
  // _w_H_Do(1,3) += _delta_pos(1);
  // _w_H_Do(2,3) += _delta_pos(2);
	_xDo_lifting(0) += _delta_pos(0);
	_xDo_lifting(1) += _delta_pos(1);
	_xDo_lifting(2) += _delta_pos(2);

	// Eigen::Matrix3f RDo_lifting = Utils<float>::quaternionToRotationMatrix(_qDo_lifting);
	// Eigen::Vector3f eulerAng_lift = Utils<float>::getEulerAnglesXYZ_FixedFrame(RDo_lifting);
	// eulerAng_lift += _delta_ang;
	// RDo_lifting  = Utils<float>::eulerAnglesToRotationMatrix(	eulerAng_lift(2), eulerAng_lift(1), eulerAng_lift(0));
	// _qDo_lifting = Utils<float>::rotationMatrixToQuaternion(RDo_lifting);
	//

	// _filt_delta_ang = 0.95*_filt_delta_ang + 0.05*_delta_ang;
	// // Eigen::Matrix3f RDo_lifting  = Utils<float>::eulerAnglesToRotationMatrix(	_filt_delta_ang(2), _filt_delta_ang(1), _filt_delta_ang(0));
	// Eigen::Matrix3f RDo_lifting  = Utils<float>::eulerAnglesToRotationMatrix(	_filt_delta_ang(0), _filt_delta_ang(1), _filt_delta_ang(2));
	// _qDo_lifting = Utils<float>::rotationMatrixToQuaternion(RDo_lifting);
	// //
	// std::cout << "AAAAAAAAAAAAAAAAAA ROTATION ANGLE EULER XYZ \t" << _filt_delta_ang.transpose() << std::endl;
	//

  _delta_pos.setZero(); 
  // _delta_ang.setZero();
}

void dual_arm_control::mirror_target2object_orientation(Eigen::Vector4f qt, Eigen::Vector4f &qo, Eigen::Vector3f ang_lim)
{
	Eigen::Vector3f eAng_t = Utils<float>::getEulerAnglesXYZ_FixedFrame(Utils<float>::quaternionToRotationMatrix(qt));
	Eigen::Vector3f eAng_o = Utils<float>::getEulerAnglesXYZ_FixedFrame(Utils<float>::quaternionToRotationMatrix(qo));

	// eAng_o(2) = eAng_o(2) + eAng_t(2);
	
	_filt_delta_ang_mir = 0.95*_filt_delta_ang_mir + 0.05*(eAng_t - eAng_o);
	eAng_t(0) = eAng_o(0) + _filt_delta_ang_mir(0);
	eAng_t(2) = eAng_o(2) + _filt_delta_ang_mir(2);

	if(eAng_t(0) >= ang_lim(0)){
		eAng_t(0)   = ang_lim(0);
	}
	else if(eAng_t(0) <= -ang_lim(0)){
		eAng_t(0)   = -ang_lim(0);
	}
	//
	if(eAng_t(2) >= ang_lim(2)){
		eAng_t(2)   = ang_lim(2);
	}
	else if(eAng_t(2) <= -ang_lim(2)){
		eAng_t(2)   = -ang_lim(2);
	}

	std::cout << " QUAT q0 before is : \t" << qo.transpose() << std::endl;
	
	// qo = Utils<float>::rotationMatrixToQuaternion( Utils<float>::eulerAnglesToRotationMatrix(eAng_t(2), eAng_o(1), eAng_o(0)) );
	qo = Utils<float>::rotationMatrixToQuaternion( Utils<float>::eulerAnglesToRotationMatrix(eAng_o(0), eAng_o(1), eAng_t(2)) );
	// qo = Utils<float>::rotationMatrixToQuaternion( Utils<float>::eulerAnglesToRotationMatrix(eAng_o(0), eAng_o(1), eAng_o(2)) );

	std::cout << " QUAT q0 After is : \t" << qo.transpose() << std::endl;
}

// //
void dual_arm_control::Keyboard_virtual_object_control()		// control of object position through keyboad
{
	// object2grasp.States_Object.pose.head(3) = ioSM->w_H_absF.block<3,3>(0,0) * init_obj_aF + ioSM->w_H_absF.block<3,1>(0,3); //
	_w_H_o(0,3) += _delta_pos(0);
	_w_H_o(1,3) += _delta_pos(1);
	_w_H_o(2,3) += _delta_pos(2); // _delta_ang

  Eigen::Vector3f ang_o = Utils<float>::getEulerAnglesXYZ_FixedFrame(_w_H_o.block<3,3>(0,0)); // psi theta phi
  // 
  // ang_o += _delta_ang;
  // _w_H_o.block<3,3>(0,0) = Utils<float>::eulerAnglesToRotationMatrix(	ang_o(2), ang_o(1), ang_o(0)); // phi theta psi

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
	_isIntercepting = false;
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
		case 5:  des_object_pos = 0.5f*(object_pos + _xDo_placing);
						 des_object_pos(2)=_xDo_placing(2) + _height_via_point; 		break;
		default: des_object_pos = _xDo; 						break;
	}
	//
	Eigen::Vector3f error_obj_pos = des_object_pos - object_pos;
	return error_obj_pos.normalized();
}

//
void dual_arm_control::update_release_position(){
	release_pos.r     += _delta_rel_pos(0);
	release_pos.theta += M_PI/180.0f * _delta_rel_pos(1);
	release_pos.phi   += M_PI/180.0f * _delta_rel_pos(2);
	//
	// _xDo_placing += _delta_rel_pos; 
	//
	_delta_rel_pos.setZero(); 
	// // //
	Eigen::Vector3f pos_xo; 
	release_pos.to_cartesian(pos_xo);
	_tossVar.release_position = pos_xo + _xDo_lifting; //_xo;
	if(_tossVar.release_position(0) > 0.70){  // 0.65
			_tossVar.release_position(0) = 0.70;
	}
}

void dual_arm_control::update_placing_position(float y_t_min, float y_t_max){
  _xDo_placing.head(2) = _xDo_placing.head(2) + _adaptationActive * _dt * (- 5.0f*(_xDo_placing.head(2) - target_._xt.head(2)));
  if( (target_._xt(1) >= y_t_min) && (target_._xt(1) <= y_t_max)){
    _xDo_placing.head(2) = _xDo_placing.head(2) + _adaptationActive * _dt * (target_._vt.head(2) - 5.0f*(_xDo_placing.head(2) - target_._xt.head(2)));
  }
}

void dual_arm_control::constrain_placing_position(float x_t_min, float x_t_max, float y_t_min, float y_t_max){
  if(_xDo_placing(0)<x_t_min) _xDo_placing(0) = x_t_min;
  if(_xDo_placing(0)>x_t_max) _xDo_placing(0) = x_t_max;
  if(_xDo_placing(1)<y_t_min) _xDo_placing(1) = y_t_min;
  if(_xDo_placing(1)>y_t_max) _xDo_placing(1) = y_t_max;
}

void dual_arm_control::set_2d_position_box_constraints(Eigen::Vector3f &position_vec, float limits[]){
	if(position_vec(0)<limits[0]) position_vec(0) = limits[0]; 	// x_min
  if(position_vec(0)>limits[1]) position_vec(0) = limits[1];	// x_max
  if(position_vec(1)<limits[2]) position_vec(1) = limits[2]; 	// y_min
  if(position_vec(1)>limits[3]) position_vec(1) = limits[3];	// y_max
}

Eigen::Vector3f dual_arm_control::compute_intercept_with_target(const Eigen::Vector3f &x_pick, 
									                                              const Eigen::Vector3f &x_target, 
									                                              const Eigen::Vector3f &v_target, 
									                                              float phi_i){
  //
  float phi_target = 0.0f;
  if(v_target.head(2).norm() > 1e-2){
    phi_target = std::atan2(v_target(1), v_target(0));
  }
  float tan_phi_t = std::tan(phi_target);
  if(tan_phi_t > 1e4){
  	tan_phi_t = 1e4;
  }
  if(-tan_phi_t< -1e4){
  	tan_phi_t = -1e4;
  }
  //
  Eigen::Vector3f x_i = x_target;
  // Eigen::Vector2f x_i_0;
  // x_i_0(0) = x_target(1) - tan_phi_t*x_target(0),
  // x_i_0(1) = x_pick(1) - std::tan(phi_i)*x_pick(0);
  // Eigen::Matrix2f Ti = Eigen::MatrixXf::Identity(2,2);
  // Ti << -tan_phi_t, 1.0f,
  //            -std::tan(phi_i), 1.0f;
  // x_i.head(2) = Ti.inverse() * x_i_0;
 	// x_i(0) = -1.0f/(std::tan(phi_i) - tan_phi_t) * (x_pick(1) - std::tan(phi_i)*x_pick(0) - (x_target(1) - tan_phi_t*x_target(0)));
	// x_i(1) =  1.0f/(std::tan(phi_i) - tan_phi_t) * ( (std::tan(phi_i)*(x_target(1) - tan_phi_t*x_target(0))) - (tan_phi_t*(x_pick(1) - std::tan(phi_i)*x_pick(0)) ) );
  //
  float x_coord_land	 = x_target(0);
	float y_coord_land	 = x_target(1);
	float tang_phi_throw = std::tan(phi_i);

	std::cout << " TTTTTTTTTTTTTTTTT PHI Throw  TTTTTTTTTTTis  -----------> : \t " << 180.f/M_PI *phi_i << std::endl;


	if(v_target.head(2).norm()> 1e-2){
		float phi_conveyor = std::atan2(v_target(1), v_target(0));
		float tang_phi_conv  = std::tan(phi_conveyor);
		x_coord_land	 = ((tang_phi_conv*x_target(0) - x_target(1))/(tang_phi_conv - tang_phi_throw));
	}
	else{
		x_coord_land	 = x_target(0);
	} 
	y_coord_land	   = x_coord_land * tang_phi_throw;
	//
	x_i  << x_coord_land, y_coord_land, x_target(2);

  return x_i;
}

float dual_arm_control::get_desired_yaw_angle_target(const Eigen::Vector4f &qt, const Eigen::Vector3f &ang_lim)
{
  Eigen::Vector3f eAng_t = Utils<float>::getEulerAnglesXYZ_FixedFrame(Utils<float>::quaternionToRotationMatrix(qt));

  float phi_t_rot = eAng_t(2);
  //
  if(phi_t_rot >= ang_lim(2)){
    phi_t_rot   = ang_lim(2);
  }
  else if(phi_t_rot <= -ang_lim(2)){
    phi_t_rot   =-ang_lim(2);
  }
  
  return phi_t_rot;
}

void dual_arm_control::estimate_moving_average_ee_speed(){
	// Estimation of moving average EE  and speed
	// -------------------------------------
	float avgSpeedEE = Utils<float>::get_abs_3d(_Vee, true).norm(); //0.5f*(_Vee[LEFT].head(3).norm() + _Vee[RIGHT].head(3).norm());

	//
  if(_windowSpeedEE.size() < _winLengthAvgSpeedEE){   	
    _windowSpeedEE.push_back(avgSpeedEE);
    _movingAvgSpeedEE = 0.0f;
  }
  else{
    _windowSpeedEE.pop_front();
    _windowSpeedEE.push_back(avgSpeedEE);
    _movingAvgSpeedEE = 0.0f;

    for(int i = 0; i < _winLengthAvgSpeedEE; i++){
			_movingAvgSpeedEE+=_windowSpeedEE[i]/_winLengthAvgSpeedEE;
    }
  }
}

void dual_arm_control::estimate_moving_average_target_velocity(){
	// Estimation of moving average target velocity
	// --------------------------------------------
  if(_windowVelTarget.size() < _winLengthAvgSpeedEE){   	
    _windowVelTarget.push_back(target_._vt);
    _movingAvgVelTarget.setZero();
  }
  else{
    _windowVelTarget.pop_front();
    _windowVelTarget.push_back(target_._vt);
    _movingAvgVelTarget.setZero();

    for(int i = 0; i < _winLengthAvgSpeedEE; i++){
			_movingAvgVelTarget +=_windowVelTarget[i] * (1.f/_winLengthAvgSpeedEE);
    }
  }
}


void dual_arm_control::find_desired_landing_position(Eigen::Vector3f x_origin, bool isPlacing, bool isPlaceTossing, bool isThrowing){
	// // determine the throwing/placing direction 
	// float feas_yaw_target = this->get_desired_yaw_angle_target(_qt, _dual_angular_limit);
	// float phi_throwing    = feas_yaw_target; 

	// // determine the intercept or desired landing position  
	// // ----------------------------------------------------
	// if(_isTargetFixed){
	//   if( isPlacing || isPlaceTossing ){
	//     phi_throwing = std::atan2(_xDo_placing(1), _xDo_placing(0));
	//   }
	//   if( isThrowing ){
	//     phi_throwing = std::atan2(_tossVar.release_position(1), _tossVar.release_position(0));
	//   }
	//   _xd_landing = this->compute_intercept_with_target(x_origin, _xt,  _vt, phi_throwing);
	// }
	// else{
	//   _xd_landing = this->compute_intercept_with_target(x_origin, _xt,  _vt, feas_yaw_target);

	// }
	// ----------------------------------------------------------------------------------------
	Eigen::Vector3f xd_land = target_._xt;
	float phi_throwing      = 0.0; 	

	// determine the intercept or desired landing position  
	// ----------------------------------------------------------------------------------------
	if(userSelect_){
		//
		if( isPlacing || isPlaceTossing ){
			xd_land.head(2) = _xDo_placing.head(2);
		}
		if(isThrowing){
			xd_land.head(2) = _tossVar.release_position.head(2);
		}
		//
		phi_throwing = std::atan2(xd_land(1), xd_land(0));

		if(_isTargetFixed){
			target_._xd_landing = xd_land;
		}
		else{
			target_._xd_landing = this->compute_intercept_with_target(x_origin, target_._xt,  target_._vt, phi_throwing);
		}
	}
	else{ // autoSelect_
		phi_throwing = this->get_desired_yaw_angle_target(target_._qt, _dual_angular_limit);
		//
		target_._xd_landing  = this->compute_intercept_with_target(x_origin, target_._xt,  target_._vt, phi_throwing);
	}
	// -----------------------------------------------------------------------------------------


}

//
void dual_arm_control::update_intercept_position(float flytime_obj, float intercep_limits[]){

	Eigen::Vector3f x_t_intercept = target_._xt + target_._vt*(_time2intercept_bot + flytime_obj);
	// apply constraints
	this->set_2d_position_box_constraints(x_t_intercept, intercep_limits); 
	//
	if(_adaptationActive){
		target_._xd_landing.head(2) = x_t_intercept.head(2);
		if(_isPlaceTossing || (_dualTaskSelector == PLACE_TOSSING) ){
			_xDo_placing.head(2) = target_._xt.head(2) + 0.35*target_._vt.head(2)*(_time2intercept_bot + flytime_obj);
		}
	}
}

void dual_arm_control::find_release_configuration(){ // xD_landing
	// basic release configuration
	// ----------------------------
	if(_feasibleAlgo){
		// generate using feasibilty algorithm
		_tossVar.release_position.head(2) = target_._xd_landing.head(2); // 

		// extract from topics of the dual feasibility algo

		// 
	}
	else if(_pickupBased){
		//
		// _tossVar.release_position.head(2) = target_._xd_landing.head(2); // 
		//
		Eigen::Vector3f x_release_bar = _tossVar.release_position - target_._x_pickup;
		float phi_throw_bar = std::atan2(x_release_bar(1), x_release_bar(0));

		_tossVar.release_linear_velocity << _tossVar.release_linear_velocity.head(2).norm() * std::cos(phi_throw_bar),
																				_tossVar.release_linear_velocity.head(2).norm() * std::sin(phi_throw_bar),
																				_tossVar.release_linear_velocity(2);
		_tossVar.release_linear_velocity =  _desVtoss * _tossVar.release_linear_velocity.normalized();
	}
	else{ // user-defined
		_tossVar.release_position 			 = _tossVar.release_position;
		_tossVar.release_linear_velocity = _desVtoss*_tossVar.release_linear_velocity.normalized();
	}
	
}

void dual_arm_control::set_release_state(){
	// set the release state and the object pickup position
	//-------------------------------------------------------
	dsThrowing.set_toss_pose(_tossVar.release_position, _tossVar.release_orientation);
	dsThrowing.set_toss_linear_velocity(_tossVar.release_linear_velocity);
	dsThrowing.set_pickup_object_pose(target_._x_pickup, _qo);
}

//
void dual_arm_control::estimate_target_state_to_go(Eigen::Vector2f Lp_Va_pred_bot, Eigen::Vector2f Lp_Va_pred_tgt, float flytime_obj){
	// Estimation of the target state-to-go
	// -------------------------------------
	// Target state to go
	target_._xt_state2go = tossParamEstimator.estimate_target_state_to_go(target_._xt, target_._vt, target_._xd_landing, Lp_Va_pred_bot, Lp_Va_pred_tgt, flytime_obj);

	// boolean robot's motion trigger
	Eigen::Vector3f xt_bar 		= target_._xt - target_._xd_landing;
	Eigen::Vector3f xt2go_bar = target_._xt_state2go - target_._xd_landing;
	_isMotionTriggered 				= (-xt_bar.dot(target_._vt) > 0) && ((_initPoseCount > 50) && ((xt_bar - xt2go_bar).norm() < 0.04f));

}


void dual_arm_control::compute_adaptation_factors(Eigen::Vector2f Lp_Va_pred_bot, Eigen::Vector2f Lp_Va_pred_tgt, float flytime_obj){
	if(_isMotionTriggered){
		_isIntercepting = true; 
	}
	//
	float beta_vel_mod_max = min( 2.0f, min((_v_max/_Vd_ee[LEFT].head(3).norm()), (_v_max/_Vd_ee[RIGHT].head(3).norm())) );
	//
	if(_isIntercepting && !_releaseAndretract){
		//
		float eps_den = 1e-6;
		_time2intercept_tgt = fabs(fabs(Lp_Va_pred_tgt(0) +eps_den - Lp_Va_pred_tgt(1)*flytime_obj)/(Lp_Va_pred_tgt(1)+eps_den));
		_time2intercept_bot = Lp_Va_pred_bot(0)/Lp_Va_pred_bot(1);

		// Velocity-based adaptation factor
		//----------------------------------
		if(_isRatioFactor){
			_beta_vel_mod_unfilt = fabs((Lp_Va_pred_tgt(1)/(Lp_Va_pred_bot(1) +eps_den)) * (Lp_Va_pred_bot(0)/fabs(Lp_Va_pred_tgt(0) +eps_den - Lp_Va_pred_tgt(1)*flytime_obj) ));
		}
		else{
			_beta_vel_mod_unfilt = (std::tanh(7.0f * (_time2intercept_bot - _time2intercept_tgt)) + 1.0 );
		}
		if(_beta_vel_mod_unfilt >=beta_vel_mod_max){ 
			_beta_vel_mod_unfilt = beta_vel_mod_max; 
		}
		// Attractor-based adaptation factor
		//-----------------------------------
		FreeMotionCtrl._activationAperture = _adaptationActive ? 0.5f*(std::tanh(_switchSlopeAdapt * ((target_._xd_landing - target_._xt).normalized().dot(target_._vt) - _tol_attractor)) + 1.0) : 1.0f;
	}
	else{
		_beta_vel_mod_unfilt = 1.0f;
		FreeMotionCtrl._activationAperture = 1.0f;
	}
}	

// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Callback functions
// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// current_object_pose_world_callback
// -------------------------------------
void dual_arm_control::objectPoseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	// _xo << msg->position.x, 	msg->position.y, 	msg->position.z;
	Eigen::Vector3f xom, t_xo_xom; // _objectDim
	if(!_isSimulation){
		t_xo_xom << 0.0f, 0.0f, -object_._objectDim(2)/2.0f;
	}
	else{
		t_xo_xom << 0.0f, 0.0f, 0.0f;
	}
	xom << msg->position.x, 	msg->position.y, 	msg->position.z;
	_qo << msg->orientation.w, 	msg->orientation.x, msg->orientation.y, msg->orientation.z;
	// _w_H_o = Utils<float>::pose2HomoMx(_xo, _qo);
	Eigen::Matrix3f w_R_o = Utils<float>::quaternionToRotationMatrix(_qo);
	_xo = xom + w_R_o*t_xo_xom;

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

	_Vo.head(3) = _vo;
	_Vo.tail(3) = _wo;

	_w_H_o = Utils<float>::pose2HomoMx(_xo, _qo);
}

void dual_arm_control::targetPoseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	target_._xt << msg->position.x, 	msg->position.y, 	msg->position.z;
	target_._qt << msg->orientation.w, 	msg->orientation.x, msg->orientation.y, msg->orientation.z;
	// filtered object position

	target_.get_filtered_state();
	// SGF::Vec temp(3);
  // _xt_filtered->AddData(_xt);
  // _xt_filtered->GetOutput(0,temp);
  // _xt = temp;
  // _xt_filtered->GetOutput(1,temp);
  // _vt = temp;	
  // _xt_KF_filtered.update(_vt);
	// _vt = _xt_KF_filtered.get_estimate_position();
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

	// update velocity Twist transformation from EE to tcp
	Eigen::Vector3f tcp = _toolOffsetFromEE[k]*_wRb[k].col(2);
	Eigen::Matrix3f skew_Mx_tcp; 
	skew_Mx_tcp <<   0.0f,   -tcp(2),     tcp(1),
	               tcp(2),      0.0f,    -tcp(0),
	              -tcp(1),    tcp(0),       0.0f;

	_tcp_W_EE[k].block(0,3,3,3) = skew_Mx_tcp;
	//
	// _w_H_ee[k]  = Utils<float>::pose2HomoMx(_x[k],  _q[k]);			// WITH EE pose wrt. the world
}

void dual_arm_control::updateEETwistCallback(const geometry_msgs::Twist::ConstPtr& msg, int k)
{
  _v[k] << msg->linear.x, msg->linear.y, msg->linear.z;
  _w[k] << msg->angular.x, msg->angular.y, msg->angular.z;
	_Vee[k].head(3)  = _v[k];
	_Vee[k].tail(3)  = _w[k];
	_Vee[k] = _tcp_W_EE[k] * _Vee[k];
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

void dual_arm_control::updateRobotStatesLeft(const sensor_msgs::JointState::ConstPtr &msg) 
{
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

void dual_arm_control::updateRobotStatesRight(const sensor_msgs::JointState::ConstPtr &msg) 
{
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
  // Compute errors to object center position and dimension vector
	Eigen::Matrix4f le_H_re     =  _w_H_ee[LEFT].inverse() * _w_H_ee[RIGHT];
	Eigen::Matrix4f lgp_H_rgp   =  _w_H_gp[LEFT].inverse() * _w_H_gp[RIGHT];
	Eigen::Vector3f t_o_absEE   =  Utils<float>::get_abs_3d(_w_H_gp) - Utils<float>::get_abs_3d(_w_H_ee);
	_eoD = fabs(le_H_re(2,3)) - fabs(lgp_H_rgp(2,3)); 
	_eoC = t_o_absEE.norm(); 

  // if(_normalForceAverage[LEFT] > 2.0f && _normalForceAverage[RIGHT] > 2.0f &&  _eoD < 0.05f && _eoC < 0.1f)
  // if (_normalForceAverage[LEFT] > _forceThreshold || _normalForceAverage[RIGHT] > _forceThreshold && (CooperativeCtrl._ContactConfidence == 1.0f)) 
  if ((_normalForceAverage[LEFT] > 2.0f || _normalForceAverage[RIGHT] > 2.0f) && _eoD < 0.065f && (_eoC < 0.065f || CooperativeCtrl._ContactConfidence == 1.0f))
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

// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Publish commands and data
// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

		// applied wrench
		geometry_msgs::Wrench msgAppliedWrench;
		msgAppliedWrench.force.x  = -_nu_Wr0 *CooperativeCtrl._f_applied[k](0);
    msgAppliedWrench.force.y  = -_nu_Wr0 *CooperativeCtrl._f_applied[k](1);
    msgAppliedWrench.force.z  = -_nu_Wr0 *CooperativeCtrl._f_applied[k](2);
    msgAppliedWrench.torque.x = -_nu_Wr0 *CooperativeCtrl._f_applied[k](3);
    msgAppliedWrench.torque.y = -_nu_Wr0 *CooperativeCtrl._f_applied[k](4);
    msgAppliedWrench.torque.z = -_nu_Wr0 *CooperativeCtrl._f_applied[k](5);
		_pubAppliedWrench[k].publish(msgAppliedWrench);

		// contact normal and applied moment
		geometry_msgs::Wrench msgFnormMoment;
		msgFnormMoment.force.x 	= _n[k](0);
		msgFnormMoment.force.y 	= _n[k](1);
		msgFnormMoment.force.z 	= _n[k](2);
		msgFnormMoment.torque.x = -CooperativeCtrl._f_applied[k](3);
		msgFnormMoment.torque.y = -CooperativeCtrl._f_applied[k](4);
		msgFnormMoment.torque.z = -CooperativeCtrl._f_applied[k](5);
		_pubApplied_fnornMoment[k].publish(msgFnormMoment);
  }

  // send speed command to the conveyor belt
  std_msgs::Int32 _speedMessage;
  _speedMessage.data = _desSpeed_conveyor_belt;
  if(_ctrl_mode_conveyor_belt && (fmod(_cycle_count, 30)==0)){
	_pubConveyorBeltSpeed.publish(_speedMessage);
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

	// if(_startlogging)
	// {
		datalog._OutRecord_pose			<< (float)(_cycle_count * _dt) << ", ";																																																						// cycle time
		datalog._OutRecord_pose			<< _x[LEFT].transpose().format(CSVFormat)  	<< " , " << _q[LEFT].transpose().format(CSVFormat) 	<< " , ";																					// left end-effector
		datalog._OutRecord_pose   	<< _x[RIGHT].transpose().format(CSVFormat) 	<< " , " << _q[RIGHT].transpose().format(CSVFormat) << " , ";																					// right end-effector
		datalog._OutRecord_pose   	<< _xo.transpose().format(CSVFormat) 			 	<< " , " << _qo.transpose().format(CSVFormat) 			<< " , ";																					// object
		datalog._OutRecord_pose   	<< _w_H_Do(0,3) << " , " << _w_H_Do(1,3) 		<< " , " << _w_H_Do(2,3) << " , ";																																		// desired object
		datalog._OutRecord_pose   	<< xgrL.transpose().format(CSVFormat) 			<< " , " << qgrL.transpose().format(CSVFormat) 			<< " , ";																					// left  grasping point
		datalog._OutRecord_pose   	<< xgrR.transpose().format(CSVFormat) 			<< " , " << qgrR.transpose().format(CSVFormat) 			<< " , ";																					// right grasping point
		datalog._OutRecord_pose   	<< _tossVar.release_position.transpose().format(CSVFormat) 	<< " , " << _tossVar.release_orientation.transpose().format(CSVFormat) << " , ";			// release pose
		datalog._OutRecord_pose   	<< _tossVar.rest_position.transpose().format(CSVFormat) 			<< " , " << _tossVar.rest_orientation.transpose().format(CSVFormat)  << " , ";			// rest pose 
		datalog._OutRecord_pose   	<< target_._xt.transpose().format(CSVFormat) 				<< " , " << target_._qt.transpose().format(CSVFormat)   << " , ";																							// target pose
		datalog._OutRecord_pose   	<< target_._xd_landing.transpose().format(CSVFormat) 				<< " , " << target_._x_intercept.transpose().format(CSVFormat)   << " , ";														// landing and intercept position
		datalog._OutRecord_pose   	<< target_._xt_state2go.transpose().format(CSVFormat) << " , " << _xDo_placing.transpose().format(CSVFormat)<<  std::endl;																																											// target state to go   
		
		datalog._OutRecord_velo			<< (float)(_cycle_count * _dt) << ", ";
		datalog._OutRecord_velo			<< _Vd_ee[LEFT].transpose().format(CSVFormat) << " , " << _Vd_ee[RIGHT].transpose().format(CSVFormat) << " , ";
		datalog._OutRecord_velo			<< _Vee[LEFT].transpose().format(CSVFormat) 	<< " , " << _Vee[RIGHT].transpose().format(CSVFormat) 	<< " , ";
		datalog._OutRecord_velo			<< _vd[LEFT].transpose().format(CSVFormat) 		<< " , " << _vd[RIGHT].transpose().format(CSVFormat) 		<< " , ";
		datalog._OutRecord_velo			<< _omegad[LEFT].transpose().format(CSVFormat)<< " , " << _omegad[RIGHT].transpose().format(CSVFormat)<< " , ";
		datalog._OutRecord_velo			<< _vo.transpose().format(CSVFormat) 					<< " , " << _wo.transpose().format(CSVFormat) 					<< " , ";
		datalog._OutRecord_velo			<< _Vd_o.transpose().format(CSVFormat) << " , ";
		datalog._OutRecord_velo			<< _tossVar.release_linear_velocity.transpose().format(CSVFormat) << " , " <<  _tossVar.release_angular_velocity .transpose().format(CSVFormat) << " , ";
		datalog._OutRecord_velo			<< target_._vt.transpose().format(CSVFormat) << std::endl;
				
		datalog._OutRecord_efforts	<< (float)(_cycle_count * _dt) << ", ";
		datalog._OutRecord_efforts	<< _filteredWrench[LEFT].transpose().format(CSVFormat)  << " , ";
		datalog._OutRecord_efforts	<< _filteredWrench[RIGHT].transpose().format(CSVFormat) << " , ";
		datalog._OutRecord_efforts  << CooperativeCtrl._f_applied[LEFT].transpose().format(CSVFormat) << " , ";
		datalog._OutRecord_efforts  << CooperativeCtrl._f_applied[RIGHT].transpose().format(CSVFormat) << std::endl;
	
		datalog._OutRecord_tasks		<< (float)(_cycle_count * _dt) << ", ";
		datalog._OutRecord_tasks   	<< _desVimp << " , " << _desVtoss << " , "; 
		datalog._OutRecord_tasks   	<< _goHome 	<< " , " << _goToAttractors << " , " << _releaseAndretract << " , " << _isThrowing << " , " << _isPlacing << " , " << _c << " , "; 			//CooperativeCtrl._ContactConfidence << " , ";
		datalog._OutRecord_tasks   	<< FreeMotionCtrl.a_proximity_ << " , " << FreeMotionCtrl.a_normal_ << " , " << FreeMotionCtrl.a_tangent_ << " , " << FreeMotionCtrl.a_release_ << " , " << FreeMotionCtrl.a_retract_ << " , ";
		// datalog._OutRecord_tasks   	<< dsThrowing.a_proximity_ << " , " << dsThrowing.a_normal_  << " , " << dsThrowing.a_tangent_<< " , " << dsThrowing.a_toss_  << std::endl;
		datalog._OutRecord_tasks   	<< dsThrowing.a_proximity_ << " , " << dsThrowing.a_normal_  << " , " << dsThrowing.a_tangent_ << " , " << dsThrowing.a_toss_  << " , "; 
		datalog._OutRecord_tasks   	<< _beta_vel_mod << " , " << _dual_PathLen_AvgSpeed.transpose() << std::endl;
		// 
		datalog._OutRecord_jts_states << (float)(_cycle_count * _dt) << ", ";
		datalog._OutRecord_jts_states << _joints_positions[LEFT].transpose().format(CSVFormat)  		<< " , " << _joints_positions[RIGHT].transpose().format(CSVFormat)  		<< " , ";
		datalog._OutRecord_jts_states << _joints_velocities[LEFT].transpose().format(CSVFormat)  		<< " , " << _joints_velocities[RIGHT].transpose().format(CSVFormat)  		<< " , ";
		datalog._OutRecord_jts_states << _joints_accelerations[LEFT].transpose().format(CSVFormat)  << " , " << _joints_accelerations[RIGHT].transpose().format(CSVFormat)  << " , " ;
		datalog._OutRecord_jts_states << _joints_torques[LEFT].transpose().format(CSVFormat)   			<< " , " <<  _joints_torques[RIGHT].transpose().format(CSVFormat) 			<< " , " ;
		datalog._OutRecord_jts_states << power_left(0,0)   << " , " <<  power_right(0,0) << std::endl;
	// }

}	

void dual_arm_control::publish_conveyor_belt_cmds(){
	// desired conveyor belt contro mode
	std_msgs::Int32 _modeMessage;
	_modeMessage.data = _mode_conveyor_belt;
	_pubConveyorBeltMode.publish(_modeMessage);
}