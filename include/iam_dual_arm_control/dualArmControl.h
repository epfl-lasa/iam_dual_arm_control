#pragma once

#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <pthread.h>
#include <signal.h>
#include <sstream>
#include <stdio.h>
#include <termios.h>
#include <vector>

#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/WrenchStamped.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include <ros/package.h>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "sg_filter.h"

#include "ObjectToGrasp.hpp"
#include "RobotVariables.hpp"
#include "TossingTarget.hpp"
#include "data_logging.hpp"
#include "dualArmCooperativeController.h"
#include "dualArmFreeMotionController.h"
#include "throwingDS.h"
#include "toss_task_param_estimator.h"

// TODO use const or constexpr
#define NB_ROBOTS 2// Number of robots
#define NB_FT_SENSOR_SAMPLES                                                                                           \
  50// Number of force torque sensors' samples used for initial calibration (compute the offsets)
#define NB_OPTITRACK_SAMPLES 100   // Number of optitrack samples used for initial objects' pose estimation
#define NB_TRACKED_OBJECTS 6       // Number of objects tracked by the motion capture system (optitrack)
#define MOVING_FORCE_WINDOW_SIZE 10// Window's size used to average the force data and detect peristent contact

#define NB_OBJECTS 3// Number of objects

// typedef Eigen::Matrix<float, 7, 1> Vector7f;
// typedef Eigen::Matrix<float, 6, 1> Vector6f;
// typedef Eigen::Matrix<float, 6, 6> Matrix6f;

//! reading keyboard functions TODO
int khbit_2();
void nonblock_2(int state);
bool keyState_2(char key);

struct SphericalPosition {
  float r;
  float theta;
  float phi;

  void fromCartesian(Eigen::Vector3f pos) {
    r = pos.norm();
    theta = std::atan2(pos(1), pos(0));
    phi = std::atan2(pos(2), pos.head(2).norm());
  }

  void toCartesian(Eigen::Vector3f& pos) {
    pos(0) = r * std::cos(phi) * std::cos(theta);
    pos(1) = r * std::cos(phi) * std::sin(theta);
    pos(2) = r * std::sin(phi);
  }
};

class dualArmControl {
private:
  ///////////////////
  // ROS variables //
  ///////////////////
  ros::NodeHandle nh_;// Ros node handle
  ros::Rate loopRate_;// Ros loop rate [Hz]
  SGF::real dt_;

  float timeStartRun_;
  int cycleCount_;

  // TODO function in private?
  // Callback called when CTRL is detected to stop the node
  static void stopNode(int sig);
  //  static dualArmControl* me; // Pointer on the instance of the class

  std::string topic_pose_target_;

  //////////////////////////////
  // Publishers:
  //////////////////////////////
  ros::Publisher pubTSCommands_[NB_ROBOTS];        // Publisher of the End effectors velocity twist
  ros::Publisher pubDesiredTwist_[NB_ROBOTS];      // Publish desired twist to DS-impdedance controller
  ros::Publisher pubDesiredOrientation_[NB_ROBOTS];// Publish desired orientation to DS-impedance controller
  ros::Publisher pubFilteredWrench_[NB_ROBOTS];    // Publish filtered measured wrench
  ros::Publisher pubNormalForce_[NB_ROBOTS];       // Publish measured normal force to the surface
  ros::Publisher pubDesiredVelQuat_[NB_ROBOTS];    // Publish desired EE linear velocity and quaternion

  ros::Publisher pubDistAttractorEE_[NB_ROBOTS];
  ros::Publisher pubAttractor_[NB_ROBOTS];
  ros::Publisher pubNormLinVel_[NB_ROBOTS];// Publish norms of EE linear velocities

  ros::Publisher pubAppliedWrench_[NB_ROBOTS];     // Publish applied EE wrench
  ros::Publisher pubAppliedFNormMoment_[NB_ROBOTS];// Publish the contact normal and the moment of the applied wrench

  ros::Publisher pubConveyorBeltMode_; // Publish conveyor belt mode
  ros::Publisher pubConveyorBeltSpeed_;// Publish conveyor belt Speed

public:
  // Robot ID: left or right
  enum Robot { LEFT = 0, RIGHT = 1 };

  // Contact state:
  // CONTACT: Both robots are in contact with the object
  // CLOSE_TO_CONTACT: Both robots are close to make contact with the object
  // NO_CONTACT: Both robots are not in contact with the object
  enum ContactState { CONTACT = 0, CLOSE_TO_CONTACT = 1, NO_CONTACT = 2 };
  // TaskType: dual-arm reaching and manipulation tasks
  enum TaskType {
    REACH = 0,
    PICK_AND_LIFT = 1,
    TOSSING = 2,
    PICK_AND_TOSS = 3,
    PICK_AND_PLACE = 4,
    PLACE_TOSSING = 5,
    THROWING = 6,
    HANDINGOVER = 7,
    PAUSE_MOTION = 8
  };
  // 0=reach, 1=pick, 2=toss, 3=pick_and_toss, 4=pick_and_place

protected:
  std::mutex mutex;

  // int _nb_joints[NB_ROBOTS];

  // TODO
  //////////////////////////////
  // Subscribers declarations //
  //////////////////////////////
  // ros::Subscriber _sub_object_pose;
  // ros::Subscriber _sub_target_pose;
  // ros::Subscriber _sub_base_pose[NB_ROBOTS];       // subscribe to the base pose of the robots
  // ros::Subscriber _sub_ee_pose[NB_ROBOTS];         // subscribe to the end effectors poses
  // ros::Subscriber _sub_ee_velo[NB_ROBOTS];         // subscribe to the end effectors velocity Twist
  // ros::Subscriber _subForceTorqueSensor[NB_ROBOTS];// Subscribe to force torque sensors
  // ros::Subscriber _sub_joint_states[NB_ROBOTS];    // subscriber for the joint position

  //////////////////////////////
  // List of the topics
  //////////////////////////////
  std::string _topic_pose_object;
  std::string _topic_pose_robot_base[NB_ROBOTS];
  std::string _topic_pose_robot_ee[NB_ROBOTS];
  std::string _topic_ee_commands[NB_ROBOTS];
  std::string _topic_subForceTorqueSensor[NB_ROBOTS];

  // Velocity commands to be sent to the robots
  std_msgs::Float64MultiArray _pubVelo[NB_ROBOTS];// velocity Twist data to be published

  geometry_msgs::WrenchStamped _msgFilteredWrench;

  // --------------------------------------------------------------------------------
  // robot
  robot_var robot_;

  float _toolMass;                   // Tool mass [kg]
  float _toolOffsetFromEE[NB_ROBOTS];// Tool offset along z axis of end effector [m]
  Eigen::Vector3f _gravity;
  Eigen::Vector3f _toolComPositionFromSensor;
  int _wrenchCount[NB_ROBOTS];// Counter used to pre-process the force data
  ContactState _contactState; // Contact state with the object
  std::deque<float> _normalForceWindow
      [NB_ROBOTS];// Moving window saving the robots' measured normal force to the object's surface [N]
  float _normalForceAverage[NB_ROBOTS];// Average normal force measured through the force windows [N]
  float _normalForce[NB_ROBOTS];       // Normal force to the surface [N]
  float _c;                            // Contact value (1 = CONTACT, 0 otherwise)
  bool _wrenchBiasOK[NB_ROBOTS];       // Check if computation of force/torque sensor bias is OK

  float _eoD;// Error to object dimension vector [m]
  float _eoC;// Error to object center position [m]
  // --------------------------------------------------------------------------------------------

  float _Fd[NB_ROBOTS];// Desired force profiles [N]
  float _targetForce;  // Target force in contact [N]
  float _d1[NB_ROBOTS];
  float _err[NB_ROBOTS];
  bool _qp_wrench_generation;
  bool _firstRobotPose[NB_ROBOTS];
  bool _firstRobotTwist[NB_ROBOTS];
  bool _firstWrenchReceived[NB_ROBOTS];
  // ---------------------------------------------------------------------------------------------
  bool _sensedContact;

  bool _startlogging;

  Eigen::Vector3f _delta_pos;// variation of object position
  Eigen::Vector3f _delta_ang;// variation of object orientation euler angles
  Eigen::Vector3f _filt_delta_ang;
  Eigen::Vector3f _filt_delta_ang_mir;
  bool _objCtrlKey;

  //------------------------------------------------------------------------------------------------
  // object to grasp
  object_to_grasp object_;

  Vector6f _Vo;
  Vector6f _Vd_o;// desired object velocity (toss)
  Vector6f _desired_object_wrench;

  // -------------------------------
  // tossing target
  tossing_target target_;

  //-------------------------------------------------------------------------------------------------
  Eigen::Matrix4f _o_H_ee[NB_ROBOTS];
  int _objecPoseCount;
  int _initPoseCount;// Counter of received initial poses measurements

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
  float _delta_oDx;
  float _delta_oDy;
  float _delta_oDz;
  float _desVtoss;
  float _desVimp;
  float _desVreach;
  float _refVreach;
  float _friction_angle = 0.0f;
  float _max_friction_angle = 0.0f;
  float _height_via_point;

  bool _goHome;
  bool _releaseAndretract;
  bool _stop;          // Check for CTRL+C
  bool _isThrowing;    // if true execute throwing of the object
  bool _goToAttractors;// send the robots to their attractors
  bool _isPlacing;
  bool _isPickupSet;
  bool _isPlaceTossing;// fast interrupted placing motion
  bool _impact_dir_preset = true;
  int _dualTaskSelector = 1;
  bool _old_dual_method = false;

  // data logging
  std::string _DataID;
  // create data logging object
  data_logging datalog;
  ////////////////////////////////////////////
  ros::Subscriber _sub_N_objects_pose[NB_ROBOTS];// subscribe to the base pose of the robots
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

  float _delta_Imp = 0.0f;
  float _delta_Toss = 0.0f;
  float _trackingFactor;
  float _delta_tracking;

  // Vector6f _VEE_oa[NB_ROBOTS];
  std::string _dsDampingTopic[NB_ROBOTS];

  Eigen::Vector3f _delta_rel_pos;
  bool _increment_release_pos = false;
  bool _increment_lift_pos = false;
  bool _ctrl_mode_conveyor_belt = false;
  SphericalPosition release_pos;

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
  bool _pickupBased = true;
  bool _trackTargetRotation = false;
  bool _isMotionTriggered = false;
  bool _isRatioFactor = false;
  float _tol_attractor = 0.07f;
  float _switchSlopeAdapt = 100.0f;
  float _beta_vel_mod_unfilt = 1.0f;
  float _time2intercept_tgt;
  float _time2intercept_bot;

  // ------------------------------------------------------------------------
  bool _updatePathEstim = false;
  int _counter_monocycle = 0;
  int _counter_pickup = 0;
  float _dxEE_dual_avg = 0.f;
  float _dxEE_dual_avg_pcycle = 0.f;
  float _dxEE_dual_avg_0 = 0.f;
  float _Del_xEE_dual_avg = 0.f;
  Eigen::Vector3f _xEE_dual;
  Eigen::Vector3f _xEE_dual_0;
  // ------------------------------------------------------------------------

  // target
  std::deque<Eigen::Vector3f> _windowVelTarget;
  Eigen::Vector3f _movingAvgVelTarget;
  ////////////////////////////////////////////////////////////////////////
  // Objects for Unconstrained and contrained motion and force generation
  ////////////////////////////////////////////////////////////////////////
  dualArmFreeMotionController FreeMotionCtrl;  // Motion generation
  dualArmCooperativeController CooperativeCtrl;// Force generation
  throwingDS dsThrowing;                       //

  toss_task_param_estimator tossParamEstimator;// tossing task param estimator
  dualArmFreeMotionController FreeMotionCtrlEstim;
  throwingDS dsThrowingEstim;//
  bool _isSimulation;

  /////////////////////
  // ROS Callbacks
  /////////////////////
  void objectPoseCallback(const geometry_msgs::Pose::ConstPtr& msg);
  void targetPoseCallback(const geometry_msgs::Pose::ConstPtr& msg);
  void updateBasePoseCallback(const geometry_msgs::Pose::ConstPtr& msg, int k);
  void updateEEPoseCallback(const geometry_msgs::Pose::ConstPtr& msg, int k);
  void updateEETwistCallback(const geometry_msgs::Twist::ConstPtr& msg, int k);
  void updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg, int k);
  void updateRobotWrenchLeft(const geometry_msgs::WrenchStamped::ConstPtr& msg);
  void updateRobotWrenchRight(const geometry_msgs::WrenchStamped::ConstPtr& msg);
  void updateContactState();
  void updateRobotStates(const sensor_msgs::JointState::ConstPtr& msg, int k);
  void updateObjectsPoseCallback(const geometry_msgs::Pose::ConstPtr& msg, int k);

public:
  /////////////////////

  tossingTaskVariables _tossVar;

  /////////////////////
  dualArmControl(ros::NodeHandle& n,
                 double frequency,//std::string dataID,
                 std::string topic_pose_object_,
                 std::string topic_pose_robot_base[],
                 std::string topic_pose_robot_ee[],
                 std::string topic_ee_commands[],
                 std::string topic_sub_ForceTorque_Sensor[]);
  ~dualArmControl();

  bool init();
  bool initRosSubscribers();
  bool initRosPublisher();
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
  void set_2d_position_box_constraints(Eigen::Vector3f& position_vec, float limits[]);
  void mirror_target2object_orientation(Eigen::Vector4f qt, Eigen::Vector4f& qo, Eigen::Vector3f ang_lim);
  Eigen::Vector3f compute_intercept_with_target(const Eigen::Vector3f& x_pick,
                                                const Eigen::Vector3f& x_target,
                                                const Eigen::Vector3f& v_target,
                                                float phi_i);
  float get_desired_yaw_angle_target(const Eigen::Vector4f& qt, const Eigen::Vector3f& ang_lim);
  void estimate_moving_average_ee_speed();
  void estimate_moving_average_target_velocity();
  void find_desired_landing_position(Eigen::Vector3f x_origin, bool isPlacing, bool isPlaceTossing, bool isThrowing);
  void update_intercept_position(float flytime_obj, float intercep_limits[]);
  void find_release_configuration();
  void set_release_state();
  void estimate_target_state_to_go(Eigen::Vector2f Lp_Va_pred_bot, Eigen::Vector2f Lp_Va_pred_tgt, float flytime_obj);
  void compute_adaptation_factors(Eigen::Vector2f Lp_Va_pred_bot, Eigen::Vector2f Lp_Va_pred_tgt, float flytime_obj);
};
