
#include "iam_dual_arm_control/dualArmControl.h"
#include "iam_dual_arm_control/Utils.hpp"
#include "iam_dual_arm_control/keyboard.h"
#include <chrono>
#include <ctime>

using namespace std;
using namespace Eigen;

const static IOFormat CSVFormat(FullPrecision, DontAlignCols, ", ", "\n");

// ---------------------------------------------------------------------

dualArmControl::dualArmControl(ros::NodeHandle& n,
                               double frequency,
                               std::string topicPoseObject,
                               std::string topicPoseRobotBase[],
                               std::string topicPoseRobotEE[],
                               std::string topicEECommands[],
                               std::string topicFTSensorSub[]) :
    nh_(n),
    loopRate_(frequency), dt_(1.0f / frequency) {

  cycleCount_ = 0;
  gravity_ << 0.0f, 0.0f, -9.80665f;
  topicPoseObject_ = topicPoseObject;
  memcpy(topicPoseRobotBase_, &topicPoseRobotBase[0], NB_ROBOTS * sizeof *topicPoseRobotBase);
  memcpy(topicPoseRobotEE_, &topicPoseRobotEE[0], NB_ROBOTS * sizeof *topicPoseRobotEE);
  memcpy(topicEECommands_, &topicEECommands[0], NB_ROBOTS * sizeof *topicEECommands);
  memcpy(topicFTSensor_, &topicFTSensorSub[0], NB_ROBOTS * sizeof *topicFTSensorSub);

  for (int k = 0; k < NB_ROBOTS; k++) {
    oHEE_[k].setIdentity();
    d1_[k] = 1.0f;
    err_[k] = 1.0f;
    wrenchBiasOK_[k] = false;

    pubVel_[k].data.clear();
    pubVel_[k].data.push_back(0.0);// axis angle poses _x
    pubVel_[k].data.push_back(0.0);// axis angle poses _y
    pubVel_[k].data.push_back(0.0);// axis angle poses _z
    pubVel_[k].data.push_back(0.0);// linear velocity v_x
    pubVel_[k].data.push_back(0.0);// linear velocity v_y
    pubVel_[k].data.push_back(0.0);// linear velocity v_z

    basisQ_[k].setIdentity();
    dirImp_[k].setZero();
    vdImpact_[k].setZero();
  }

  objVelDes_.setZero();
  initPoseCount_ = 0;
  desiredObjectWrench_.setZero();

  isContact_ = 0.0f;

  // Velocity limits
  vMax_ = 1.50f;

  // Coordination errors
  errorObjDim_ = 0.0f;
  errorObjPos_ = 0.0f;

  // Coordination motion gains
  gainAbs_.setZero();
  gainRel_.setZero();

  // Forces
  filteredForceGain_ = 0.9f;
  sensedContact_ = false;
  forceThreshold_ = 2.0f;
  nuWr0_ = 0.0f;
  nuWr1_ = 0.0f;
  qpWrenchGeneration_ = false;

  deltaPos_.setZero();
  deltaAng_.setZero();
  filtDeltaAng_.setZero();
  filtDeltaAngMir_.setZero();
  deltaRelPos_.setZero();

  desVtoss_ = 0.5f;
  applyVelo_ = 0.0f;
  desVimp_ = 0.5f;
  desVreach_ = 0.75f;
  refVreach_ = 0.0f;

  objCtrlKey_ = true;
  goHome_ = true;
  goToAttractors_ = true;
  releaseAndretract_ = false;
  isThrowing_ = false;
  isPlacing_ = false;
  isPlaceTossing_ = false;
  releaseFlag_ = false;
  startlogging_ = false;
  isPickupSet_ = false;
  dualTaskSelector_ = 1;
  oldDualMethod_ = true;
  modeConveyorBelt_ = 0;
  nominalSpeedConveyorBelt_ = 200;
  magniturePertConveyorBelt_ = 100;
  dualPathLenAvgSpeed_.setZero();
  hasCaughtOnce_ = false;
  isIntercepting_ = false;
  isDisturbTarget_ = false;
  betaVelMod_ = 1.0f;
  initSpeedScaling_ = 1.0f;//0.75;
  trackingFactor_ = 0.40f; //0.17f; 	// 0.35f better
  winLengthAvgSpeedEE_ = 20;
  isSimulation_ = true;
  adaptationActive_ = false;
  isTargetFixed_ = true;
  userSelect_ = true;

  feasibleAlgo_ = false;
  pickupBased_ = true;
  trackTargetRotation_ = true;
  isMotionTriggered_ = false;
  isRatioFactor_ = false;
  tolAttractor_ = 0.07f;
  switchSlopeAdapt_ = 100.0f;
  betaVelModUnfilt_ = 1.0f;
  timeToInterceptTgt_ = 0.0f;
  timeToInterceptBot_ = 0.0f;

  dualAngularLimit_.setZero();
}

dualArmControl::~dualArmControl() {}

bool dualArmControl::initRosSubscribers() {

  std::string topicSubEEVel[NB_ROBOTS];
  std::string topicSubJointState[NB_ROBOTS];

  if (!nh_.getParam("vel/robot_ee/robot_left", topicSubEEVel[LEFT])) {
    ROS_ERROR("Topic vel/robot_ee/robot_left not found");
  }
  if (!nh_.getParam("vel/robot_ee/robot_right", topicSubEEVel[RIGHT])) {
    ROS_ERROR("Topic vel/robot_ee/robot_right not found");
  }
  if (!nh_.getParam("pose/joints/robot_left", topicSubJointState[LEFT])) {
    ROS_ERROR("Topic pose/joints/robot_left not found");
  }
  if (!nh_.getParam("pose/joints/robot_right", topicSubJointState[RIGHT])) {
    ROS_ERROR("Topic pose/joints/robot_right not found");
  }

  ros::Subscriber subObjectPose = nh_.subscribe(topicPoseObject_,
                                                1,
                                                &dualArmControl::objectPoseCallback,
                                                this,
                                                ros::TransportHints().reliable().tcpNoDelay());
  ros::Subscriber subTargetPose = nh_.subscribe(topic_pose_target_,
                                                1,
                                                &dualArmControl::targetPoseCallback,
                                                this,
                                                ros::TransportHints().reliable().tcpNoDelay());
  ros::Subscriber subBasePoseLeft =
      nh_.subscribe<geometry_msgs::Pose>(topicPoseRobotBase_[LEFT],
                                         1,
                                         boost::bind(&dualArmControl::updateBasePoseCallback, this, _1, LEFT),
                                         ros::VoidPtr(),
                                         ros::TransportHints().reliable().tcpNoDelay());
  ros::Subscriber subBasePoseRight =
      nh_.subscribe<geometry_msgs::Pose>(topicPoseRobotBase_[RIGHT],
                                         1,
                                         boost::bind(&dualArmControl::updateBasePoseCallback, this, _1, RIGHT),
                                         ros::VoidPtr(),
                                         ros::TransportHints().reliable().tcpNoDelay());
  ros::Subscriber subEEPoseLeft =
      nh_.subscribe<geometry_msgs::Pose>(topicPoseRobotEE_[LEFT],
                                         1,
                                         boost::bind(&dualArmControl::updateEEPoseCallback, this, _1, LEFT),
                                         ros::VoidPtr(),
                                         ros::TransportHints().reliable().tcpNoDelay());
  ros::Subscriber subEEPoseRight =
      nh_.subscribe<geometry_msgs::Pose>(topicPoseRobotEE_[RIGHT],
                                         1,
                                         boost::bind(&dualArmControl::updateEEPoseCallback, this, _1, RIGHT),
                                         ros::VoidPtr(),
                                         ros::TransportHints().reliable().tcpNoDelay());
  ros::Subscriber subEEVelLeft =
      nh_.subscribe<geometry_msgs::Twist>(topicSubEEVel[LEFT],
                                          1,
                                          boost::bind(&dualArmControl::updateEETwistCallback, this, _1, LEFT),
                                          ros::VoidPtr(),
                                          ros::TransportHints().reliable().tcpNoDelay());
  ros::Subscriber subEEVelRight =
      nh_.subscribe<geometry_msgs::Twist>(topicSubEEVel[RIGHT],
                                          1,
                                          boost::bind(&dualArmControl::updateEETwistCallback, this, _1, RIGHT),
                                          ros::VoidPtr(),
                                          ros::TransportHints().reliable().tcpNoDelay());
  ros::Subscriber subForceTorqueSensorLeft =
      nh_.subscribe<geometry_msgs::WrenchStamped>(topicFTSensor_[LEFT],
                                                  1,
                                                  boost::bind(&dualArmControl::updateRobotWrench, this, _1, LEFT),
                                                  ros::VoidPtr(),
                                                  ros::TransportHints().reliable().tcpNoDelay());
  ros::Subscriber subForceTorqueSensorRight =
      nh_.subscribe<geometry_msgs::WrenchStamped>(topicFTSensor_[RIGHT],
                                                  1,
                                                  boost::bind(&dualArmControl::updateRobotWrench, this, _1, RIGHT),
                                                  ros::VoidPtr(),
                                                  ros::TransportHints().reliable().tcpNoDelay());
  ros::Subscriber subJointStateLeft =
      nh_.subscribe<sensor_msgs::JointState>(topicSubJointState[LEFT],
                                             1,
                                             boost::bind(&dualArmControl::updateRobotStates, this, _1, LEFT),
                                             ros::VoidPtr(),
                                             ros::TransportHints().reliable().tcpNoDelay());
  ros::Subscriber subJointStateRight =
      nh_.subscribe<sensor_msgs::JointState>(topicSubJointState[RIGHT],
                                             1,
                                             boost::bind(&dualArmControl::updateRobotStates, this, _1, RIGHT),
                                             ros::VoidPtr(),
                                             ros::TransportHints().reliable().tcpNoDelay());

  return true;
}

bool dualArmControl::initRosPublisher() {
  // Commands
  pubTSCommands_[LEFT] = nh_.advertise<std_msgs::Float64MultiArray>(topicEECommands_[LEFT], 1);
  pubTSCommands_[RIGHT] = nh_.advertise<std_msgs::Float64MultiArray>(topicEECommands_[RIGHT], 1);

  // Desired orientation
  std::string topicDesiredOrientation[NB_ROBOTS];
  while (!nh_.getParam("orientation/ee_desired/robot_left", topicDesiredOrientation[LEFT])) {
    ROS_INFO("Waitinng for param: orientation/ee_desired/robot_left ");
  }
  while (!nh_.getParam("orientation/ee_desired/robot_right", topicDesiredOrientation[RIGHT])) {
    ROS_INFO("Waitinng for param: orientation/ee_desired/robot_right ");
  }
  pubDesiredOrientation_[LEFT] = nh_.advertise<geometry_msgs::Quaternion>(topicDesiredOrientation[LEFT], 1);
  pubDesiredOrientation_[RIGHT] = nh_.advertise<geometry_msgs::Quaternion>(topicDesiredOrientation[RIGHT], 1);

  // Wrench topics
  std::string topicFilteredWrench[NB_ROBOTS], topicAppliedWrench[NB_ROBOTS];
  while (!nh_.getParam("wrench/filtered/robot_left", topicFilteredWrench[LEFT])) {
    ROS_INFO("Waitinng for param: wrench/filtered/robot_left ");
  }
  while (!nh_.getParam("wrench/filtered/robot_right", topicFilteredWrench[RIGHT])) {
    ROS_INFO("Waitinng for param: wrench/filtered/robot_right ");
  }
  while (!nh_.getParam("wrench/applied/robot_left", topicAppliedWrench[LEFT])) {
    ROS_INFO("Waitinng for param: wrench/applied/robot_left ");
  }
  while (!nh_.getParam("wrench/applied/robot_right", topicAppliedWrench[RIGHT])) {
    ROS_INFO("Waitinng for param: wrench/applied/robot_right ");
  }
  pubFilteredWrench_[LEFT] = nh_.advertise<geometry_msgs::WrenchStamped>(topicFilteredWrench[LEFT], 1);
  pubFilteredWrench_[RIGHT] = nh_.advertise<geometry_msgs::WrenchStamped>(topicFilteredWrench[RIGHT], 1);
  pubAppliedWrench_[LEFT] = nh_.advertise<geometry_msgs::Wrench>(topicAppliedWrench[LEFT], 1);
  pubAppliedWrench_[RIGHT] = nh_.advertise<geometry_msgs::Wrench>(topicAppliedWrench[RIGHT], 1);

  // Forces topics
  std::string topicNormalForce[NB_ROBOTS], topicAppliedFNormMoment[NB_ROBOTS];
  while (!nh_.getParam("force/normal/robot_left", topicNormalForce[LEFT])) {
    ROS_INFO("Waitinng for param: force/normal/robot_left ");
  }
  while (!nh_.getParam("force/normal/robot_right", topicNormalForce[RIGHT])) {
    ROS_INFO("Waitinng for param: force/normal/robot_right ");
  }
  while (!nh_.getParam("force/applied_ext/robot_left", topicAppliedFNormMoment[LEFT])) {
    ROS_INFO("Waitinng for param: force/applied_ext/robot_left ");
  }
  while (!nh_.getParam("force/applied_ext/robot_right", topicAppliedFNormMoment[RIGHT])) {
    ROS_INFO("Waitinng for param: force/applied_ext/robot_right ");
  }
  pubNormalForce_[LEFT] = nh_.advertise<std_msgs::Float64>(topicNormalForce[LEFT], 1);
  pubNormalForce_[RIGHT] = nh_.advertise<std_msgs::Float64>(topicNormalForce[RIGHT], 1);
  pubAppliedFNormMoment_[LEFT] = nh_.advertise<geometry_msgs::Wrench>(topicAppliedFNormMoment[LEFT], 1);
  pubAppliedFNormMoment_[RIGHT] = nh_.advertise<geometry_msgs::Wrench>(topicAppliedFNormMoment[RIGHT], 1);

  // Desired velocities
  std::string topicDesiredVelQuat[NB_ROBOTS], topicDesiredTwist[NB_ROBOTS], topicNormLinVel[NB_ROBOTS];
  while (!nh_.getParam("veloctiy/quat_desired/robot_left", topicDesiredVelQuat[LEFT])) {
    ROS_INFO("Waitinng for param: veloctiy/quat_desired/robot_left ");
  }
  while (!nh_.getParam("veloctiy/quat_desired/robot_right", topicDesiredVelQuat[RIGHT])) {
    ROS_INFO("Waitinng for param: veloctiy/quat_desired/robot_right ");
  }
  while (!nh_.getParam("veloctiy/ee_desired/robot_left", topicDesiredTwist[LEFT])) {
    ROS_INFO("Waitinng for param: veloctiy/ee_desired/robot_left ");
  }
  while (!nh_.getParam("veloctiy/ee_desired/robot_right", topicDesiredTwist[RIGHT])) {
    ROS_INFO("Waitinng for param: veloctiy/ee_desired/robot_right ");
  }
  while (!nh_.getParam("veloctiy/linear_vel_norm/robot_left", topicNormLinVel[LEFT])) {
    ROS_INFO("Waitinng for param: veloctiy/ee_desired/robot_left ");
  }
  while (!nh_.getParam("veloctiy/linear_vel_norm/robot_right", topicNormLinVel[RIGHT])) {
    ROS_INFO("Waitinng for param: veloctiy/ee_desired/robot_right ");
  }
  pubDesiredVelQuat_[LEFT] = nh_.advertise<geometry_msgs::Pose>(topicDesiredVelQuat[LEFT], 1);
  pubDesiredVelQuat_[RIGHT] = nh_.advertise<geometry_msgs::Pose>(topicDesiredVelQuat[RIGHT], 1);
  pubDesiredTwist_[LEFT] = nh_.advertise<geometry_msgs::Twist>(topicDesiredTwist[LEFT], 1);
  pubDesiredTwist_[RIGHT] = nh_.advertise<geometry_msgs::Twist>(topicDesiredTwist[RIGHT], 1);
  pubNormLinVel_[LEFT] = nh_.advertise<std_msgs::Float64>(topicNormLinVel[LEFT], 1);
  pubNormLinVel_[RIGHT] = nh_.advertise<std_msgs::Float64>(topicNormLinVel[RIGHT], 1);

  // Attractor
  std::string topicAttractor[NB_ROBOTS], topicDistAttractorEE[NB_ROBOTS];
  while (!nh_.getParam("attractor/pos/robot_left", topicAttractor[LEFT])) {
    ROS_INFO("Waitinng for param: attractor/pos/robot_left ");
  }
  while (!nh_.getParam("attractor/pos/robot_right", topicAttractor[RIGHT])) {
    ROS_INFO("Waitinng for param: attractor/pos/robot_right ");
  }
  while (!nh_.getParam("attractor/error/robot_left", topicDistAttractorEE[LEFT])) {
    ROS_INFO("Waitinng for param: attractor/error/robot_left ");
  }
  while (!nh_.getParam("attractor/error/robot_right", topicDistAttractorEE[RIGHT])) {
    ROS_INFO("Waitinng for param: attractor/error/robot_right ");
  }
  pubAttractor_[LEFT] = nh_.advertise<geometry_msgs::Pose>(topicAttractor[LEFT], 1);
  pubAttractor_[RIGHT] = nh_.advertise<geometry_msgs::Pose>(topicAttractor[RIGHT], 1);
  pubDistAttractorEE_[LEFT] = nh_.advertise<std_msgs::Float64>(topicDistAttractorEE[LEFT], 1);
  pubDistAttractorEE_[RIGHT] = nh_.advertise<std_msgs::Float64>(topicDistAttractorEE[RIGHT], 1);

  // Conveyor Belt
  std::string topicConveyorBeltMode, topicConveyorBeltSpeed;
  while (!nh_.getParam("conveyor_belt/desired_mode", topicConveyorBeltMode)) {
    ROS_INFO("Waitinng for param: conveyor_belt/desired_mode ");
  }
  while (!nh_.getParam("conveyor_belt/desired_speed", topicConveyorBeltSpeed)) {
    ROS_INFO("Waitinng for param: conveyor_belt/desired_speed ");
  }
  pubConveyorBeltMode_ = nh_.advertise<std_msgs::Int32>(topicConveyorBeltMode, 1);
  pubConveyorBeltSpeed_ = nh_.advertise<std_msgs::Int32>(topicConveyorBeltSpeed, 1);

  return true;
}

bool dualArmControl::initRobotParam() {

  // Param vector
  std::vector<float> toolComPositionFromSensorVect[NB_ROBOTS];
  std::vector<float> xrbStandbyVect[NB_ROBOTS];
  std::vector<float> qrbStandbyVect[NB_ROBOTS];

  // Robot init param
  float toolOffsetFromEE[NB_ROBOTS];
  float paramToolMass[NB_ROBOTS];
  Eigen::Vector3f toolComPositionFromSensor[NB_ROBOTS];
  Eigen::Vector3f xrbStandby[NB_ROBOTS];
  Eigen::Vector4f qrbStandby[NB_ROBOTS];

  if (isSimulation_) {
    while (!nh_.getParam("dual_system/tool/offset2end_effector/sim/left", toolOffsetFromEE[LEFT])) {
      ROS_INFO("Waitinng for param: offset2end_effector/sim/left ");
    }
    while (!nh_.getParam("dual_system/tool/offset2end_effector/sim/right", toolOffsetFromEE[RIGHT])) {
      ROS_INFO("Waitinng for param: offset2end_effector/sim/right ");
    }
  } else {
    while (!nh_.getParam("dual_system/tool/offset2end_effector/real/left", toolOffsetFromEE[LEFT])) {
      ROS_INFO("Waitinng for param:  offset2end_effector/real/left");
    }
    while (!nh_.getParam("dual_system/tool/offset2end_effector/real/right", toolOffsetFromEE[RIGHT])) {
      ROS_INFO("Waitinng for param: offset2end_effector/real/right ");
    }
  }
  while (!nh_.getParam("dual_system/tool/mass/left", paramToolMass[0])) {
    ROS_INFO("Waitinng for param: tool mass/left ");
  }
  while (!nh_.getParam("dual_system/tool/mass/right", paramToolMass[1])) {
    ROS_INFO("Waitinng for param: tool mass/right ");
  }
  while (!nh_.getParam("dual_system/tool/com_position_from_sensor/left", toolComPositionFromSensorVect[LEFT])) {
    ROS_INFO("Waitinng for param: tool com_position_from_sensor/left ");
  }
  while (!nh_.getParam("dual_system/tool/com_position_from_sensor/right", toolComPositionFromSensorVect[RIGHT])) {
    ROS_INFO("Waitinng for param: tool com_position_from_sensor/right ");
  }
  while (!nh_.getParam("dual_arm_task/standby_pose/robot_left/position", xrbStandbyVect[LEFT])) {
    ROS_INFO("Waitinng for param: robot_left/position ");
  }
  while (!nh_.getParam("dual_arm_task/standby_pose/robot_left/orientation", qrbStandbyVect[LEFT])) {
    ROS_INFO("Waitinng for param: robot_left/orientation ");
  }
  while (!nh_.getParam("dual_arm_task/standby_pose/robot_right/position", xrbStandbyVect[RIGHT])) {
    ROS_INFO("Waitinng for param: robot_right/position ");
  }
  while (!nh_.getParam("dual_arm_task/standby_pose/robot_right/orientation", qrbStandbyVect[RIGHT])) {
    ROS_INFO("Waitinng for param: robot_right/orientation ");
  }

  toolComPositionFromSensor[LEFT] =
      Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(toolComPositionFromSensorVect[LEFT].data(),
                                                    toolComPositionFromSensorVect[LEFT].size());
  toolComPositionFromSensor[RIGHT] =
      Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(toolComPositionFromSensorVect[RIGHT].data(),
                                                    toolComPositionFromSensorVect[RIGHT].size());

  xrbStandby[LEFT] =
      Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(xrbStandbyVect[LEFT].data(), xrbStandbyVect[LEFT].size());
  qrbStandby[LEFT] =
      Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(qrbStandbyVect[LEFT].data(), qrbStandbyVect[LEFT].size());
  xrbStandby[RIGHT] =
      Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(xrbStandbyVect[RIGHT].data(), xrbStandbyVect[RIGHT].size());
  qrbStandby[RIGHT] =
      Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(qrbStandbyVect[RIGHT].data(), qrbStandbyVect[RIGHT].size());

  int sgf_dq[3];
  sgf_dq[0] = 7;
  sgf_dq[1] = 3;
  sgf_dq[2] = 6;

  robot_.init_robot(sgf_dq, dt_, gravity_);

  robot_.set_init_parameters(paramToolMass, toolOffsetFromEE, toolComPositionFromSensor, xrbStandby, qrbStandby);

  return true;
}

bool dualArmControl::initObjectParam() {
  // Object desired grasping points
  Eigen::Matrix3f oRGraspPosLeft;
  Eigen::Matrix3f oRGraspPosRight;
  oRGraspPosLeft.setZero();
  oRGraspPosRight.setZero();
  oRGraspPosLeft(0, 0) = 1.0f;
  oRGraspPosLeft(2, 1) = -1.0f;
  oRGraspPosLeft(1, 2) = 1.0f;
  oRGraspPosRight(0, 0) = 1.0f;
  oRGraspPosRight(2, 1) = 1.0f;
  oRGraspPosRight(1, 2) = -1.0f;

  int sgfP[3];
  int sgfO[3];
  sgfP[0] = 3;
  sgfP[1] = 3;
  sgfP[2] = 6;
  sgfO[0] = 4;
  sgfO[1] = 3;
  sgfO[2] = 10;

  // Object get param
  std::string objectName;
  float objectMassVect;
  std::vector<float> graspOffsetLeftVect;
  std::vector<float> graspOffsetRightVect;
  std::vector<float> objectDimVect;
  while (!nh_.getParam("object/name", objectName)) { ROS_INFO("Waitinng for param: object/name"); }
  while (!nh_.getParam("object/" + objectName + "/mass", objectMassVect)) {
    ROS_INFO("Waitinng for param: object mass ");
  }
  while (!nh_.getParam("object/" + objectName + "/dimension", objectDimVect)) {
    ROS_INFO("Waitinng for param: object dimension ");
  }
  while (!nh_.getParam("object/" + objectName + "/graspOffset_L", graspOffsetLeftVect)) {
    ROS_INFO("Waitinng for param: object/graspOffset_L ");
  }
  while (!nh_.getParam("object/" + objectName + "/graspOffset_R", graspOffsetRightVect)) {
    ROS_INFO("Waitinng for param: object/graspOffset_R ");
  }

  Eigen::Vector3f graspOffsetLeft =
      Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(graspOffsetLeftVect.data(), graspOffsetLeftVect.size());
  Eigen::Vector3f graspOffsetRight =
      Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(graspOffsetRightVect.data(), graspOffsetRightVect.size());

  object_.init_object(sgfP, sgfO, dt_, oRGraspPosLeft, oRGraspPosRight);
  object_._objectMass = objectMassVect;
  object_._objectDim = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(objectDimVect.data(), objectDimVect.size());

  // Relative grasping positons
  object_._xgp_o[0] = Eigen::Vector3f(0.0f, -object_._objectDim(1) / 2.0f, 0.0f) + graspOffsetLeft;
  object_._xgp_o[1] = Eigen::Vector3f(0.0f, object_._objectDim(1) / 2.0f, 0.0f) + graspOffsetRight;

  return true;
}

bool dualArmControl::initFreeMotionCtrl() {
  bool modulated_reaching = true;
  bool isNorm_impact_vel = false;
  while (!nh_.getParam("dual_arm_task/reach_to_grasp/desVreach", desVreach_)) {
    ROS_INFO("Waitinng for param: reach_to_grasp/desVreach");
  }
  while (!nh_.getParam("dual_arm_task/modulated_reaching", modulated_reaching)) {
    ROS_INFO("Waitinng for param:  modulated_reaching");
  }
  while (!nh_.getParam("dual_arm_task/isNorm_impact_vel", isNorm_impact_vel)) {
    ROS_INFO("Waitinng for param:  isNorm_impact_vel");
  }
  while (!nh_.getParam("dual_arm_task/placing/height_via_point", heightViaPoint_)) {
    ROS_INFO("Waitinng for param: placing/height_via_point");
  }
  freeMotionCtrl_.init(robot_._w_H_eeStandby, this->gainAbs_, this->gainRel_);
  freeMotionCtrl_._dt = dt_;
  freeMotionCtrl_._objectDim = object_._objectDim;

  freeMotionCtrl_._desVreach = desVreach_;
  freeMotionCtrl_._refVreach[LEFT] = refVreach_;
  freeMotionCtrl_._refVreach[RIGHT] = refVreach_;
  freeMotionCtrl_._modulated_reaching = modulated_reaching;
  freeMotionCtrl_._isNorm_impact_vel = isNorm_impact_vel;
  freeMotionCtrl_._height_via_point = heightViaPoint_;

  return true;
}

bool dualArmControl::initTossVar() {
  std::vector<float> releasePosVect;
  std::vector<float> releaseOrientVect;
  std::vector<float> releaseLinVelDirVect;
  std::vector<float> releaseAngVelVect;
  std::vector<float> restOrientVect;
  std::vector<float> restPosVect;
  while (!nh_.getParam("dual_arm_task/tossing/desVtoss", desVtoss_)) {
    ROS_INFO("Waitinng for param: tossing/desVtoss ");
  }
  while (!nh_.getParam("dual_arm_task/tossing/releasePos", releasePosVect)) {
    ROS_INFO("Waitinng for param: tossing/releasePos ");
  }
  while (!nh_.getParam("dual_arm_task/tossing/releaseOrient", releaseOrientVect)) {
    ROS_INFO("Waitinng for param: tossing/releaseOrien ");
  }
  while (!nh_.getParam("dual_arm_task/tossing/releaseLinVel_dir", releaseLinVelDirVect)) {
    ROS_INFO("Waitinng for param: tossing/releaseLinVel_dir ");
  }
  while (!nh_.getParam("dual_arm_task/tossing/releaseAngVel", releaseAngVelVect)) {
    ROS_INFO("Waitinng for param: tossing/releaseAngVel ");
  }
  while (!nh_.getParam("dual_arm_task/tossing/restPos", restPosVect)) {
    ROS_INFO("Waitinng for param: tossing/restPos ");
  }

  while (!nh_.getParam("dual_arm_task/tossing/restOrient", restOrientVect)) {
    ROS_INFO("Waitinng for param: tossing/restOrient ");
  }
  tossVar_.release_position =
      Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(releasePosVect.data(), releasePosVect.size());
  tossVar_.release_orientation =
      Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(releaseOrientVect.data(), releaseOrientVect.size());
  tossVar_.release_linear_velocity =
      Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(releaseLinVelDirVect.data(), releaseLinVelDirVect.size());
  tossVar_.release_angular_velocity =
      Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(releaseAngVelVect.data(), releaseAngVelVect.size());
  tossVar_.rest_position = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(restPosVect.data(), restPosVect.size());
  tossVar_.rest_orientation =
      Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(restOrientVect.data(), restOrientVect.size());

  tossVar_.release_linear_velocity = desVtoss_ * tossVar_.release_linear_velocity.normalized();

  return true;
}

bool dualArmControl::initDesTasksPosAndLimits() {
  std::vector<float> param_abs_gains;
  std::vector<float> param_rel_gains;

  std::vector<float> param_xDo_lifting;
  std::vector<float> param_qDo_lifting;
  std::vector<float> param_xDo_placing;
  std::vector<float> param_qDo_placing;

  std::vector<float> param_dual_angular_limit;

  while (!nh_.getParam("dual_arm_task/coordination/ds_absolute_gains", param_abs_gains)) {
    ROS_INFO("Waitinng for param: ds_absolute_gains ");
  }
  while (!nh_.getParam("dual_arm_task/coordination/ds_relative_gains", param_rel_gains)) {
    ROS_INFO("Waitinng for param: ds_relative_gains ");
  }
  while (!nh_.getParam("dual_arm_task/lifting/position", param_xDo_lifting)) {
    ROS_INFO("Waitinng for param: lifting/position");
  }
  while (!nh_.getParam("dual_arm_task/lifting/orientation", param_qDo_lifting)) {
    ROS_INFO("Waitinng for param: lifting/orientation");
  }
  while (!nh_.getParam("dual_arm_task/placing/position", param_xDo_placing)) {
    ROS_INFO("Waitinng for param: placing/position");
  }
  while (!nh_.getParam("dual_arm_task/placing/orientation", param_qDo_placing)) {
    ROS_INFO("Waitinng for param: placing/orientation");
  }
  while (!nh_.getParam("dual_arm_task/tossing/dual_angular_limit", param_dual_angular_limit)) {
    ROS_INFO("Waitinng for param: tossing/param_dual_angular_limit ");
  }

  gainAbs_.diagonal() = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(param_abs_gains.data(), param_abs_gains.size());
  gainRel_.diagonal() = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(param_rel_gains.data(), param_rel_gains.size());
  xLifting_ = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(param_xDo_lifting.data(), param_xDo_lifting.size());
  qLifting_ = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(param_qDo_lifting.data(), param_qDo_lifting.size());
  xPlacing_ = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(param_xDo_placing.data(), param_xDo_placing.size());
  qPlacing_ = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(param_qDo_placing.data(), param_qDo_placing.size());
  dualAngularLimit_ =
      Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(param_dual_angular_limit.data(), param_dual_angular_limit.size());

  Eigen::Matrix3f RDo_lifting = Utils<float>::quaternionToRotationMatrix(qLifting_);
  filtDeltaAng_ = Utils<float>::getEulerAnglesXYZ_FixedFrame(RDo_lifting);
  filtDeltaAngMir_ = Utils<float>::getEulerAnglesXYZ_FixedFrame(RDo_lifting);

  return true;
}

bool dualArmControl::initDampingTopicCtrl() {

  std::string param_damping_topic_CustomCtrl_left = "/iiwa1/CustomControllers/controllers/PassiveDS/params";
  std::string param_damping_topic_CustomCtrl_right = "/iiwa_blue/CustomControllers/controllers/PassiveDS/params";
  std::string param_damping_topic_TorqueCtrl_left = "/iiwa1/control/lambda_Pos";
  std::string param_damping_topic_TorqueCtrl_right = "/iiwa_blue/control/lambda_Pos";

  while (
      !nh_.getParam("dual_system/passiveDS/dampingTopic/CustomController/left", param_damping_topic_CustomCtrl_left)) {
    ROS_INFO("Waitinng for param : CustomController/left");
  }
  while (!nh_.getParam("dual_system/passiveDS/dampingTopic/CustomController/right",
                       param_damping_topic_CustomCtrl_right)) {
    ROS_INFO("Waitinng for param : CustomController/right");
  }
  while (
      !nh_.getParam("dual_system/passiveDS/dampingTopic/TorqueController/left", param_damping_topic_TorqueCtrl_left)) {
    ROS_INFO("Waitinng for param : TorqueController/left");
  }
  while (!nh_.getParam("dual_system/passiveDS/dampingTopic/TorqueController/right",
                       param_damping_topic_TorqueCtrl_right)) {
    ROS_INFO("Waitinng for param : TorqueController/right");
  }

  std::vector<float> param_damp_l;
  std::vector<float> param_damp_r;

  ros::param::getCached(param_damping_topic_TorqueCtrl_left, param_damp_l);
  ros::param::getCached(param_damping_topic_TorqueCtrl_right, param_damp_r);

  if ((!param_damp_l.empty()) && (!param_damp_r.empty())) {
    dsDampingTopic_[LEFT] = param_damping_topic_TorqueCtrl_left;
    dsDampingTopic_[RIGHT] = param_damping_topic_TorqueCtrl_right;
  } else {
    dsDampingTopic_[LEFT] = param_damping_topic_CustomCtrl_left;
    dsDampingTopic_[RIGHT] = param_damping_topic_CustomCtrl_right;
  }

  return true;
}

bool dualArmControl::initConveyorBelt() {
  while (!nh_.getParam("conveyor_belt/control_mode", ctrlModeConveyorBelt_)) {
    ROS_INFO("Waitinng for param: conveyor_belt/control_mode ");
  }
  while (!nh_.getParam("conveyor_belt/nominal_speed", nominalSpeedConveyorBelt_)) {
    ROS_INFO("Waitinng for param: conveyor_belt/nominal_speed");
  }
  while (!nh_.getParam("conveyor_belt/magnitude_perturbation", magniturePertConveyorBelt_)) {
    ROS_INFO("Waitinng for param: conveyor_belt/magnitude_perturbation");
  }

  desSpeedConveyorBelt_ = nominalSpeedConveyorBelt_;

  return true;
}

bool dualArmControl::initUserInteraction() {
  while (!nh_.getParam("dual_arm_task/isQP_wrench_generation", qpWrenchGeneration_)) {
    ROS_INFO("Waitinng for param:  isQP_wrench_generation");
  }
  while (!nh_.getParam("dual_arm_task/objCtrlKey", objCtrlKey_)) { ROS_INFO("Waitinng for param:  objCtrlKey"); }
  while (!nh_.getParam("dual_arm_task/isTargetFixed", isTargetFixed_)) {
    ROS_INFO("Waitinng for param:  isTargetFixed");
  }
  while (!nh_.getParam("dual_arm_task/userSelect", userSelect_)) { ROS_INFO("Waitinng for param:  userSelect"); }

  return true;
}

bool dualArmControl::initTossParamEstimator() {
  std::string path2LearnedModelfolder = ros::package::getPath(std::string("dual_arm_control")) + "/LearnedModel/model1";
  std::string file_gmm[3];
  std::string dataType = "/throwingParam";

  file_gmm[0] = path2LearnedModelfolder + dataType + "_prio.txt";
  file_gmm[1] = path2LearnedModelfolder + dataType + "_mu.txt";
  file_gmm[2] = path2LearnedModelfolder + dataType + "_sigma.txt";

  tossParamEstimator_.init(file_gmm,
                           tossVar_.release_position,
                           tossVar_.release_orientation,
                           tossVar_.release_linear_velocity,
                           tossVar_.release_angular_velocity);

  target_._xd_landing = Eigen::Vector3f(1.0f, 0.0f, 0.0f);

  tossParamEstimator_.estimate_tossing_param(toss_task_param_estimator::PHYS_IDEAL,
                                             target_._xd_landing,
                                             tossVar_.release_position);

  return true;
}

bool dualArmControl::initDSThrowing() {

  dsThrowing_.init(dsThrowing_.ds_param_,
                   tossVar_.release_position,
                   tossVar_.release_orientation,
                   tossVar_.release_linear_velocity,
                   tossVar_.release_angular_velocity,
                   tossVar_.rest_position,
                   tossVar_.rest_orientation);
  // TODO  if statement?
  // IF AUTOMATICALLY DETERMINED (USING RELEASE POSE GENERATOR)
  // dsThrowing_.init(dsThrowing_.ds_param_,
  // 								tossParamEstimator_.get_release_position(),
  // 								tossParamEstimator_.get_release_orientation(),
  // 								tossParamEstimator_.get_release_linear_velocity(),
  // 								tossParamEstimator_.get_release_angular_velocity(),
  // 								tossVar_.rest_position, tossVar_.rest_orientation);
  // desVtoss_ = tossParamEstimator_.get_release_linear_velocity().norm();
  //
  tossVar_.release_linear_velocity = desVtoss_ * (tossVar_.release_position - object_._xo).normalized();

  dsThrowing_.set_pickup_object_pose(object_._xo, object_._qo);
  dsThrowing_.set_toss_linear_velocity(tossVar_.release_linear_velocity);
  dsThrowing_._refVtoss = desVimp_;

  return true;
}

bool dualArmControl::init() {

  topic_pose_target_ = "/simo_track/target_pose";

  while (!nh_.getParam("dual_system/simulation", isSimulation_)) {
    ROS_INFO("Waitinng for param: dual_system/simulation ");
  }

  while (!nh_.getParam("dual_arm_task/reach_to_grasp/impact/desVimp", desVimp_)) {
    ROS_INFO("Waitinng for param: impact/desVimp");
  }
  while (!nh_.getParam("dual_arm_task/reach_to_grasp/impact/impact_direction/friction_angle", frictionAngle_)) {
    ROS_INFO("Waitinng for param: impact_direction/friction_angl");
  }
  while (!nh_.getParam("dual_arm_task/reach_to_grasp/impact/impact_direction/max_friction_angle", frictionAngleMax_)) {
    ROS_INFO("Waitinng for param: impact_direction/max_friction_angle");
  }
  while (!nh_.getParam("dual_arm_task/reach_to_grasp/impact/impact_direction/impact_dir_preset", impactDirPreset_)) {
    ROS_INFO("Waitinng for param: impact_direction/impact_dir_preset");
  }

  while (!nh_.getParam("dual_arm_task/tossing/increment_release_pos", incrementReleasePos_)) {
    ROS_INFO("Waitinng for param: tossing/increment_release_pos ");
  }
  while (!nh_.getParam("dual_arm_task/dualTaskSelector", dualTaskSelector_)) {
    ROS_INFO("Waitinng for param: dualTaskSelector");
  }
  while (!nh_.getParam("dual_arm_task/old_dual_method", oldDualMethod_)) {
    ROS_INFO("Waitinng for param:  old_dual_method");
  }

  // Get params and init
  initConveyorBelt();
  initUserInteraction();

  initDampingTopicCtrl();
  initTossVar();
  initDesTasksPosAndLimits();

  initRosSubscribers();
  initRosPublisher();

  initRobotParam();
  initObjectParam();

  target_.init_target(3, 3, 10, dt_);

  //-----------------------------------
  // Motion and Force generation: DS
  //-----------------------------------

  // Initialize the Free motion generator DS
  initFreeMotionCtrl();

  // Initialize the cooperative controller
  CooperativeCtrl.init();

  // Initialization of the toss task parameter estimator
  initTossParamEstimator();

  // Object tossing DS
  target_._xd_landing = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
  target_._x_intercept = target_._xd_landing;
  object_._x_pickup = object_._xo;

  // Initialize throwing object
  initDSThrowing();
  dsThrowingEstim_ = dsThrowing_;
  freeMotionCtrlEstim_ = freeMotionCtrl_;

  // Data recording:
  dataLog_.datalog_init(ros::package::getPath(std::string("dualArmControl")) + "/Data");

  if (nh_.ok()) {
    // Wait for poses being published
    ros::spinOnce();
    ROS_INFO("[dualArmControl]: The object grabbing node is ready.");
    return true;
  } else {
    ROS_ERROR("[dualArmControl]: The ros node has a problem.");
    return false;
  }
  return true;
}

// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Run
// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void dualArmControl::run() {
  ROS_INFO("Running the dualArmControl");

  // --------------------------------------------------------
  while (nh_.ok()) {
    //
    auto start = std::chrono::high_resolution_clock::now();
    // update
    update_states_machines();
    // get the first eigen value of the passive ds controller and its updated value
    get_pasive_ds_1st_damping();

    mutex.lock();
    // update the poses of the robots and the object
    updatePoses();
    // compute generated desired motion and forces
    computeCommands();
    // publish the commands to be exectued
    publish_commands();
    // publish data through topics for analysis
    publishData();
    //
    if (startlogging_) { saveData(); }
    //
    mutex.unlock();

    ros::spinOnce();
    loopRate_.sleep();
    cycleCount_++;// counter the cycles

    // Estimation of the running period
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << " [dualArmControl] RUNNING PERIOD ------------> : \t " << duration.count() << " ms" << std::endl;
  }
  // Send zero command
  for (int k = 0; k < NB_ROBOTS; k++) {
    robot_._vd[k].setZero();
    robot_._omegad[k].setZero();
    robot_._qd[k] = robot_._q[k];
  }
  publish_commands();
  //
  ros::spinOnce();
  loopRate_.sleep();
  // close the data logging files
  dataLog_.Close_files();
  //
  ros::shutdown();
}
// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void dualArmControl::update_states_machines() {
  // ------------------------------------------
  keyboard::Keyboard::nonblock_2(1);
  if (keyboard::Keyboard::khbit_2() != 0) {
    char c = fgetc(stdin);
    fflush(stdin);

    switch (c) {
      case 'q': {
        goHome_ = !goHome_;
        if (goHome_) {
          goToAttractors_ = true;
          startlogging_ = false;
          // dataLog_.datalog_reset(ros::package::getPath(std::string("dual_arm_control")) +"/Data");
        }
        if (!goHome_) { startlogging_ = true; }
      } break;
      // case 'g': goToAttractors_ = !goToAttractors_; break;
      case 'g': {
        goToAttractors_ = !goToAttractors_;
        if (goToAttractors_) {
          goHome_ = false;
          releaseAndretract_ = false;
        }
      } break;
      // control of the object (desired using keyboard)
      // position
      // case 'a': deltaPos_(0) -= 0.01f; break;
      // case 's': deltaPos_(0) += 0.01f; break;
      // case 'd': deltaPos_(1) -= 0.01f; break;
      // case 'f': deltaPos_(1) += 0.01f; break;
      // case 'z': deltaPos_(2) -= 0.01f; break;
      // case 'w': deltaPos_(2) += 0.01f; break;
      // position
      case 'a':
        if (ctrlModeConveyorBelt_) {
          modeConveyorBelt_ = 2;
          publish_conveyor_belt_cmds();
          startlogging_ = true;
        } else if (incrementReleasePos_)
          deltaRelPos_(0) -= 0.025f;//deltaRelPos_(0)  -= 0.05f;  //[m]
        else
          deltaPos_(0) -= 0.01f;
        break;
      case 's':
        if (ctrlModeConveyorBelt_) {
          modeConveyorBelt_ = 0;
          publish_conveyor_belt_cmds();
        } else if (incrementReleasePos_)
          deltaRelPos_(0) += 0.025f;//deltaRelPos_(0)  += 0.05f; //[m]
        else
          deltaPos_(0) += 0.01f;
        break;
      case 'd':
        if (ctrlModeConveyorBelt_) {
          modeConveyorBelt_ = 1;
          publish_conveyor_belt_cmds();
        } else if (incrementReleasePos_)
          deltaRelPos_(1) -= 5.0f;//[deg]   deltaRelPos_(1)  -= 0.025f; //
        else
          deltaPos_(1) -= 0.01f;
        break;
      case 'f':
        if (incrementReleasePos_) deltaRelPos_(1) += 5.0f;//[deg]   deltaRelPos_(1)  += 0.025f; //
        else
          deltaPos_(1) += 0.01f;
        break;
      case 'z':
        if (ctrlModeConveyorBelt_) trackingFactor_ -= 0.01f;
        else if (incrementReleasePos_)
          deltaRelPos_(2) -= 5.0f;//[deg]   deltaRelPos_(2)  -= 0.025f; //
        else
          deltaPos_(2) -= 0.01f;
        break;
      case 'w':
        if (ctrlModeConveyorBelt_) trackingFactor_ += 0.01f;
        else if (incrementReleasePos_)
          deltaRelPos_(2) += 5.0f;//[deg]   deltaRelPos_(2)  += 0.025f; //
        else
          deltaPos_(2) += 0.01f;
        break;
      //orientation
      case 'h':
        if (ctrlModeConveyorBelt_) nominalSpeedConveyorBelt_ -= 50;
        else
          deltaAng_(0) -= 0.05f;
        break;
      case 'j':
        if (ctrlModeConveyorBelt_) nominalSpeedConveyorBelt_ += 50;
        else
          deltaAng_(0) += 0.05f;
        break;
      case 'k':
        if (ctrlModeConveyorBelt_) adaptationActive_ = !adaptationActive_;
        else
          deltaAng_(1) -= 0.05f;
        break;
      // case 'l': deltaAng_(1) += 0.05f; break;
      case 'm':
        if (ctrlModeConveyorBelt_) magniturePertConveyorBelt_ -= 50;
        else
          deltaAng_(2) -= 0.05f;
        break;
      case 'i':
        if (ctrlModeConveyorBelt_) magniturePertConveyorBelt_ += 50;
        else
          deltaAng_(2) += 0.05f;
        break;

      // user control of release or throwing (keyboard)
      case 'r':
        releaseAndretract_ = !releaseAndretract_;
        break;

      case 'l': {
        dualTaskSelector_ = PICK_AND_LIFT;
        hasCaughtOnce_ = false;
      } break;

      case 't': {
        isThrowing_ = !isThrowing_;
        if (isThrowing_) {
          dualTaskSelector_ = PICK_AND_TOSS;//TOSSING; // toss
          hasCaughtOnce_ = false;
        } else if (!isThrowing_) {
          dualTaskSelector_ = PICK_AND_LIFT;// toss
        }
      } break;
      case 'p': {
        isPlacing_ = !isPlacing_;
        if (isPlacing_) {
          dualTaskSelector_ = PICK_AND_PLACE;
          hasCaughtOnce_ = false;
        } else if (!isPlacing_) {
          dualTaskSelector_ = PICK_AND_LIFT;
        }
      } break;
      case 'o': {
        isPlaceTossing_ = !isPlaceTossing_;
        if (isPlaceTossing_) {
          dualTaskSelector_ = PLACE_TOSSING;
          hasCaughtOnce_ = false;
        } else if (!isPlaceTossing_) {
          dualTaskSelector_ = PICK_AND_LIFT;
        }
      } break;
      // control of impact and tossing velocity
      case 'v':
        desVtoss_ -= 0.05f;
        if (desVtoss_ < 0.2f) desVtoss_ = 0.2f;
        dsThrowing_.set_toss_linear_velocity(desVtoss_ * tossVar_.release_linear_velocity.normalized());
        dsThrowingEstim_.set_toss_linear_velocity(desVtoss_ * tossVar_.release_linear_velocity.normalized());
        break;
      case 'b':
        desVtoss_ += 0.05f;
        if (desVtoss_ > 2.0f) desVtoss_ = 2.0f;
        dsThrowing_.set_toss_linear_velocity(desVtoss_ * tossVar_.release_linear_velocity.normalized());
        dsThrowingEstim_.set_toss_linear_velocity(desVtoss_ * tossVar_.release_linear_velocity.normalized());
        break;
      case 'y':
        desVimp_ -= 0.05f;
        if (desVimp_ < 0.05f) desVimp_ = 0.05f;
        break;
      case 'u':
        desVimp_ += 0.05f;
        if (desVimp_ > 0.6f) desVimp_ = 0.6f;
        break;

      // reset the data logging
      case 'c':
        startlogging_ = false;
        dataLog_.datalog_reset(ros::package::getPath(std::string("dual_arm_control")) + "/Data");
        break;

      // disturb the target speed
      case 'e':
        isDisturbTarget_ = !isDisturbTarget_;
        break;

      // placing hight
      case 'x':
        if (dualTaskSelector_ == PICK_AND_TOSS) {
          tossVar_.release_position(1) -= 0.01;
        } else {
          xPlacing_(2) -= 0.01;
        }
        break;

      // placing hight
      case 'n':
        if (dualTaskSelector_ == PICK_AND_TOSS) {
          tossVar_.release_position(1) += 0.01;
        } else {
          xPlacing_(2) += 0.01;
        }
        break;
    }
  }
  keyboard::Keyboard::nonblock_2(0);
  // ----------------------------------------------------
}

void dualArmControl::updatePoses() {
  if (initPoseCount_ < 100) {
    // get stanby transformation of the EEs wrt. the world frame
    robot_.get_StandbyHmgTransformInWorld();
    freeMotionCtrl_._w_H_eeStandby[LEFT] = robot_._w_H_eeStandby[LEFT];
    freeMotionCtrl_._w_H_eeStandby[RIGHT] = robot_._w_H_eeStandby[RIGHT];
    //
    object_._xDo = Eigen::Vector3f(object_._xo(0), object_._xo(1), xLifting_(2));// set attractor of lifting task
    object_._qDo = object_._qo;                                                  // qLifting_
    object_.get_desiredHmgTransform();

    target_._x_intercept = Eigen::Vector3f(object_._xo(0), 0.0, object_._xo(2));
    // for catching
    freeMotionCtrl_.set_virtual_object_frame(Utils<float>::pose2HomoMx(target_._x_intercept, object_._qo));

    initPoseCount_++;
  }

  // Estimation of moving average EE and target speed
  // this->estimate_moving_average_ee_speed();
  // this->estimate_moving_average_target_velocity();

  // Update the object position or its desired position (attractor) through keyboard
  if (objCtrlKey_) {
    this->Keyboard_virtual_object_control();
  } else {
    this->Keyboard_reference_object_control();
  }
  //
  if (incrementReleasePos_) { this->update_release_position(); }

  // homogeneous transformations associated with the reaching task
  robot_.get_EndEffectorHmgTransform();
  object_.get_grasp_point_HTransform();
  object_.get_grasp_point_desiredHTransform();
  object_.update_grasp_normals();

  for (int k = 0; k < NB_ROBOTS; k++) {
    err_[k] = (robot_._w_H_ee[k].block(0, 3, 3, 1) - object_._w_H_gp[k].block(0, 3, 3, 1)).norm();
    oHEE_[k] = object_._w_H_o.inverse() * robot_._w_H_ee[k];

    if (CooperativeCtrl._ContactConfidence == 1.0) {
      oHEE_[k](1, 3) *= 0.95f;
      object_._w_H_Dgp[k] = object_._w_H_Do * oHEE_[k];
      // _w_H_gp[k]  = _w_H_o * oHEE_[k];
    }
  }
}

//
void dualArmControl::get_pasive_ds_1st_damping() {
  std::vector<float> param_values;
  ros::param::getCached(dsDampingTopic_[LEFT], param_values);
  d1_[LEFT] = param_values[0];
  if (d1_[LEFT] < FLT_EPSILON) d1_[LEFT] = 150.0f;
  ros::param::getCached(dsDampingTopic_[RIGHT], param_values);
  d1_[RIGHT] = param_values[0];
  if (d1_[RIGHT] < FLT_EPSILON) d1_[RIGHT] = 150.0f;
}

// // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void dualArmControl::computeCommands() {
  // Update contact state
  updateContactState();
  // --------------------------------------------------------------------------------------------------------------------------------------
  // self imposed limits on intercept region (placing on moving target)
  float x_t_min = 0.60f;
  float x_t_max = 0.75f;
  float y_t_min = -0.10f;
  float y_t_max = 0.10f;
  float beta_vel_mod_unfilt = 1.0f;
  // ---------------------------------------------------------------------------------------------------------------------------------------
  bool no_dual_mds_method = oldDualMethod_;
  bool isContact = true && sensedContact_ && CooperativeCtrl._ContactConfidence == 1.0f;
  bool isPlacing = isPlacing_ || (dualTaskSelector_ == PICK_AND_PLACE);
  bool isThrowing = isThrowing_ || (dualTaskSelector_ == TOSSING) || (dualTaskSelector_ == PICK_AND_TOSS);
  bool isPlaceTossing = isPlaceTossing_ || (dualTaskSelector_ == PLACE_TOSSING);
  bool isClose2Release = (dsThrowing_.a_tangent_ > 0.99f);
  //
  bool isPlacingCommand = (releaseFlag_) || ((object_._w_H_o.block<3, 1>(0, 3) - xPlacing_).norm() <= 0.07);// 0.07 0.05
  bool isTossingCommand =
      (releaseFlag_) || ((object_._w_H_o.block<3, 1>(0, 3) - tossVar_.release_position).norm() <= 0.035);
  //
  bool placing_done = (releaseFlag_) || ((object_._w_H_o.block<3, 1>(0, 3) - xPlacing_).norm() <= 0.08);//0.05
  bool placeTossing_done = (releaseFlag_)
      || (((object_._w_H_o.block<3, 1>(0, 3) - tossVar_.release_position).norm() <= 0.07)
          || ((object_._w_H_o.block<2, 1>(0, 3) - xPlacing_.head(2)).norm() <= 0.05));
  bool tossing_done =
      (releaseFlag_) || (((object_._w_H_o.block<3, 1>(0, 3) - tossVar_.release_position).norm() <= 0.035));
  bool isForceDetected =
      (robot_._normalForceAverage[LEFT] > forceThreshold_ || robot_._normalForceAverage[RIGHT] > forceThreshold_);
  // ----------------------------------------------------------------------------------------------------------------------------------------
  // Intercept/ landing location
  // ----------------------------------------------------------------------------------------------------------------------------------------
  // limits for throwing object's yaw angle
  // compute intercept position limits
  Eigen::Vector3f x_origin = object_._x_pickup;
  x_origin = Eigen::Vector3f(0.35, 0, 0);
  Eigen::Vector3f x_i_min =
      this->compute_intercept_with_target(x_origin, target_._xt, target_._vt, -dualAngularLimit_(2));
  Eigen::Vector3f x_i_max =
      this->compute_intercept_with_target(x_origin, target_._xt, target_._vt, dualAngularLimit_(2));

  // self imposed limits on intercept region (placing on moving target)
  float intercep_limits[4];
  intercep_limits[0] = 0.60f;     // x_min
  intercep_limits[1] = 0.75f;     // x_max
  intercep_limits[2] = x_i_min(1);// -0.10f;  // y_min
  intercep_limits[3] = x_i_max(1);//  0.10f;  // y_max

  // // // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // 		// // ---------------------------------------------------------------------------------------------------------------------------------
  // 		// // calibration to estimate the tracking Factor
  // 		// // ------------------------------------------------------------------------------------------reset_variables---------------------------------------
  // 		// if(!isPickupSet_ && !releaseAndretract_){
  // 		// 	// dxEE_dual_ = 0.5f*(VEE[LEFT] + VEE[RIGHT]);
  // 		// 	// _dxEE_dual_avg 		 += (0.5f*(_Vee[LEFT].head(3) + _Vee[RIGHT].head(3)).norm() - _dxEE_dual_avg_0)/(_counter_monocycle + 1);
  // 		// 	_dxEE_dual_avg 		 += (Utils<float>::get_abs_3d(_Vee, true).norm() - _dxEE_dual_avg_0)/(_counter_monocycle + 1);
  // 		// 	_xEE_dual 					= Utils<float>::get_abs_3d(_w_H_ee); //0.5f*(robot_._w_H_ee[LEFT].block(0,3,3,1) + robot_._w_H_ee[RIGHT].block(0,3,3,1));
  // 		// 	_Del_xEE_dual_avg  += (_xEE_dual- _xEE_dual_0).norm();
  // 		// 	//
  // 		// 	_dxEE_dual_avg_0 = _dxEE_dual_avg;
  // 		// 	_xEE_dual_0 		 = _xEE_dual;

  // 		// 	_updatePathEstim = false;
  // 		// 	_counter_monocycle ++;
  // 		// }
  // 		// // //
  // 		// // float new_trackingFactor = 1.0f;
  // 		// // if(isPickupSet_ && !_updatePathEstim && releaseAndretract_){
  // 		// // 	//
  // 		// // 	_dxEE_dual_avg_pcycle += (_dxEE_dual_avg - _dxEE_dual_avg_pcycle)/(_counter_pickup + 1);
  // 		// // 	_dxEE_dual_avg_pcycle  = _dxEE_dual_avg;
  // 		// // 	//
  // 		// // 	new_trackingFactor = min((_dxEE_dual_avg_pcycle/(dualPathLenAvgSpeed_(1)+1e-5)), 1.0);
  // 		// // 	trackingFactor_ 	 = trackingFactor_ + (new_trackingFactor - trackingFactor_)/(_counter_pickup + 1);
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
  if ((!releaseAndretract_) && (fmod(cycleCount_, 20) == 0)) {
    dualPathLenAvgSpeed_ = freeMotionCtrlEstim_.predictRobotTranslation(robot_._w_H_ee,
                                                                        object_._w_H_gp,
                                                                        robot_._w_H_eeStandby,
                                                                        object_._w_H_o,
                                                                        tossVar_.release_position,
                                                                        desVtoss_,
                                                                        0.05f,
                                                                        0.100f,
                                                                        initSpeedScaling_);
  }

  // -------------------------------------------------------------
  Eigen::Vector2f Lp_Va_pred_bot = {dualPathLenAvgSpeed_(0), trackingFactor_ * dualPathLenAvgSpeed_(1)};
  Eigen::Vector2f Lp_Va_pred_tgt =
      tossParamEstimator_.estimateTarget_SimpPathLength_AverageSpeed(target_._xt, target_._xd_landing, target_._vt);
  //
  float flytime_obj = 0.200f;//0.255f;
  float eps_den = 1e-6;
  float time2intercept_tgt =
      fabs(fabs(Lp_Va_pred_tgt(0) + eps_den - Lp_Va_pred_tgt(1) * flytime_obj) / (Lp_Va_pred_tgt(1) + eps_den));
  float time2intercept_bot = Lp_Va_pred_bot(0) / Lp_Va_pred_bot(1);
  //
  timeToInterceptTgt_ = 0.0f;
  timeToInterceptBot_ = 0.0f;
  // -------------------------------------------------------------

  // determine the desired landing position
  this->find_desired_landing_position(x_origin, isPlacing, isPlaceTossing, isThrowing);// ---> _xd_landing
  // this->set_2d_position_box_constraints(_xd_landing, intercep_limits);
  std::cout << " DDDDDDDDDDDDDDDD  XD LANDING IS : \t " << target_._xd_landing.transpose() << std::endl;
  // Estimate the target state to go
  this->estimate_target_state_to_go(Lp_Va_pred_bot, Lp_Va_pred_tgt, flytime_obj);// ---> _xt_state2go

  // set at pickup instant
  if (!isPickupSet_ && !releaseAndretract_) {
    if (sensedContact_ && (CooperativeCtrl._ContactConfidence == 1.0)) {
      // Update intercept (desired landing) position
      this->update_intercept_position(flytime_obj, intercep_limits);// ---> _xd_landing
      // Determination of the release configuration
      this->find_release_configuration();// ---> tossVar_.release_position  tossVar_.release_linear_velocity
      // set the release state and the object pickup position
      this->set_release_state();// <----- tossVar_.release_position  tossVar_.release_linear_velocity
      //
      isPickupSet_ = true;
    } else {
      object_._x_pickup = object_._xo;
      dsThrowing_.set_pickup_object_pose(object_._x_pickup, object_._qo);
    }
  }

  // Adaptation of the desired motion
  this->compute_adaptation_factors(Lp_Va_pred_bot, Lp_Va_pred_tgt, flytime_obj);
  // ===========================================================================================================

  // ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  if (goHome_)
  // ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  {
    //
    freeMotionCtrl_
        .computeAsyncMotion(robot_._w_H_ee, robot_._w_H_eeStandby, object_._w_H_o, robot_._Vd_ee, robot_._qd, true);
    //
    objVelDes_ = dsThrowing_.apply(object_._xo,
                                   object_._qo,
                                   object_._vo,
                                   Eigen::Vector3f(0.0f, 0.0f, 0.0f),
                                   1);// 	Function to call in a loop
    //
    for (int i = 0; i < NB_ROBOTS; i++) {
      dirImp_[i] = this->get_impact_direction(objVelDes_.head(3), object_._n[i], frictionAngle_);//  impact direction
      vdImpact_[i] = desVimp_ * dirImp_[i];//	impact velocity [LEFT];
      basisQ_[i] =
          Utils<float>::create3dOrthonormalMatrixFromVector(dirImp_[i]);//  Orthogonal Basis of Modulated Dual-arm DS
    }
    // if( ((initPoseCount_ > 50) && ((xt_bar - xt2go_bar).norm() < 0.04f)) && (!hasCaughtOnce_) )// 0.80     // tossing isMotionTriggered
    // if((initPoseCount_ > 50) && (fabs(_xo(1)-y_2_go) < 0.02f) && (!hasCaughtOnce_)) 						// 0.80   	// catching

    // if(isMotionTriggered_ && (!hasCaughtOnce_)){
    // 	goHome_ 				= false;
    // 	hasCaughtOnce_ 	= true;
    // 	// startlogging_  = true;
    // }

    // reset some controller variables
    this->reset_variables();

  } else {
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (releaseAndretract_)//  release_and_retract || release
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    {
      freeMotionCtrl_.computeReleaseAndRetractMotion(robot_._w_H_ee,
                                                     object_._w_H_Dgp,
                                                     object_._w_H_o,
                                                     robot_._Vd_ee,
                                                     robot_._qd,
                                                     true);
      isThrowing_ = false;
      isPlacing_ = false;
      isPickupSet_ = false;
      isPlaceTossing_ = false;
      nuWr0_ = nuWr1_ = 0.0f;
      dsThrowing_.reset_release_flag();
      isIntercepting_ = false;
    }
    //===================================================================================================================
    else if (isContact)// Constraint motion phase (Cooperative control)
    //===================================================================================================================
    {
      objVelDes_ = dsThrowing_.apply(object_._xo,
                                     object_._qo,
                                     object_._vo,
                                     Eigen::Vector3f(0.0f, 0.0f, 0.0f),
                                     1);// Function to call in a loop

      Eigen::Vector3f xDesTask = xLifting_;
      Eigen::Vector4f qDesTask = qLifting_;

      //----------------------------------------------
      // desired task position and orientation vectors
      //----------------------------------------------
      if (isPlacing) {
        xDesTask = xPlacing_;
        qDesTask = qPlacing_;
      }
      if (isPlaceTossing) {
        xDesTask = xPlacing_;
        qDesTask = qPlacing_;
      }
      if (isThrowing) {
        xDesTask = tossVar_.release_position;
        qDesTask = tossVar_.release_orientation;
      }
      // Target to object Orientation Adaptation
      // ----------------------------------------
      if (trackTargetRotation_) {// && !(isPlacing || isThrowing || isPlaceTossing)){
        this->mirror_target2object_orientation(target_._qt, qDesTask, dualAngularLimit_);
        dsThrowing_.set_toss_pose(tossVar_.release_position, qDesTask);//	<<==============
      }

      // Desired object pose
      //--------------------
      Eigen::Matrix4f w_H_DesObj = Utils<float>::pose2HomoMx(xDesTask, qDesTask);
      object_._w_H_Do = Utils<float>::pose2HomoMx(xDesTask, qDesTask);//
      // Desired pose of the grasping points
      //------------------------------------
      for (int k = 0; k < NB_ROBOTS; k++) {
        object_._w_H_Dgp[k] = object_._w_H_Do * oHEE_[k];
        object_._w_H_Dgp[k].block(0, 0, 3, 3) = w_H_DesObj.block(0, 0, 3, 3)
            * Utils<float>::pose2HomoMx(object_._xgp_o[k], object_._qgp_o[k]).block(0, 0, 3, 3);
        // object_._w_H_Dgp[k].block(0,0,3,3)  = object_._w_H_o.block(0,0,3,3) * Utils<float>::pose2HomoMx(object_._xgp_o[k], object_._qgp_o[k]).block(0,0,3,3);
        if (isThrowing && isClose2Release) {
          // object_._w_H_Dgp[k]  = object_._w_H_Do * oHEE_[k];
          // object_._w_H_Dgp[k]  = object_._w_H_o * oHEE_[k];
        }
      }

      //
      // Motion generation
      //-------------------
      freeMotionCtrl_.dual_arm_motion(robot_._w_H_ee,
                                      robot_._Vee,
                                      object_._w_H_Dgp,
                                      object_._w_H_o,
                                      object_._w_H_Do,
                                      objVelDes_,
                                      basisQ_,
                                      vdImpact_,
                                      false,
                                      dualTaskSelector_,
                                      robot_._Vd_ee,
                                      robot_._qd,
                                      releaseFlag_);// 0=reach, 1=pick, 2=toss, 3=pick_and_toss, 4=pick_and_place

      // Release and Retract condition
      //------------------------------
      // releaseAndretract_ = dsThrowing_.get_release_flag();
      if ((isPlacing && placing_done) || (isPlaceTossing && placeTossing_done) || (isThrowing && tossing_done)) {
        releaseAndretract_ = true;
      }

    }
    //======================================================================================================================================
    else// Unconstraint (Free) motion phase
    //======================================================================================================================================
    {
      freeMotionCtrl_.reachable_p =
          (robot_._w_H_ee[LEFT](0, 3) >= 0.72f || robot_._w_H_ee[RIGHT](0, 3) >= 0.72f) ? 0.0f : 1.0f;

      if (false || oldDualMethod_) {
        freeMotionCtrl_.computeCoordinatedMotion2(robot_._w_H_ee,
                                                  object_._w_H_gp,
                                                  object_._w_H_o,
                                                  robot_._Vd_ee,
                                                  robot_._qd,
                                                  false);
        // freeMotionCtrl_.computeCoordinatedMotion3(_w_H_ee, _w_H_gp, _w_H_o, _Vo, _x_intercept, _Vd_ee, _qd, false);
        //
        Eigen::Vector3f error_p_abs = object_._w_H_o.block(0, 3, 3, 1) - Utils<float>::get_abs_3d(robot_._w_H_ee);
        Eigen::Vector3f o_error_pos_abs = object_._w_H_o.block<3, 3>(0, 0).transpose() * error_p_abs;
        Eigen::Vector3f o_error_pos_abs_paral = Eigen::Vector3f(o_error_pos_abs(0), 0.0f, o_error_pos_abs(2));
        float cp_ap =
            Utils<float>::computeCouplingFactor(o_error_pos_abs_paral, 50.0f, 0.17f, 1.0f, true);// 50.0f, 0.05f, 2.8f
        // create impact at grabbing
        robot_._Vd_ee[LEFT].head(3) = robot_._Vd_ee[LEFT].head(3) + dirImp_[LEFT] * cp_ap * desVimp_;   //0.05f; //
        robot_._Vd_ee[RIGHT].head(3) = robot_._Vd_ee[RIGHT].head(3) + dirImp_[RIGHT] * cp_ap * desVimp_;//0.05f; //
      } else {
        freeMotionCtrl_.dual_arm_motion(robot_._w_H_ee,
                                        robot_._Vee,
                                        object_._w_H_gp,
                                        object_._w_H_o,
                                        object_._w_H_Do,
                                        objVelDes_,
                                        basisQ_,
                                        vdImpact_,
                                        false,
                                        0,
                                        robot_._Vd_ee,
                                        robot_._qd,
                                        releaseFlag_);// 0: reach
      }
      dsThrowing_._refVtoss = desVimp_;
      objVelDes_.setZero();// for data logging
      //
      if (freeMotionCtrl_.a_proximity_ >= 0.2f) { betaVelModUnfilt_ = 1.0; }
    }
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////
    if (isPlacing || isThrowing || isPlaceTossing) {
      // force feedback to grab objects
      float f_gain = 0.02f;
      float abs_force_correction = nuWr0_ * f_gain * 0.5f
          * ((robot_._filteredWrench[LEFT].segment(0, 3) - CooperativeCtrl._f_applied[LEFT].head(3))
                 .dot(object_._n[LEFT])
             + (robot_._filteredWrench[RIGHT].segment(0, 3) - CooperativeCtrl._f_applied[RIGHT].head(3))
                   .dot(object_._n[RIGHT]));
      if (fabs(abs_force_correction) > 0.2f) {
        abs_force_correction = abs_force_correction / fabs(abs_force_correction) * 0.2f;
      }
      robot_._Vd_ee[LEFT].head(3) =
          robot_._Vd_ee[LEFT].head(3) - 0.40 * abs_force_correction * object_._n[LEFT];// 0.6 (heavy objects)
      robot_._Vd_ee[RIGHT].head(3) =
          robot_._Vd_ee[RIGHT].head(3) - 0.40 * abs_force_correction * object_._n[RIGHT];// 0.6 (heavy objects)
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Adaptation
    // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    if (!isContact && (freeMotionCtrl_.a_proximity_ >= 0.2f)) { beta_vel_mod_unfilt = 1.0; }
    // -----------------------------------------------------------------------------------------------------------------------------
    float fil_beta = 0.10;
    betaVelMod_ = (1.f - fil_beta) * betaVelMod_ + fil_beta * beta_vel_mod_unfilt;

    if ((target_._vt.norm() >= 0.05 && (!releaseAndretract_) && (dsThrowing_.a_proximity_ <= 0.99f))) {
      robot_._Vd_ee[LEFT].head(3) *=
          initSpeedScaling_ * ((float) adaptationActive_ * betaVelMod_ + (1. - (float) adaptationActive_));
      robot_._Vd_ee[RIGHT].head(3) *=
          initSpeedScaling_ * ((float) adaptationActive_ * betaVelMod_ + (1. - (float) adaptationActive_));
    }
    // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // compute the object's grasp points velocity
    getGraspPointsVelocity();

    // -------------------------------------------------------
    // Generate grasping force and apply it in velocity space
    //--------------------------------------------------------
    // Desired object's task wrench
    desiredObjectWrench_.head(3) =
        -12.64f * (object_._vo - freeMotionCtrl_.get_des_object_motion().head(3)) - object_._objectMass * gravity_;
    desiredObjectWrench_.tail(3) = -25.00f * (object_._wo - freeMotionCtrl_.get_des_object_motion().tail(3));

    CooperativeCtrl.getAppliedWrenches(goHome_,
                                       contactState_,
                                       object_._w_H_o,
                                       robot_._w_H_ee,
                                       object_._w_H_gp,
                                       desiredObjectWrench_,
                                       object_._objectMass,
                                       qpWrenchGeneration_,
                                       isForceDetected);

    // applied force in velocity space
    for (int i = 0; i < NB_ROBOTS; i++) { robot_._fxc[i] = 1.0f / d1_[i] * CooperativeCtrl._f_applied[i].head(3); }
  }
  // compute the velocity to avoid EE collision
  freeMotionCtrl_.compute_EE_avoidance_velocity(robot_._w_H_ee, robot_._VEE_oa);

  // Extract linear velocity commands and desired axis angle command
  prepareCommands(robot_._Vd_ee, robot_._qd, object_._V_gpo);

  // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // control of conveyor belt speed
  // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  float omega_pert = 2.f * M_PI / 1;
  float delta_omega_pert = 0.1f * (2.f * (float) std::rand() / RAND_MAX - 1.0f) * omega_pert;
  // if(ctrlModeConveyorBelt_ && isDisturbTarget_){
  if (ctrlModeConveyorBelt_) {
    desSpeedConveyorBelt_ = (int) (nominalSpeedConveyorBelt_
                                   + (int) isDisturbTarget_ * magniturePertConveyorBelt_
                                       * sin((omega_pert + delta_omega_pert) * dt_ * cycleCount_));
  }

  //---------------------------------------------------------------------------------------------------------------------------------------------
  std::cout << " MEASURED HAND WRENCH _filteredWrench  LEFT \t " << robot_._filteredWrench[LEFT].transpose()
            << std::endl;
  std::cout << " MEASURED HAND WRENCH _filteredWrench RIGHT \t " << robot_._filteredWrench[RIGHT].transpose()
            << std::endl;
  std::cout << "[dualArmControl]: _w_H_o: \n" << object_._w_H_o << std::endl;
  std::cout << "[dualArmControl]: _w_H_Do: \n" << object_._w_H_Do << std::endl;
  std::cout << "[dualArmControl]: _w_H_t: \n" << Utils<float>::quaternionToRotationMatrix(target_._qt) << std::endl;
  std::cout << "[dualArmControl]: robot_._w_H_ee[LEFT]: \n" << robot_._w_H_ee[0] << std::endl;
  std::cout << "[dualArmControl]: _w_H_Dgp[LEFT]: \n" << object_._w_H_Dgp[0] << std::endl;
  std::cout << "[dualArmControl]: robot_._w_H_ee[RIGHT]: \n" << robot_._w_H_ee[1] << std::endl;// robot_._w_H_eeStandby
  std::cout << "[dualArmControl]: _w_H_Dgp[RIGHT]: \n" << object_._w_H_Dgp[1] << std::endl;

  std::cout << "[dualArmControl]: 3D STATE 2 GO : \t" << target_._xt_state2go.transpose() << std::endl;
  std::cout << "[dualArmControl]:  ------------- sensedContact_: \t" << sensedContact_ << std::endl;
  std::cout << "[dualArmControl]:  ------------- isContact: \t" << isContact << std::endl;
  std::cout << "[dualArmControl]: _Vd_ee[LEFT]:  \t" << robot_._Vd_ee[LEFT].transpose() << std::endl;
  std::cout << "[dualArmControl]: _Vd_ee[RIGHT]: \t" << robot_._Vd_ee[RIGHT].transpose() << std::endl;
  std::cout << " vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv " << std::endl;
  std::cout << "[dualArmControl]: _vd[LEFT]:  \t" << robot_._vd[LEFT].transpose() << std::endl;
  std::cout << "[dualArmControl]: _vd[RIGHT]: \t" << robot_._vd[RIGHT].transpose() << std::endl;
  std::cout << " ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ " << std::endl;
  std::cout << " COMPUTED HAND WRENCH _fxc  LEFT \t " << robot_._fxc[LEFT].transpose() << std::endl;
  std::cout << " COMPUTED HAND WRENCH _fxc RIGHT \t " << robot_._fxc[RIGHT].transpose() << std::endl;
  // std::cout << " [dualArmControl]: dirImp_[LEFT] \t " << dirImp_[LEFT].transpose()   << " normal LEFT  \t " << _n[LEFT].transpose()<< std::endl;
  // std::cout << " [dualArmControl]: dirImp_[RIGHT] \t " << dirImp_[RIGHT].transpose() << " normal RIGHT \t " << _n[RIGHT].transpose()<< std::endl;
  std::cout << " EEEE----------- EEEPPP   desVtoss_ IIIIIIII ----------- ONNNNNNNN \t " << desVtoss_ << std::endl;
  std::cout << " EEEE----------- EEEPPP   desVimp_  IIIIIIII ----------- ONNNNNNNN \t " << desVimp_ << std::endl;
  // std::cout << " EEEE- POSITION PLACING is  \t " << xPlacing_.transpose() << std::endl;
  // std::cout << " EEEE- POSITION LIFTING is  \t " << xLifting_.transpose() << std::endl;
  std::cout << " EEEE- OBJECT MASS is  \t " << object_._objectMass << std::endl;
  std::cout << " CONVEYOR_BELT SPEED is  \t " << desSpeedConveyorBelt_ << std::endl;
  std::cout << " CONVEYOR_BELT DISTURBED is  \t " << isDisturbTarget_ << std::endl;
  std::cout << " INTERCEPT STATUS is  \t " << hasCaughtOnce_ << std::endl;

  std::cout << " HOME STATUS is --------------------------> : \t " << goHome_ << std::endl;
  std::cout << " RELEASE_AND_RETRACT STATUS is -----------> : \t " << releaseAndretract_ << std::endl;
  switch (dualTaskSelector_) {
    case 0:
      std::cout << " DUAL_ARM MODE is -----------------------> : \t "
                << "REACHING-TO-GRASP" << std::endl;
      break;
    case 1:
      std::cout << " DUAL_ARM MODE is -----------------------> : \t "
                << "LIFTING" << std::endl;
      break;
    case 2:
      std::cout << " DUAL_ARM MODE is -----------------------> : \t "
                << "TOSSING" << std::endl;
      break;
    case 3:
      std::cout << " DUAL_ARM MODE is -----------------------> : \t "
                << "PICK_AND_TOSS" << std::endl;
      break;
    case 4:
      std::cout << " DUAL_ARM MODE is -----------------------> : \t "
                << "PICK_AND_PLACE" << std::endl;
      break;
    case 5:
      std::cout << " DUAL_ARM MODE is -----------------------> : \t "
                << "PLACE-TOSSING" << std::endl;
      break;
  }
  std::cout << " AAAA  TRACKING FACTOR AAAAA is  \t " << trackingFactor_ << std::endl;
  std::cout << " AAAA  ADAPTATION STATUS AAAAA is  -----------> : \t " << adaptationActive_ << std::endl;
  std::cout << " PPPPPPPPPPPPPPP xPlacing_ PPPPPPPPPP  is  -----------> : \t " << xPlacing_.transpose() << std::endl;
  std::cout << " TTTTTTTTTTTTTTT _xTarget TTTTTTTTTTT  is  -----------> : \t " << target_._xt.transpose() << std::endl;
  //-------------------------------------
  // -------------------------------------------------------------------------------
  std::cout << " PPPPPPPPPPPPPPP _dual_PathLen PPPPPPPPPP  is  -----------> : \t " << dualPathLenAvgSpeed_(0)
            << std::endl;
  std::cout << " PPPPPPPPPPPPPPP _dual_Path_AvgSpeed(1) PPPPPPPPPP  is  -----------> : \t " << dualPathLenAvgSpeed_(1)
            << std::endl;
  // std::cout << " PPPPPPPPPPPPPPP _Del_xEE_dual_avg PPPPPPPPPP  is  -----------> : \t " << _Del_xEE_dual_avg
  //           << std::endl;
  // std::cout << " PPPPPPPPPPPPPPP _dxEE_dual_avg PPPPPPPPPP  is  -----------> : \t " << _dxEE_dual_avg_pcycle
  //           << std::endl;
  // std::cout << " PPPPPPPPPPPPPPP _counter_pickup PPPPPPPPPP  is  -----------> : \t " << _counter_pickup << std::endl;
  // std::cout << "[dualArmControl]: robot_._w_H_eeStandby[LEFT]: \n" << robot_._w_H_eeStandby[0] << std::endl;  // robot_._w_H_eeStandby
  // std::cout << "[dualArmControl]: robot_._w_H_eeStandby[RIGHT]: \n" << robot_._w_H_eeStandby[1] << std::endl;  // robot_._w_H_eeStandby
  std::cout << "[dualArmControl]: desiredObjectWrench_:  WWWWWWWW  \t" << desiredObjectWrench_.transpose()
            << std::endl;//
  //---------------------------------------------------------------------------------------------------------------------------------------------
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void dualArmControl::prepareCommands(Vector6f Vd_ee[], Eigen::Vector4f qd[], Vector6f V_gpo[]) {
  Eigen::Matrix<float, 3, 1> axis_d[NB_ROBOTS];
  float angle_d[NB_ROBOTS];
  for (int i = 0; i < NB_ROBOTS; i++) {
    Vector6f VdEE = robot_._tcp_W_EE[i].inverse() * (Vd_ee[i] + V_gpo[i]);
    robot_._vd[i] = VdEE.head(3) + 1.0 * robot_._VEE_oa[i].head(3);
    robot_._omegad[i] = VdEE.tail(3);

    Utils<float>::quaternionToAxisAngle(qd[i], axis_d[i], angle_d[i]);
    robot_._aad[i] = angle_d[i] * axis_d[i];
  }

  // --------------------------------------------------------
  if (goToAttractors_ && sensedContact_ && CooperativeCtrl._ContactConfidence == 1.0f) {
    nuWr0_ = 0.80f * nuWr0_ + 0.20f;
    nuWr1_ = 0.92f * nuWr1_ + 0.08f;
  } else {
    nuWr0_ = 0.0f;
    nuWr1_ = 0.0f;
  }

  if (releaseAndretract_) {
    robot_._fxc[0].setZero();
    robot_._fxc[1].setZero();
    nuWr0_ = 0.0f;
    nuWr1_ = 0.0f;
  }
  //
  if (goToAttractors_) {
    applyVelo_ = 1.f;
  } else {
    applyVelo_ = 0.0f;
    // keep the current orientation if not going to the attractor
    robot_._qd[LEFT] = robot_._q[LEFT];
    robot_._qd[RIGHT] = robot_._q[RIGHT];
  }
  // set the command to send
  robot_.get_desired_lin_task_velocity(applyVelo_, nuWr0_);
}

//
void dualArmControl::getGraspPointsVelocity()// in object struct or object
{
  object_.get_grasp_point_velocity();
}

void dualArmControl::Keyboard_reference_object_control()// control of attractor position through keyboad
{
  // _w_H_Do(0,3) += deltaPos_(0);
  // _w_H_Do(1,3) += deltaPos_(1);
  // _w_H_Do(2,3) += deltaPos_(2);
  xLifting_(0) += deltaPos_(0);
  xLifting_(1) += deltaPos_(1);
  xLifting_(2) += deltaPos_(2);

  // Eigen::Matrix3f RDo_lifting = Utils<float>::quaternionToRotationMatrix(qLifting_);
  // Eigen::Vector3f eulerAng_lift = Utils<float>::getEulerAnglesXYZ_FixedFrame(RDo_lifting);
  // eulerAng_lift += deltaAng_;
  // RDo_lifting  = Utils<float>::eulerAnglesToRotationMatrix(	eulerAng_lift(2), eulerAng_lift(1), eulerAng_lift(0));
  // qLifting_ = Utils<float>::rotationMatrixToQuaternion(RDo_lifting);
  //

  // filtDeltaAng_ = 0.95*filtDeltaAng_ + 0.05*deltaAng_;
  // // Eigen::Matrix3f RDo_lifting  = Utils<float>::eulerAnglesToRotationMatrix(	filtDeltaAng_(2), filtDeltaAng_(1), filtDeltaAng_(0));
  // Eigen::Matrix3f RDo_lifting  = Utils<float>::eulerAnglesToRotationMatrix(	filtDeltaAng_(0), filtDeltaAng_(1), filtDeltaAng_(2));
  // qLifting_ = Utils<float>::rotationMatrixToQuaternion(RDo_lifting);
  // //
  // std::cout << "AAAAAAAAAAAAAAAAAA ROTATION ANGLE EULER XYZ \t" << filtDeltaAng_.transpose() << std::endl;
  //

  deltaPos_.setZero();
  // deltaAng_.setZero();
}

void dualArmControl::mirror_target2object_orientation(Eigen::Vector4f qt,
                                                      Eigen::Vector4f& qo,
                                                      Eigen::Vector3f ang_lim) {
  Eigen::Vector3f eAng_t = Utils<float>::getEulerAnglesXYZ_FixedFrame(Utils<float>::quaternionToRotationMatrix(qt));
  Eigen::Vector3f eAng_o = Utils<float>::getEulerAnglesXYZ_FixedFrame(Utils<float>::quaternionToRotationMatrix(qo));
  // eAng_o(2) = eAng_o(2) + eAng_t(2);

  filtDeltaAngMir_ = 0.95 * filtDeltaAngMir_ + 0.05 * (eAng_t - eAng_o);
  eAng_t(0) = eAng_o(0) + filtDeltaAngMir_(0);
  eAng_t(2) = eAng_o(2) + filtDeltaAngMir_(2);

  if (eAng_t(0) >= ang_lim(0)) {
    eAng_t(0) = ang_lim(0);
  } else if (eAng_t(0) <= -ang_lim(0)) {
    eAng_t(0) = -ang_lim(0);
  }
  //
  if (eAng_t(2) >= ang_lim(2)) {
    eAng_t(2) = ang_lim(2);
  } else if (eAng_t(2) <= -ang_lim(2)) {
    eAng_t(2) = -ang_lim(2);
  }

  std::cout << " QUAT q0 before is : \t" << qo.transpose() << std::endl;
  // qo = Utils<float>::rotationMatrixToQuaternion( Utils<float>::eulerAnglesToRotationMatrix(eAng_t(2), eAng_o(1), eAng_o(0)) );
  qo = Utils<float>::rotationMatrixToQuaternion(
      Utils<float>::eulerAnglesToRotationMatrix(eAng_o(0), eAng_o(1), eAng_t(2)));
  // qo = Utils<float>::rotationMatrixToQuaternion( Utils<float>::eulerAnglesToRotationMatrix(eAng_o(0), eAng_o(1), eAng_o(2)) );

  std::cout << " QUAT q0 After is : \t" << qo.transpose() << std::endl;
}

// //
void dualArmControl::Keyboard_virtual_object_control()// control of object position through keyboad
{
  // object2grasp.States_Object.pose.head(3) = ioSM->w_H_absF.block<3,3>(0,0) * init_obj_aF + ioSM->w_H_absF.block<3,1>(0,3); //
  object_._w_H_o(0, 3) += deltaPos_(0);
  object_._w_H_o(1, 3) += deltaPos_(1);
  object_._w_H_o(2, 3) += deltaPos_(2);// deltaAng_

  Eigen::Vector3f ang_o = Utils<float>::getEulerAnglesXYZ_FixedFrame(object_._w_H_o.block<3, 3>(0, 0));// psi theta phi
  // ang_o += deltaAng_;
  // object_._w_H_o.block<3,3>(0,0) = Utils<float>::eulerAnglesToRotationMatrix(	ang_o(2), ang_o(1), ang_o(0)); // phi theta psi
}
//
Eigen::Vector3f
dualArmControl::get_impact_direction(Eigen::Vector3f des_object_force, Eigen::Vector3f normal, float coeff_friction) {
  //
  Eigen::Matrix3f R1;
  Eigen::Matrix3f R0;
  Utils<float>::Orthobasis(des_object_force, normal, R1, R0);
  //
  float theta = 0.0f;
  if (this->impactDirPreset_) {
    theta = M_PI / 180.f * coeff_friction;
  } else {
    float vzo = des_object_force.transpose() * R0.col(2);
    float vxo = des_object_force.transpose() * normal;
    theta = std::atan2(vzo, vxo);
  }
  if (fabs(theta) >= M_PI / 180.f * this->frictionAngleMax_) {
    theta = theta / fabs(theta) * M_PI / 180.f * frictionAngleMax_;
  }
  Eigen::Vector3f imp_dir = R0 * Eigen::Vector3f(std::cos(theta), 0.0f, std::sin(theta));

  return imp_dir.normalized();
}

void dualArmControl::reset_variables() {

  releaseAndretract_ = false;
  isThrowing_ = false;
  isPlacing_ = false;
  isPlaceTossing_ = false;
  isPickupSet_ = false;
  isIntercepting_ = false;
  nuWr0_ = nuWr1_ = 0.0f;
  this->refVreach_ = 0.0f;

  freeMotionCtrl_._refVreach[LEFT] = 0.0f;
  freeMotionCtrl_._refVreach[RIGHT] = 0.0f;
  dsThrowing_._refVtoss = desVimp_;
  dsThrowing_.reset_release_flag();
  objVelDes_.setZero();

  for (int i = 0; i < NB_ROBOTS; i++) {
    object_._V_gpo[i].setZero();
    robot_._fxc[i].setZero();
  }
  //
  if (adaptationActive_) {
    tossVar_.release_position(1) = 0.0f;
    xPlacing_(1) = 0.0f;// TBC !!!!
  }
  //
  // TODO never used only set
  // _dxEE_dual_avg = 0.0f;
  // _Del_xEE_dual_avg = 0.0f;
}

Eigen::Vector3f dualArmControl::get_object_desired_direction(int task_type, Eigen::Vector3f object_pos) {
  // 0=reach, 1=pick, 2=toss, 3=pick_and_toss, 4=pick_and_place
  Eigen::Vector3f des_object_pos = object_._xDo;
  switch (task_type) {
    case 1:
      des_object_pos = xLifting_;
      break;
    case 2:
      des_object_pos = dsThrowing_.Xt_;
      break;
    case 3:
      des_object_pos = dsThrowing_.Xt_;
      break;
    case 4:
      des_object_pos = 0.5f * (object_pos + xPlacing_);
      des_object_pos(2) = xPlacing_(2) + heightViaPoint_;
      break;
    case 5:
      des_object_pos = 0.5f * (object_pos + xPlacing_);
      des_object_pos(2) = xPlacing_(2) + heightViaPoint_;
      break;
    default:
      des_object_pos = object_._xDo;
      break;
  }
  //
  Eigen::Vector3f error_obj_pos = des_object_pos - object_pos;
  return error_obj_pos.normalized();
}

//
void dualArmControl::update_release_position() {
  releasePos_.r += deltaRelPos_(0);
  releasePos_.theta += M_PI / 180.0f * deltaRelPos_(1);
  releasePos_.phi += M_PI / 180.0f * deltaRelPos_(2);
  //
  // xPlacing_ += deltaRelPos_;
  //
  deltaRelPos_.setZero();
  //
  Eigen::Vector3f pos_xo;
  releasePos_.toCartesian(pos_xo);
  tossVar_.release_position = pos_xo + xLifting_;//_xo;
  if (tossVar_.release_position(0) > 0.70) {     // 0.65
    tossVar_.release_position(0) = 0.70;
  }
}

void dualArmControl::update_placing_position(float y_t_min, float y_t_max) {
  xPlacing_.head(2) = xPlacing_.head(2) + adaptationActive_ * dt_ * (-5.0f * (xPlacing_.head(2) - target_._xt.head(2)));
  if ((target_._xt(1) >= y_t_min) && (target_._xt(1) <= y_t_max)) {
    xPlacing_.head(2) = xPlacing_.head(2)
        + adaptationActive_ * dt_ * (target_._vt.head(2) - 5.0f * (xPlacing_.head(2) - target_._xt.head(2)));
  }
}

void dualArmControl::constrain_placing_position(float x_t_min, float x_t_max, float y_t_min, float y_t_max) {
  if (xPlacing_(0) < x_t_min) xPlacing_(0) = x_t_min;
  if (xPlacing_(0) > x_t_max) xPlacing_(0) = x_t_max;
  if (xPlacing_(1) < y_t_min) xPlacing_(1) = y_t_min;
  if (xPlacing_(1) > y_t_max) xPlacing_(1) = y_t_max;
}

void dualArmControl::set_2d_position_box_constraints(Eigen::Vector3f& position_vec, float limits[]) {
  if (position_vec(0) < limits[0]) position_vec(0) = limits[0];// x_min
  if (position_vec(0) > limits[1]) position_vec(0) = limits[1];// x_max
  if (position_vec(1) < limits[2]) position_vec(1) = limits[2];// y_min
  if (position_vec(1) > limits[3]) position_vec(1) = limits[3];// y_max
}

Eigen::Vector3f dualArmControl::compute_intercept_with_target(const Eigen::Vector3f& x_pick,
                                                              const Eigen::Vector3f& x_target,
                                                              const Eigen::Vector3f& v_target,
                                                              float phi_i) {
  //
  float phi_target = 0.0f;
  if (v_target.head(2).norm() > 1e-2) { phi_target = std::atan2(v_target(1), v_target(0)); }
  float tan_phi_t = std::tan(phi_target);
  if (tan_phi_t > 1e4) { tan_phi_t = 1e4; }
  if (-tan_phi_t < -1e4) { tan_phi_t = -1e4; }
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
  float x_coord_land = x_target(0);
  float y_coord_land = x_target(1);
  float tang_phi_throw = std::tan(phi_i);
  std::cout << " TTTTTTTTTTTTTTTTT PHI Throw  TTTTTTTTTTTis  -----------> : \t " << 180.f / M_PI * phi_i << std::endl;

  if (v_target.head(2).norm() > 1e-2) {
    float phi_conveyor = std::atan2(v_target(1), v_target(0));
    float tang_phi_conv = std::tan(phi_conveyor);
    x_coord_land = ((tang_phi_conv * x_target(0) - x_target(1)) / (tang_phi_conv - tang_phi_throw));
  } else {
    x_coord_land = x_target(0);
  }
  y_coord_land = x_coord_land * tang_phi_throw;
  //
  x_i << x_coord_land, y_coord_land, x_target(2);

  return x_i;
}

float dualArmControl::get_desired_yaw_angle_target(const Eigen::Vector4f& qt, const Eigen::Vector3f& ang_lim) {
  Eigen::Vector3f eAng_t = Utils<float>::getEulerAnglesXYZ_FixedFrame(Utils<float>::quaternionToRotationMatrix(qt));

  float phi_t_rot = eAng_t(2);
  //
  if (phi_t_rot >= ang_lim(2)) {
    phi_t_rot = ang_lim(2);
  } else if (phi_t_rot <= -ang_lim(2)) {
    phi_t_rot = -ang_lim(2);
  }

  return phi_t_rot;
}

void dualArmControl::estimate_moving_average_ee_speed() {
  // Estimation of moving average EE  and speed
  // -------------------------------------
  float avgSpeedEE = Utils<float>::get_abs_3d(robot_._Vee, true)
                         .norm();//0.5f*(_Vee[LEFT].head(3).norm() + _Vee[RIGHT].head(3).norm());

  //
  if (windowSpeedEE_.size() < winLengthAvgSpeedEE_) {
    windowSpeedEE_.push_back(avgSpeedEE);
    movingAvgSpeedEE_ = 0.0f;
  } else {
    windowSpeedEE_.pop_front();
    windowSpeedEE_.push_back(avgSpeedEE);
    movingAvgSpeedEE_ = 0.0f;

    for (int i = 0; i < winLengthAvgSpeedEE_; i++) { movingAvgSpeedEE_ += windowSpeedEE_[i] / winLengthAvgSpeedEE_; }
  }
}

void dualArmControl::estimate_moving_average_target_velocity() {
  // Estimation of moving average target velocity
  // --------------------------------------------
  if (windowVelTarget_.size() < winLengthAvgSpeedEE_) {
    windowVelTarget_.push_back(target_._vt);
    movingAvgVelTarget_.setZero();
  } else {
    windowVelTarget_.pop_front();
    windowVelTarget_.push_back(target_._vt);
    movingAvgVelTarget_.setZero();

    for (int i = 0; i < winLengthAvgSpeedEE_; i++) {
      movingAvgVelTarget_ += windowVelTarget_[i] * (1.f / winLengthAvgSpeedEE_);
    }
  }
}

void dualArmControl::find_desired_landing_position(Eigen::Vector3f x_origin,
                                                   bool isPlacing,
                                                   bool isPlaceTossing,
                                                   bool isThrowing) {
  // // determine the throwing/placing direction
  // float feas_yaw_target = this->get_desired_yaw_angle_target(_qt, dualAngularLimit_);
  // float phi_throwing    = feas_yaw_target;

  // // determine the intercept or desired landing position
  // // ----------------------------------------------------
  // if(isTargetFixed_){
  //   if( isPlacing || isPlaceTossing ){
  //     phi_throwing = std::atan2(xPlacing_(1), xPlacing_(0));
  //   }
  //   if( isThrowing ){
  //     phi_throwing = std::atan2(tossVar_.release_position(1), tossVar_.release_position(0));
  //   }
  //   _xd_landing = this->compute_intercept_with_target(x_origin, _xt,  _vt, phi_throwing);
  // }
  // else{
  //   _xd_landing = this->compute_intercept_with_target(x_origin, _xt,  _vt, feas_yaw_target);

  // }
  // ----------------------------------------------------------------------------------------
  Eigen::Vector3f xd_land = target_._xt;
  float phi_throwing = 0.0;

  // determine the intercept or desired landing position
  // ----------------------------------------------------------------------------------------
  if (userSelect_) {
    //
    if (isPlacing || isPlaceTossing) { xd_land.head(2) = xPlacing_.head(2); }
    if (isThrowing) { xd_land.head(2) = tossVar_.release_position.head(2); }
    //
    phi_throwing = std::atan2(xd_land(1), xd_land(0));

    if (isTargetFixed_) {
      target_._xd_landing = xd_land;
    } else {
      target_._xd_landing = this->compute_intercept_with_target(x_origin, target_._xt, target_._vt, phi_throwing);
    }
  } else {// autoSelect_
    phi_throwing = this->get_desired_yaw_angle_target(target_._qt, dualAngularLimit_);
    //
    target_._xd_landing = this->compute_intercept_with_target(x_origin, target_._xt, target_._vt, phi_throwing);
  }
  // -----------------------------------------------------------------------------------------
}

//
void dualArmControl::update_intercept_position(float flytime_obj, float intercep_limits[]) {

  Eigen::Vector3f x_t_intercept = target_._xt + target_._vt * (timeToInterceptBot_ + flytime_obj);
  // apply constraints
  this->set_2d_position_box_constraints(x_t_intercept, intercep_limits);
  //
  if (adaptationActive_) {
    target_._xd_landing.head(2) = x_t_intercept.head(2);
    if (isPlaceTossing_ || (dualTaskSelector_ == PLACE_TOSSING)) {
      xPlacing_.head(2) = target_._xt.head(2) + 0.35 * target_._vt.head(2) * (timeToInterceptBot_ + flytime_obj);
    }
  }
}

void dualArmControl::find_release_configuration() {// xD_landing
  // basic release configuration
  // ----------------------------
  if (feasibleAlgo_) {
    // generate using feasibilty algorithm
    tossVar_.release_position.head(2) = target_._xd_landing.head(2);//

    // extract from topics of the dual feasibility algo

    //
  } else if (pickupBased_) {
    //
    // tossVar_.release_position.head(2) = target_._xd_landing.head(2); //
    //
    Eigen::Vector3f x_release_bar = tossVar_.release_position - object_._x_pickup;
    float phi_throw_bar = std::atan2(x_release_bar(1), x_release_bar(0));

    tossVar_.release_linear_velocity << tossVar_.release_linear_velocity.head(2).norm() * std::cos(phi_throw_bar),
        tossVar_.release_linear_velocity.head(2).norm() * std::sin(phi_throw_bar), tossVar_.release_linear_velocity(2);
    tossVar_.release_linear_velocity = desVtoss_ * tossVar_.release_linear_velocity.normalized();
  } else {// user-defined
    tossVar_.release_position = tossVar_.release_position;
    tossVar_.release_linear_velocity = desVtoss_ * tossVar_.release_linear_velocity.normalized();
  }
}

void dualArmControl::set_release_state() {
  // set the release state and the object pickup position
  //-------------------------------------------------------
  dsThrowing_.set_toss_pose(tossVar_.release_position, tossVar_.release_orientation);
  dsThrowing_.set_toss_linear_velocity(tossVar_.release_linear_velocity);
  dsThrowing_.set_pickup_object_pose(object_._x_pickup, object_._qo);
}

//
void dualArmControl::estimate_target_state_to_go(Eigen::Vector2f Lp_Va_pred_bot,
                                                 Eigen::Vector2f Lp_Va_pred_tgt,
                                                 float flytime_obj) {
  // Estimation of the target state-to-go
  // -------------------------------------
  // Target state to go
  target_._xt_state2go = tossParamEstimator_.estimate_target_state_to_go(target_._xt,
                                                                         target_._vt,
                                                                         target_._xd_landing,
                                                                         Lp_Va_pred_bot,
                                                                         Lp_Va_pred_tgt,
                                                                         flytime_obj);

  // boolean robot's motion trigger
  Eigen::Vector3f xt_bar = target_._xt - target_._xd_landing;
  Eigen::Vector3f xt2go_bar = target_._xt_state2go - target_._xd_landing;
  isMotionTriggered_ =
      (-xt_bar.dot(target_._vt) > 0) && ((initPoseCount_ > 50) && ((xt_bar - xt2go_bar).norm() < 0.04f));
}

void dualArmControl::compute_adaptation_factors(Eigen::Vector2f Lp_Va_pred_bot,
                                                Eigen::Vector2f Lp_Va_pred_tgt,
                                                float flytime_obj) {
  if (isMotionTriggered_) { isIntercepting_ = true; }
  //
  float beta_vel_mod_max =
      min(2.0f, min((vMax_ / robot_._Vd_ee[LEFT].head(3).norm()), (vMax_ / robot_._Vd_ee[RIGHT].head(3).norm())));
  //
  if (isIntercepting_ && !releaseAndretract_) {
    //
    float eps_den = 1e-6;
    timeToInterceptTgt_ =
        fabs(fabs(Lp_Va_pred_tgt(0) + eps_den - Lp_Va_pred_tgt(1) * flytime_obj) / (Lp_Va_pred_tgt(1) + eps_den));
    timeToInterceptBot_ = Lp_Va_pred_bot(0) / Lp_Va_pred_bot(1);

    // Velocity-based adaptation factor
    //----------------------------------
    if (isRatioFactor_) {
      betaVelModUnfilt_ =
          fabs((Lp_Va_pred_tgt(1) / (Lp_Va_pred_bot(1) + eps_den))
               * (Lp_Va_pred_bot(0) / fabs(Lp_Va_pred_tgt(0) + eps_den - Lp_Va_pred_tgt(1) * flytime_obj)));
    } else {
      betaVelModUnfilt_ = (std::tanh(7.0f * (timeToInterceptBot_ - timeToInterceptTgt_)) + 1.0);
    }
    if (betaVelModUnfilt_ >= beta_vel_mod_max) { betaVelModUnfilt_ = beta_vel_mod_max; }
    // Attractor-based adaptation factor
    //-----------------------------------
    freeMotionCtrl_._activationAperture = adaptationActive_ ? 0.5f
            * (std::tanh(switchSlopeAdapt_
                         * ((target_._xd_landing - target_._xt).normalized().dot(target_._vt) - tolAttractor_))
               + 1.0)
                                                            : 1.0f;
  } else {
    betaVelModUnfilt_ = 1.0f;
    freeMotionCtrl_._activationAperture = 1.0f;
  }
}

// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Callback functions
// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// current_object_pose_world_callback
// -------------------------------------
void dualArmControl::objectPoseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
  // _xo << msg->position.x, 	msg->position.y, 	msg->position.z;
  Eigen::Vector3f xom, t_xo_xom;// _objectDim
  if (!isSimulation_) {
    t_xo_xom << 0.0f, 0.0f, -object_._objectDim(2) / 2.0f;
  } else {
    t_xo_xom << 0.0f, 0.0f, 0.0f;
  }
  xom << msg->position.x, msg->position.y, msg->position.z;
  object_._qo << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
  // _w_H_o = Utils<float>::pose2HomoMx(_xo, _qo);
  Eigen::Matrix3f w_R_o = Utils<float>::quaternionToRotationMatrix(object_._qo);
  object_._xo = xom + w_R_o * t_xo_xom;

  // TODO Never used only set
  //   _Vo.head(3) = object_._vo;
  //   _Vo.tail(3) = object_._wo;
  // _w_H_o = Utils<float>::pose2HomoMx(_xo, _qo);
  object_.get_HmgTransform();
}

void dualArmControl::targetPoseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
  target_._xt << msg->position.x, msg->position.y, msg->position.z;
  target_._qt << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
  // filtered object position
  target_.get_filtered_state();
}

void dualArmControl::updateBasePoseCallback(const geometry_msgs::Pose::ConstPtr& msg, int k) {
  Eigen::Vector3f xB = Eigen::Vector3f(msg->position.x, msg->position.y, msg->position.z);
  Eigen::Vector4f q = Eigen::Vector4f(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  robot_.get_robotBaseFrameInWorld(xB, q, k);
}

void dualArmControl::updateEEPoseCallback(const geometry_msgs::Pose::ConstPtr& msg, int k) {
  // Update end effecotr pose (position+orientation)
  Eigen::Vector3f xB = Eigen::Vector3f(msg->position.x, msg->position.y, msg->position.z);
  Eigen::Vector4f q = Eigen::Vector4f(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  robot_.update_EndEffectorPosesInWorld(xB, q, k);
}

void dualArmControl::updateEETwistCallback(const geometry_msgs::Twist::ConstPtr& msg, int k) {
  Eigen::Vector3f vE = Eigen::Vector3f(msg->linear.x, msg->linear.y, msg->linear.z);
  Eigen::Vector3f wE = Eigen::Vector3f(msg->angular.x, msg->angular.y, msg->angular.z);
  robot_.update_EndEffectorVelocity(vE, wE, k);
}

void dualArmControl::updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg, int k) {
  Eigen::Matrix<float, 6, 1> raw;
  raw(0) = msg->wrench.force.x;
  raw(1) = msg->wrench.force.y;
  raw(2) = msg->wrench.force.z;
  raw(3) = msg->wrench.torque.x;
  raw(4) = msg->wrench.torque.y;
  raw(5) = msg->wrench.torque.z;
  //
  robot_.update_EndEffectorWrench(raw, object_._n, filteredForceGain_, wrenchBiasOK_, k);
}

void dualArmControl::updateRobotStates(const sensor_msgs::JointState::ConstPtr& msg, int k) {
  //
  for (int i = 0; i < robot_._nb_joints[k]; i++) {
    robot_._joints_positions[k](i) = (float) msg->position[i];
    robot_._joints_velocities[k](i) = (float) msg->velocity[i];
    robot_._joints_torques[k](i) = (float) msg->effort[i];
    robot_.get_estimated_joint_accelerations(k);
  }
}

void dualArmControl::updateContactState() {
  robot_.get_estimated_AverageNormalForce();
  // Compute errors to object center position and dimension vector
  Eigen::Matrix4f le_H_re = robot_._w_H_ee[LEFT].inverse() * robot_._w_H_ee[RIGHT];
  Eigen::Matrix4f lgp_H_rgp = object_._w_H_gp[LEFT].inverse() * object_._w_H_gp[RIGHT];
  Eigen::Vector3f t_o_absEE = Utils<float>::get_abs_3d(object_._w_H_gp) - Utils<float>::get_abs_3d(robot_._w_H_ee);
  errorObjDim_ = fabs(le_H_re(2, 3)) - fabs(lgp_H_rgp(2, 3));
  errorObjPos_ = t_o_absEE.norm();

  if ((robot_._normalForceAverage[LEFT] > 2.0f || robot_._normalForceAverage[RIGHT] > 2.0f) && errorObjDim_ < 0.065f
      && (errorObjPos_ < 0.065f || CooperativeCtrl._ContactConfidence == 1.0f)) {
    contactState_ = CONTACT;
    isContact_ = 1.0f;
  } else if (!(robot_._normalForceAverage[LEFT] > 2.0f && robot_._normalForceAverage[RIGHT] > 2.0f)
             && errorObjDim_ < 0.05f && errorObjPos_ < 0.05f) {
    contactState_ = CLOSE_TO_CONTACT;
    isContact_ = 0.0f;
  } else {
    contactState_ = NO_CONTACT;
    isContact_ = 0.0f;
  }
  // check contact
  sensedContact_ =
      ((fabs(robot_._normalForce[LEFT]) >= forceThreshold_) || (fabs(robot_._normalForce[RIGHT]) >= forceThreshold_))
      && (isContact_ == 1.0f);
  // sensedContact_ = (fabs(normalForce_[LEFT]) >= forceThreshold_) && (fabs(normalForce_[RIGHT]) >= forceThreshold_) && (isContact_ == 1.0f);
  std::cerr << "[dualArmControl]: contact state: " << (int) contactState_ << " c: " << isContact_ << std::endl;
  std::cerr << "[dualArmControl]: robot_._normalForceAverage[LEFT]: " << robot_._normalForceAverage[LEFT] << std::endl;
  std::cerr << "[dualArmControl]: robot_._normalForceAverage[RIGHT]: " << robot_._normalForceAverage[RIGHT]
            << std::endl;
}

// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Publish commands and data
// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void dualArmControl::publish_commands() {
  //
  geometry_msgs::Pose vel_quat[NB_ROBOTS];
  //
  for (int k = 0; k < NB_ROBOTS; k++) {
    pubVel_[k].data.clear();
    pubVel_[k].data.push_back(robot_._aad[k](0));// axis angle pose_x
    pubVel_[k].data.push_back(robot_._aad[k](1));// axis angle pose_y
    pubVel_[k].data.push_back(robot_._aad[k](2));// axis angle pose_z
    pubVel_[k].data.push_back(robot_._vd[k](0)); // linear velocity v_x
    pubVel_[k].data.push_back(robot_._vd[k](1)); // linear velocity v_y
    pubVel_[k].data.push_back(robot_._vd[k](2)); // linear velocity v_z
                                                 // desired velocity
    vel_quat[k].position.x = robot_._vd[k](0);
    vel_quat[k].position.y = robot_._vd[k](1);
    vel_quat[k].position.z = robot_._vd[k](2);
    // desired pose
    vel_quat[k].orientation.w = robot_._qd[k](0);
    vel_quat[k].orientation.x = robot_._qd[k](1);
    vel_quat[k].orientation.y = robot_._qd[k](2);
    vel_quat[k].orientation.z = robot_._qd[k](3);
  }
  //
  pubTSCommands_[LEFT].publish(pubVel_[LEFT]);
  pubTSCommands_[RIGHT].publish(pubVel_[RIGHT]);
  pubDesiredVelQuat_[LEFT].publish(vel_quat[LEFT]);
  pubDesiredVelQuat_[RIGHT].publish(vel_quat[RIGHT]);
}

void dualArmControl::publishData() {
  for (int k = 0; k < NB_ROBOTS; k++) {
    // Publish desired twist
    geometry_msgs::Twist msgDesiredTwist;
    msgDesiredTwist.linear.x = robot_._vd[k](0);
    msgDesiredTwist.linear.y = robot_._vd[k](1);
    msgDesiredTwist.linear.z = robot_._vd[k](2);
    // Convert desired end effector frame angular velocity to world frame
    msgDesiredTwist.angular.x = robot_._omegad[k](0);
    msgDesiredTwist.angular.y = robot_._omegad[k](1);
    msgDesiredTwist.angular.z = robot_._omegad[k](2);
    pubDesiredTwist_[k].publish(msgDesiredTwist);
    // Publish desired orientation
    geometry_msgs::Quaternion msgDesiredOrientation;
    msgDesiredOrientation.w = robot_._qd[k](0);
    msgDesiredOrientation.x = robot_._qd[k](1);
    msgDesiredOrientation.y = robot_._qd[k](2);
    msgDesiredOrientation.z = robot_._qd[k](3);
    pubDesiredOrientation_[k].publish(msgDesiredOrientation);
    // filtered wrench
    geometry_msgs::WrenchStamped msgFilteredWrench;
    msgFilteredWrench.header.frame_id = "world";
    msgFilteredWrench.header.stamp = ros::Time::now();
    msgFilteredWrench.wrench.force.x = robot_._filteredWrench[k](0);
    msgFilteredWrench.wrench.force.y = robot_._filteredWrench[k](1);
    msgFilteredWrench.wrench.force.z = robot_._filteredWrench[k](2);
    msgFilteredWrench.wrench.torque.x = robot_._filteredWrench[k](3);
    msgFilteredWrench.wrench.torque.y = robot_._filteredWrench[k](4);
    msgFilteredWrench.wrench.torque.z = robot_._filteredWrench[k](5);
    pubFilteredWrench_[k].publish(msgFilteredWrench);
    // normal forces
    std_msgs::Float64 msg;
    msg.data = normalForce_[k];// TODO normalForce_ never set ?! should it be robot_._normalForce ?
    pubNormalForce_[k].publish(msg);
    //
    msg.data = err_[k];
    pubDistAttractorEE_[k].publish(msg);
    geometry_msgs::Pose msgPose;
    msgPose.position.x = object_._w_H_gp[k](0, 3);
    msgPose.position.y = object_._w_H_gp[k](1, 3);
    msgPose.position.z = object_._w_H_gp[k](2, 3);
    Eigen::Matrix3f Rgr = object_._w_H_gp[k].block(0, 0, 3, 3);
    Eigen::Quaternionf qgr(Rgr);
    msgPose.orientation.x = qgr.x();
    msgPose.orientation.y = qgr.y();
    msgPose.orientation.z = qgr.z();
    msgPose.orientation.w = qgr.w();
    pubAttractor_[k].publish(msgPose);
    // norm of desired velocity
    std_msgs::Float64 msgVel;
    msgVel.data = robot_._Vee[k].head(3).norm();
    pubNormLinVel_[k].publish(msgVel);

    // applied wrench
    geometry_msgs::Wrench msgAppliedWrench;
    msgAppliedWrench.force.x = -nuWr0_ * CooperativeCtrl._f_applied[k](0);
    msgAppliedWrench.force.y = -nuWr0_ * CooperativeCtrl._f_applied[k](1);
    msgAppliedWrench.force.z = -nuWr0_ * CooperativeCtrl._f_applied[k](2);
    msgAppliedWrench.torque.x = -nuWr0_ * CooperativeCtrl._f_applied[k](3);
    msgAppliedWrench.torque.y = -nuWr0_ * CooperativeCtrl._f_applied[k](4);
    msgAppliedWrench.torque.z = -nuWr0_ * CooperativeCtrl._f_applied[k](5);
    pubAppliedWrench_[k].publish(msgAppliedWrench);

    // contact normal and applied moment
    geometry_msgs::Wrench msgFnormMoment;
    msgFnormMoment.force.x = object_._n[k](0);
    msgFnormMoment.force.y = object_._n[k](1);
    msgFnormMoment.force.z = object_._n[k](2);
    msgFnormMoment.torque.x = -CooperativeCtrl._f_applied[k](3);
    msgFnormMoment.torque.y = -CooperativeCtrl._f_applied[k](4);
    msgFnormMoment.torque.z = -CooperativeCtrl._f_applied[k](5);
    pubAppliedFNormMoment_[k].publish(msgFnormMoment);
  }

  // send speed command to the conveyor belt
  std_msgs::Int32 _speedMessage;
  _speedMessage.data = desSpeedConveyorBelt_;
  if (ctrlModeConveyorBelt_ && (fmod(cycleCount_, 30) == 0)) { pubConveyorBeltSpeed_.publish(_speedMessage); }
}

void dualArmControl::saveData() {
  Eigen::Vector3f xgrL = object_._w_H_gp[LEFT].block(0, 3, 3, 1);
  Eigen::Vector3f xgrR = object_._w_H_gp[RIGHT].block(0, 3, 3, 1);
  Eigen::Vector4f qgrL = Utils<float>::rotationMatrixToQuaternion(object_._w_H_gp[LEFT].block(0, 0, 3, 3)); //
  Eigen::Vector4f qgrR = Utils<float>::rotationMatrixToQuaternion(object_._w_H_gp[RIGHT].block(0, 0, 3, 3));//
  //
  Eigen::MatrixXf power_left = robot_._joints_torques[LEFT].transpose() * robot_._joints_velocities[LEFT];
  Eigen::MatrixXf power_right = robot_._joints_torques[RIGHT].transpose() * robot_._joints_velocities[RIGHT];

  // if(startlogging_)
  // {
  dataLog_._OutRecord_pose << (float) (cycleCount_ * dt_) << ", ";// cycle time
  dataLog_._OutRecord_pose << robot_._x[LEFT].transpose().format(CSVFormat) << " , "
                           << robot_._q[LEFT].transpose().format(CSVFormat) << " , ";// left end-effector
  dataLog_._OutRecord_pose << robot_._x[RIGHT].transpose().format(CSVFormat) << " , "
                           << robot_._q[RIGHT].transpose().format(CSVFormat) << " , ";// right end-effector
  dataLog_._OutRecord_pose << object_._xo.transpose().format(CSVFormat) << " , "
                           << object_._qo.transpose().format(CSVFormat) << " , ";// object
  dataLog_._OutRecord_pose << object_._w_H_Do(0, 3) << " , " << object_._w_H_Do(1, 3) << " , " << object_._w_H_Do(2, 3)
                           << " , ";// desired object
  dataLog_._OutRecord_pose << xgrL.transpose().format(CSVFormat) << " , " << qgrL.transpose().format(CSVFormat)
                           << " , ";// left  grasping point
  dataLog_._OutRecord_pose << xgrR.transpose().format(CSVFormat) << " , " << qgrR.transpose().format(CSVFormat)
                           << " , ";// right grasping point
  dataLog_._OutRecord_pose << tossVar_.release_position.transpose().format(CSVFormat) << " , "
                           << tossVar_.release_orientation.transpose().format(CSVFormat) << " , ";// release pose
  dataLog_._OutRecord_pose << tossVar_.rest_position.transpose().format(CSVFormat) << " , "
                           << tossVar_.rest_orientation.transpose().format(CSVFormat) << " , ";// rest pose
  dataLog_._OutRecord_pose << target_._xt.transpose().format(CSVFormat) << " , "
                           << target_._qt.transpose().format(CSVFormat) << " , ";// target pose
  dataLog_._OutRecord_pose << target_._xd_landing.transpose().format(CSVFormat) << " , "
                           << target_._x_intercept.transpose().format(CSVFormat)
                           << " , ";// landing and intercept position
  dataLog_._OutRecord_pose << target_._xt_state2go.transpose().format(CSVFormat) << " , "
                           << xPlacing_.transpose().format(CSVFormat) << std::endl;// target state to go

  dataLog_._OutRecord_velo << (float) (cycleCount_ * dt_) << ", ";
  dataLog_._OutRecord_velo << robot_._Vd_ee[LEFT].transpose().format(CSVFormat) << " , "
                           << robot_._Vd_ee[RIGHT].transpose().format(CSVFormat) << " , ";
  dataLog_._OutRecord_velo << robot_._Vee[LEFT].transpose().format(CSVFormat) << " , "
                           << robot_._Vee[RIGHT].transpose().format(CSVFormat) << " , ";
  dataLog_._OutRecord_velo << robot_._vd[LEFT].transpose().format(CSVFormat) << " , "
                           << robot_._vd[RIGHT].transpose().format(CSVFormat) << " , ";
  dataLog_._OutRecord_velo << robot_._omegad[LEFT].transpose().format(CSVFormat) << " , "
                           << robot_._omegad[RIGHT].transpose().format(CSVFormat) << " , ";
  dataLog_._OutRecord_velo << object_._vo.transpose().format(CSVFormat) << " , "
                           << object_._wo.transpose().format(CSVFormat) << " , ";
  dataLog_._OutRecord_velo << objVelDes_.transpose().format(CSVFormat) << " , ";
  dataLog_._OutRecord_velo << tossVar_.release_linear_velocity.transpose().format(CSVFormat) << " , "
                           << tossVar_.release_angular_velocity.transpose().format(CSVFormat) << " , ";
  dataLog_._OutRecord_velo << target_._vt.transpose().format(CSVFormat) << std::endl;

  dataLog_._OutRecord_efforts << (float) (cycleCount_ * dt_) << ", ";
  dataLog_._OutRecord_efforts << robot_._filteredWrench[LEFT].transpose().format(CSVFormat) << " , ";
  dataLog_._OutRecord_efforts << robot_._filteredWrench[RIGHT].transpose().format(CSVFormat) << " , ";
  dataLog_._OutRecord_efforts << CooperativeCtrl._f_applied[LEFT].transpose().format(CSVFormat) << " , ";
  dataLog_._OutRecord_efforts << CooperativeCtrl._f_applied[RIGHT].transpose().format(CSVFormat) << std::endl;

  dataLog_._OutRecord_tasks << (float) (cycleCount_ * dt_) << ", ";
  dataLog_._OutRecord_tasks << desVimp_ << " , " << desVtoss_ << " , ";
  dataLog_._OutRecord_tasks << goHome_ << " , " << goToAttractors_ << " , " << releaseAndretract_ << " , "
                            << isThrowing_ << " , " << isPlacing_ << " , " << isContact_
                            << " , ";//CooperativeCtrl._ContactConfidence << " , ";
  dataLog_._OutRecord_tasks << freeMotionCtrl_.a_proximity_ << " , " << freeMotionCtrl_.a_normal_ << " , "
                            << freeMotionCtrl_.a_tangent_ << " , " << freeMotionCtrl_.a_release_ << " , "
                            << freeMotionCtrl_.a_retract_ << " , ";
  // dataLog_._OutRecord_tasks   	<< dsThrowing_.a_proximity_ << " , " << dsThrowing_.a_normal_  << " , " << dsThrowing_.a_tangent_<< " , " << dsThrowing_.a_toss_  << std::endl;
  dataLog_._OutRecord_tasks << dsThrowing_.a_proximity_ << " , " << dsThrowing_.a_normal_ << " , "
                            << dsThrowing_.a_tangent_ << " , " << dsThrowing_.a_toss_ << " , ";
  dataLog_._OutRecord_tasks << betaVelMod_ << " , " << dualPathLenAvgSpeed_.transpose() << std::endl;
  //
  dataLog_._OutRecord_jts_states << (float) (cycleCount_ * dt_) << ", ";
  dataLog_._OutRecord_jts_states << robot_._joints_positions[LEFT].transpose().format(CSVFormat) << " , "
                                 << robot_._joints_positions[RIGHT].transpose().format(CSVFormat) << " , ";
  dataLog_._OutRecord_jts_states << robot_._joints_velocities[LEFT].transpose().format(CSVFormat) << " , "
                                 << robot_._joints_velocities[RIGHT].transpose().format(CSVFormat) << " , ";
  dataLog_._OutRecord_jts_states << robot_._joints_accelerations[LEFT].transpose().format(CSVFormat) << " , "
                                 << robot_._joints_accelerations[RIGHT].transpose().format(CSVFormat) << " , ";
  dataLog_._OutRecord_jts_states << robot_._joints_torques[LEFT].transpose().format(CSVFormat) << " , "
                                 << robot_._joints_torques[RIGHT].transpose().format(CSVFormat) << " , ";
  dataLog_._OutRecord_jts_states << power_left(0, 0) << " , " << power_right(0, 0) << std::endl;
  // }
}

void dualArmControl::publish_conveyor_belt_cmds() {
  // desired conveyor belt contro mode
  std_msgs::Int32 _modeMessage;
  _modeMessage.data = modeConveyorBelt_;
  pubConveyorBeltMode_.publish(_modeMessage);
}
