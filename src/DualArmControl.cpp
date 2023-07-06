
#include "iam_dual_arm_control/DualArmControl.h"

using namespace std;
using namespace Eigen;

const static IOFormat CSVFormat(FullPrecision, DontAlignCols, ", ", "\n");

// ---------------------------------------------------------------------

DualArmControl::DualArmControl(ros::NodeHandle& n,
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
  desiredVelImp_ = 0.5f;
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

DualArmControl::~DualArmControl() {}

// ---- Init
bool DualArmControl::initRosSubscribers() {

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

  subObjectPose_ = nh_.subscribe(topicPoseObject_,
                                 1,
                                 &DualArmControl::objectPoseCallback,
                                 this,
                                 ros::TransportHints().reliable().tcpNoDelay());
  subTargetPose_ = nh_.subscribe(topic_pose_target_,
                                 1,
                                 &DualArmControl::targetPoseCallback,
                                 this,
                                 ros::TransportHints().reliable().tcpNoDelay());
  subBasePoseLeft_ =
      nh_.subscribe<geometry_msgs::Pose>(topicPoseRobotBase_[LEFT],
                                         1,
                                         boost::bind(&DualArmControl::updateBasePoseCallback, this, _1, LEFT),
                                         ros::VoidPtr(),
                                         ros::TransportHints().reliable().tcpNoDelay());
  subBasePoseRight_ =
      nh_.subscribe<geometry_msgs::Pose>(topicPoseRobotBase_[RIGHT],
                                         1,
                                         boost::bind(&DualArmControl::updateBasePoseCallback, this, _1, RIGHT),
                                         ros::VoidPtr(),
                                         ros::TransportHints().reliable().tcpNoDelay());
  subEEPoseLeft_ =
      nh_.subscribe<geometry_msgs::Pose>(topicPoseRobotEE_[LEFT],
                                         1,
                                         boost::bind(&DualArmControl::updateEEPoseCallback, this, _1, LEFT),
                                         ros::VoidPtr(),
                                         ros::TransportHints().reliable().tcpNoDelay());
  subEEPoseRight_ =
      nh_.subscribe<geometry_msgs::Pose>(topicPoseRobotEE_[RIGHT],
                                         1,
                                         boost::bind(&DualArmControl::updateEEPoseCallback, this, _1, RIGHT),
                                         ros::VoidPtr(),
                                         ros::TransportHints().reliable().tcpNoDelay());
  subEEVelLeft_ =
      nh_.subscribe<geometry_msgs::Twist>(topicSubEEVel[LEFT],
                                          1,
                                          boost::bind(&DualArmControl::updateEETwistCallback, this, _1, LEFT),
                                          ros::VoidPtr(),
                                          ros::TransportHints().reliable().tcpNoDelay());
  subEEVelRight_ =
      nh_.subscribe<geometry_msgs::Twist>(topicSubEEVel[RIGHT],
                                          1,
                                          boost::bind(&DualArmControl::updateEETwistCallback, this, _1, RIGHT),
                                          ros::VoidPtr(),
                                          ros::TransportHints().reliable().tcpNoDelay());
  subForceTorqueSensorLeft_ = nh_.subscribe<geometry_msgs::WrenchStamped>(
      topicFTSensor_[LEFT],
      1,
      boost::bind(&DualArmControl::updateRobotWrenchCallback, this, _1, LEFT),
      ros::VoidPtr(),
      ros::TransportHints().reliable().tcpNoDelay());
  subForceTorqueSensorRight_ = nh_.subscribe<geometry_msgs::WrenchStamped>(
      topicFTSensor_[RIGHT],
      1,
      boost::bind(&DualArmControl::updateRobotWrenchCallback, this, _1, RIGHT),
      ros::VoidPtr(),
      ros::TransportHints().reliable().tcpNoDelay());
  subJointStateLeft_ =
      nh_.subscribe<sensor_msgs::JointState>(topicSubJointState[LEFT],
                                             1,
                                             boost::bind(&DualArmControl::updateRobotStatesCallback, this, _1, LEFT),
                                             ros::VoidPtr(),
                                             ros::TransportHints().reliable().tcpNoDelay());
  subJointStateRight_ =
      nh_.subscribe<sensor_msgs::JointState>(topicSubJointState[RIGHT],
                                             1,
                                             boost::bind(&DualArmControl::updateRobotStatesCallback, this, _1, RIGHT),
                                             ros::VoidPtr(),
                                             ros::TransportHints().reliable().tcpNoDelay());

  return true;
}

bool DualArmControl::initRosPublisher() {
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

bool DualArmControl::initRobotParam() {

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

  // Savitzky-Golay Filter params
  int sgf_dq[3];
  sgf_dq[0] = 7;// dim
  sgf_dq[1] = 3;// order
  sgf_dq[2] = 6;// window length

  robot_.init(sgf_dq, dt_, gravity_);

  robot_.setInitParameters(paramToolMass, toolOffsetFromEE, toolComPositionFromSensor, xrbStandby, qrbStandby);

  return true;
}

bool DualArmControl::initObjectParam() {
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

  // Savitzky-Golay Filter params
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

  object_.init(sgfP, sgfO, dt_, oRGraspPosLeft, oRGraspPosRight);
  object_.setObjectMass(objectMassVect);
  object_.setObjectDim(Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(objectDimVect.data(), objectDimVect.size()));

  // Relative grasping positons
  object_.setXGpO(Eigen::Vector3f(0.0f, -object_.getObjectDimSpecific(1) / 2.0f, 0.0f) + graspOffsetLeft, 0);
  object_.setXGpO(Eigen::Vector3f(0.0f, object_.getObjectDimSpecific(1) / 2.0f, 0.0f) + graspOffsetRight, 1);

  return true;
}

bool DualArmControl::initFreeMotionCtrl() {
  bool modulatedReaching = true;
  bool isNormImpactVel = false;
  while (!nh_.getParam("dual_arm_task/reach_to_grasp/desVreach", desVreach_)) {
    ROS_INFO("Waitinng for param: reach_to_grasp/desVreach");
  }
  while (!nh_.getParam("dual_arm_task/modulated_reaching", modulatedReaching)) {
    ROS_INFO("Waitinng for param:  modulated_reaching");
  }
  while (!nh_.getParam("dual_arm_task/isNorm_impact_vel", isNormImpactVel)) {
    ROS_INFO("Waitinng for param:  isNorm_impact_vel");
  }
  while (!nh_.getParam("dual_arm_task/placing/height_via_point", heightViaPoint_)) {
    ROS_INFO("Waitinng for param: placing/height_via_point");
  }

  freeMotionCtrl_.init(robot_.getWHEEStandby(), this->gainAbs_, this->gainRel_);
  freeMotionCtrl_.setDt(dt_);
  freeMotionCtrl_.setObjectDim(object_.getObjectDim());
  freeMotionCtrl_.setDesVelReach(desVreach_);
  freeMotionCtrl_.setRefVelReach(refVreach_, LEFT);
  freeMotionCtrl_.setRefVelReach(refVreach_, RIGHT);
  freeMotionCtrl_.setModulatedReaching(modulatedReaching);
  freeMotionCtrl_.setIsNormImpactVel(isNormImpactVel);
  freeMotionCtrl_.setHeightViaPoint(heightViaPoint_);

  return true;
}

bool DualArmControl::initTossVar() {
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

  tossVar_.releasePosition =
      Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(releasePosVect.data(), releasePosVect.size());
  tossVar_.releaseOrientation =
      Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(releaseOrientVect.data(), releaseOrientVect.size());
  tossVar_.releaseLinearVelocity =
      Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(releaseLinVelDirVect.data(), releaseLinVelDirVect.size());
  tossVar_.releaseAngularVelocity =
      Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(releaseAngVelVect.data(), releaseAngVelVect.size());
  tossVar_.restPosition = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(restPosVect.data(), restPosVect.size());
  tossVar_.restOrientation =
      Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(restOrientVect.data(), restOrientVect.size());

  tossVar_.releaseLinearVelocity = desVtoss_ * tossVar_.releaseLinearVelocity.normalized();

  return true;
}

bool DualArmControl::initDesTasksPosAndLimits() {
  std::vector<float> paramAbsGains;
  std::vector<float> paramRelGains;
  std::vector<float> paramXDoLifting;
  std::vector<float> paramQDoLifting;
  std::vector<float> paramXDoPlacing;
  std::vector<float> paramQDoPlacing;
  std::vector<float> paramDualAngularLimit;

  while (!nh_.getParam("dual_arm_task/coordination/ds_absolute_gains", paramAbsGains)) {
    ROS_INFO("Waitinng for param: ds_absolute_gains ");
  }
  while (!nh_.getParam("dual_arm_task/coordination/ds_relative_gains", paramRelGains)) {
    ROS_INFO("Waitinng for param: ds_relative_gains ");
  }
  while (!nh_.getParam("dual_arm_task/lifting/position", paramXDoLifting)) {
    ROS_INFO("Waitinng for param: lifting/position");
  }
  while (!nh_.getParam("dual_arm_task/lifting/orientation", paramQDoLifting)) {
    ROS_INFO("Waitinng for param: lifting/orientation");
  }
  while (!nh_.getParam("dual_arm_task/placing/position", paramXDoPlacing)) {
    ROS_INFO("Waitinng for param: placing/position");
  }
  while (!nh_.getParam("dual_arm_task/placing/orientation", paramQDoPlacing)) {
    ROS_INFO("Waitinng for param: placing/orientation");
  }
  while (!nh_.getParam("dual_arm_task/tossing/dual_angular_limit", paramDualAngularLimit)) {
    ROS_INFO("Waitinng for param: tossing/param_dual_angular_limit ");
  }

  gainAbs_.diagonal() = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(paramAbsGains.data(), paramAbsGains.size());
  gainRel_.diagonal() = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(paramRelGains.data(), paramRelGains.size());
  xLifting_ = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(paramXDoLifting.data(), paramXDoLifting.size());
  qLifting_ = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(paramQDoLifting.data(), paramQDoLifting.size());
  xPlacing_ = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(paramXDoPlacing.data(), paramXDoPlacing.size());
  qPlacing_ = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(paramQDoPlacing.data(), paramQDoPlacing.size());
  dualAngularLimit_ =
      Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(paramDualAngularLimit.data(), paramDualAngularLimit.size());

  Eigen::Matrix3f RDo_lifting = Utils<float>::quaternionToRotationMatrix(qLifting_);
  filtDeltaAng_ = Utils<float>::getEulerAnglesXYZ_FixedFrame(RDo_lifting);
  filtDeltaAngMir_ = Utils<float>::getEulerAnglesXYZ_FixedFrame(RDo_lifting);

  return true;
}

bool DualArmControl::initDampingTopicCtrl() {

  // Get Passive DS params from iiwa_ros
  std::string paramDampingTopicCustomCtrlLeft;
  std::string paramDampingTopicCustomCtrlRight;
  std::string paramDampingTopicTorqueCtrlLeft;
  std::string paramDampingTopicTorqueCtrlRight;

  while (!nh_.getParam("dual_system/passiveDS/dampingTopic/CustomController/left", paramDampingTopicCustomCtrlLeft)) {
    ROS_INFO("Waitinng for param : CustomController/left");
  }
  while (!nh_.getParam("dual_system/passiveDS/dampingTopic/CustomController/right", paramDampingTopicCustomCtrlRight)) {
    ROS_INFO("Waitinng for param : CustomController/right");
  }
  while (!nh_.getParam("dual_system/passiveDS/dampingTopic/TorqueController/left", paramDampingTopicTorqueCtrlLeft)) {
    ROS_INFO("Waitinng for param : TorqueController/left");
  }
  while (!nh_.getParam("dual_system/passiveDS/dampingTopic/TorqueController/right", paramDampingTopicTorqueCtrlRight)) {
    ROS_INFO("Waitinng for param : TorqueController/right");
  }

  std::vector<float> paramDampLeft;
  std::vector<float> paramDampRight;

  ros::param::getCached(paramDampingTopicTorqueCtrlLeft, paramDampLeft);
  ros::param::getCached(paramDampingTopicTorqueCtrlRight, paramDampRight);

  if ((!paramDampLeft.empty()) && (!paramDampRight.empty())) {
    dsDampingTopicParams_[LEFT] = paramDampingTopicTorqueCtrlLeft;
    dsDampingTopicParams_[RIGHT] = paramDampingTopicTorqueCtrlRight;
  } else {
    dsDampingTopicParams_[LEFT] = paramDampingTopicCustomCtrlLeft;
    dsDampingTopicParams_[RIGHT] = paramDampingTopicCustomCtrlRight;
  }

  return true;
}

bool DualArmControl::initConveyorBelt() {
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

bool DualArmControl::initUserInteraction() {
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

bool DualArmControl::initTossParamEstimator() {
  std::string path2LearnedModelfolder = ros::package::getPath(std::string("dual_arm_control")) + "/LearnedModel/model1";
  std::string file_gmm[3];
  std::string dataType = "/throwingParam";

  file_gmm[0] = path2LearnedModelfolder + dataType + "_prio.txt";
  file_gmm[1] = path2LearnedModelfolder + dataType + "_mu.txt";
  file_gmm[2] = path2LearnedModelfolder + dataType + "_sigma.txt";

  tossParamEstimator_.init(file_gmm,
                           tossVar_.releasePosition,
                           tossVar_.releaseOrientation,
                           tossVar_.releaseLinearVelocity,
                           tossVar_.releaseAngularVelocity);

  target_.setXdLanding(Eigen::Vector3f(1.0f, 0.0f, 0.0f));

  tossParamEstimator_.estimateTossingParam(TossTaskParamEstimator::PHYS_IDEAL,
                                           target_.getXdLanding(),
                                           tossVar_.releasePosition);

  return true;
}

bool DualArmControl::initDSThrowing() {

  dsThrowing_.init(dsThrowing_.getDsParam(),
                   tossVar_.releasePosition,
                   tossVar_.releaseOrientation,
                   tossVar_.releaseLinearVelocity,
                   tossVar_.releaseAngularVelocity,
                   tossVar_.restPosition,
                   tossVar_.restOrientation);
  // TODO  if statement?
  // IF AUTOMATICALLY DETERMINED (USING RELEASE POSE GENERATOR)
  // dsThrowing_.init(dsThrowing_.getDsParam(),
  // 								tossParamEstimator_.getReleasePosition(),
  // 								tossParamEstimator_.getReleaseOrientation(),
  // 								tossParamEstimator_.getReleaseLinearVelocity(),
  // 								tossParamEstimator_.getReleaseAngularVelocity(),
  // 								tossVar_.restPosition, tossVar_.restOrientation);
  // desVtoss_ = tossParamEstimator_.getReleaseLinearVelocity().norm();
  //
  tossVar_.releaseLinearVelocity = desVtoss_ * (tossVar_.releasePosition - object_.getXo()).normalized();

  dsThrowing_.setPickupObjectPose(object_.getXo(), object_.getQo());
  dsThrowing_.setTossLinearVelocity(tossVar_.releaseLinearVelocity);
  dsThrowing_.setRefVtoss(desiredVelImp_);

  return true;
}

bool DualArmControl::init() {

  topic_pose_target_ = "/simo_track/target_pose";

  while (!nh_.getParam("dual_system/simulation", isSimulation_)) {
    ROS_INFO("Waitinng for param: dual_system/simulation ");
  }

  while (!nh_.getParam("dual_arm_task/reach_to_grasp/impact/desVimp", desiredVelImp_)) {
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

  target_.init(3, 3, 10, dt_);

  // -------- Motion and Force generation: DS --------

  // Initialize the Free motion generator DS
  initFreeMotionCtrl();

  // Initialize the cooperative controller
  CooperativeCtrl.init();

  // Initialization of the toss task parameter estimator
  initTossParamEstimator();

  // Object tossing DS
  target_.setXdLanding(Eigen::Vector3f(1.0f, 0.0f, 0.0f));
  target_.setXIntercept(target_.getXdLanding());
  object_.setXPickup(object_.getXo());

  // Initialize throwing object
  initDSThrowing();
  dsThrowingEstim_ = dsThrowing_;
  freeMotionCtrlEstim_ = freeMotionCtrl_;

  // Data recording:
  dataLog_.init(ros::package::getPath(std::string("dual_arm_control")) + "/Data");

  if (nh_.ok()) {
    // Wait for poses being published
    ros::spinOnce();
    ROS_INFO("[DualArmControl]: The object grabbing node is ready.");
    return true;
  } else {
    ROS_ERROR("[DualArmControl]: The ros node has a problem.");
    return false;
  }
  return true;
}

// ---- RUN

void DualArmControl::run() {
  ROS_INFO("Running the DualArmControl");

  while (nh_.ok()) {
    auto start = std::chrono::high_resolution_clock::now();

    // Keyboard commands
    updateStatesMachines();

    // Get the first eigen value of the passive ds controller and its updated value
    getPassiveDSDamping();

    mutex.lock();
    // Update the poses of the robots and the object
    updatePoses();
    // Compute generated desired motion and forces
    computeCommands();
    // Publish the commands to be exectued
    publishCommands();
    // Publish data through topics for analysis
    publishData();
    // Log data
    if (startlogging_) { saveData(); }
    mutex.unlock();

    ros::spinOnce();
    loopRate_.sleep();
    cycleCount_++;

    // Estimation of the running period
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "GO releaseAndretract_ : " << releaseAndretract_ << std::endl;
  }

  // Send zero command
  for (int k = 0; k < NB_ROBOTS; k++) {
    robot_.setVDes(robot_.getVDes(k).setZero(), k);
    robot_.setOmegaDes(robot_.getOmegaDes(k).setZero(), k);
    robot_.getQdSpecific(k) = robot_.getQ(k);
  }
  publishCommands();

  ros::spinOnce();
  loopRate_.sleep();

  // Close the data logging files
  dataLog_.closeFiles();

  ros::shutdown();
}

// ---- Compute Commands

void DualArmControl::computeCommands() {
  // Update contact state
  updateContactState();

  // Self imposed limits on intercept region (placing on moving target)
  float betaVelModUnfiltered = 1.0f;

  bool isContact = true && sensedContact_ && CooperativeCtrl.getContactConfidence() == 1.0f;
  bool isPlacing = isPlacing_ || (dualTaskSelector_ == PICK_AND_PLACE);
  bool isThrowing = isThrowing_ || (dualTaskSelector_ == TOSSING) || (dualTaskSelector_ == PICK_AND_TOSS);
  bool isPlaceTossing = isPlaceTossing_ || (dualTaskSelector_ == PLACE_TOSSING);
  bool isClose2Release = (dsThrowing_.getActivationTangent() > 0.99f);

  bool placingDone = (releaseFlag_) || ((object_.getWHo().block<3, 1>(0, 3) - xPlacing_).norm() <= 0.08);//0.05
  bool placeTossingDone = (releaseFlag_)
      || (((object_.getWHo().block<3, 1>(0, 3) - tossVar_.releasePosition).norm() <= 0.07)
          || ((object_.getWHo().block<2, 1>(0, 3) - xPlacing_.head(2)).norm() <= 0.05));
  bool tossingDone =
      (releaseFlag_) || (((object_.getWHo().block<3, 1>(0, 3) - tossVar_.releasePosition).norm() <= 0.035));
  bool isForceDetected =
      (robot_.getNormalForceAverage(LEFT) > forceThreshold_ || robot_.getNormalForceAverage(RIGHT) > forceThreshold_);

  Vector6f vDesEE[NB_ROBOTS];
  // ---------- Intercept/ landing location ----------
  // Compute intercept position with yaw angle limits for throwing object
  Eigen::Vector3f interceptMin =
      this->computeInterceptWithTarget(target_.getXt(), target_.getVt(), -dualAngularLimit_(2));
  Eigen::Vector3f interceptMax =
      this->computeInterceptWithTarget(target_.getXt(), target_.getVt(), dualAngularLimit_(2));

  // Self imposed limits on intercept region (placing on moving target)
  float intercepLimits[4];
  intercepLimits[0] = 0.60f;          // x_min
  intercepLimits[1] = 0.75f;          // x_max
  intercepLimits[2] = interceptMin(1);// y_min
  intercepLimits[3] = interceptMax(1);// y_max

  // ---------- Application ----------
  if ((!releaseAndretract_) && (fmod(cycleCount_, 20) == 0)) {
    dualPathLenAvgSpeed_ = freeMotionCtrlEstim_.predictRobotTranslation(robot_.getWHEE(),
                                                                        object_.getWHGp(),
                                                                        robot_.getWHEEStandby(),
                                                                        object_.getWHo(),
                                                                        tossVar_.releasePosition,
                                                                        desVtoss_,
                                                                        0.05f,
                                                                        0.100f,
                                                                        initSpeedScaling_);
  }

  Eigen::Vector2f lengthPathAvgSpeedRobot = {dualPathLenAvgSpeed_(0), trackingFactor_ * dualPathLenAvgSpeed_(1)};
  Eigen::Vector2f lengthPathAvgSpeedTarget =
      tossParamEstimator_.estimateTargetSimplePathLengthAverageSpeed(target_.getXt(),
                                                                     target_.getXdLanding(),
                                                                     target_.getVt());

  float flyTimeObj = 0.200f;
  timeToInterceptTgt_ = 0.0f;
  timeToInterceptBot_ = 0.0f;

  // Determine the desired landing position
  this->findDesiredLandingPosition(isPlacing, isPlaceTossing, isThrowing);

  // Estimate the target state to go
  this->estimateTargetStateToGo(lengthPathAvgSpeedRobot, lengthPathAvgSpeedTarget, flyTimeObj);

  // Set at pickup instant
  if (!isPickupSet_ && !releaseAndretract_) {
    if (sensedContact_ && (CooperativeCtrl.getContactConfidence() == 1.0)) {
      // Update intercept (desired landing) position
      this->updateInterceptPosition(flyTimeObj, intercepLimits);
      // Determination of the release configuration
      this->findReleaseConfiguration();
      // Set the release state and the object pickup position
      this->setReleaseState();

      isPickupSet_ = true;
    } else {
      object_.setXPickup(object_.getXo());
      dsThrowing_.setPickupObjectPose(object_.getXPickup(), object_.getQo());
    }
  }

  // Adaptation of the desired motion
  this->computeAdaptationFactors(lengthPathAvgSpeedRobot, lengthPathAvgSpeedTarget, flyTimeObj);

  if (goHome_) {
    freeMotionCtrl_.computeAsyncMotion(robot_.getWHEE(),
                                       robot_.getWHEEStandby(),
                                       object_.getWHo(),
                                       robot_.getVelDesEE(),
                                       robot_.getQd(),
                                       true);

    objVelDes_ =
        dsThrowing_.apply(object_.getXo(), object_.getQo(), object_.getVo(), Eigen::Vector3f(0.0f, 0.0f, 0.0f));

    for (int i = 0; i < NB_ROBOTS; i++) {
      dirImp_[i] =
          this->getImpactDirection(objVelDes_.head(3), object_.getNormalVectSurfObjSpecific(i), frictionAngle_);
      vdImpact_[i] = desiredVelImp_ * dirImp_[i];

      // Orthogonal Basis of Modulated Dual-arm DS
      basisQ_[i] = Utils<float>::create3dOrthonormalMatrixFromVector(dirImp_[i]);
    }

    // Reset some controller variables
    this->resetVariables();

  } else {//  release_and_retract || release
    if (releaseAndretract_) {

      freeMotionCtrl_.computeReleaseAndRetractMotion(robot_.getWHEE(),
                                                     object_.getWHDgp(),
                                                     object_.getWHo(),
                                                     robot_.getVelDesEE(),
                                                     robot_.getQd(),
                                                     true);
      isThrowing_ = false;
      isPlacing_ = false;
      isPickupSet_ = false;
      isPlaceTossing_ = false;
      nuWr0_ = nuWr1_ = 0.0f;
      dsThrowing_.resetReleaseFlag();
      isIntercepting_ = false;
    } else if (isContact) {// Constraint motion phase (Cooperative control)
      objVelDes_ =
          dsThrowing_.apply(object_.getXo(), object_.getQo(), object_.getVo(), Eigen::Vector3f(0.0f, 0.0f, 0.0f));

      // Desired task position and orientation vectors
      Eigen::Vector3f xDesTask = xLifting_;
      Eigen::Vector4f qDesTask = qLifting_;
      if (isPlacing) {
        xDesTask = xPlacing_;
        qDesTask = qPlacing_;
      }
      if (isPlaceTossing) {
        xDesTask = xPlacing_;
        qDesTask = qPlacing_;
      }
      if (isThrowing) {
        xDesTask = tossVar_.releasePosition;
        qDesTask = tossVar_.releaseOrientation;
      }

      // Target to object Orientation Adaptation
      if (trackTargetRotation_) {
        this->mirrorTargetToObjectOrientation(target_.getQt(), qDesTask, dualAngularLimit_);
        dsThrowing_.setTossPose(tossVar_.releasePosition, qDesTask);
      }

      // Desired object pose
      Eigen::Matrix4f desObjPosHomogTransfo = Utils<float>::pose2HomoMx(xDesTask, qDesTask);
      object_.setWHDo(Utils<float>::pose2HomoMx(xDesTask, qDesTask));

      // Desired pose of the grasping points
      for (int k = 0; k < NB_ROBOTS; k++) {
        object_.setWHDgp(object_.getWHDo() * oHEE_[k], k);
        object_.getWHDgpSpecific(k).block(0, 0, 3, 3) = desObjPosHomogTransfo.block(0, 0, 3, 3)
            * Utils<float>::pose2HomoMx(object_.getXGpO(k), object_.getQGpO(k)).block(0, 0, 3, 3);
      }

      // Motion generation
      freeMotionCtrl_.dualArmMotion(robot_.getWHEE(),
                                    robot_.getVelEE(),
                                    object_.getWHDgp(),
                                    object_.getWHo(),
                                    object_.getWHDo(),
                                    objVelDes_,
                                    basisQ_,
                                    vdImpact_,
                                    false,
                                    dualTaskSelector_,
                                    robot_.getVelDesEE(),
                                    robot_.getQd(),
                                    releaseFlag_);

      // Release and Retract condition
      if ((isPlacing && placingDone) || (isPlaceTossing && placeTossingDone) || (isThrowing && tossingDone)) {
        releaseAndretract_ = true;
      }

    } else {// Unconstraint (Free) motion phase
      freeMotionCtrl_.setReachableP(
          (robot_.getWHEESpecific(LEFT)(0, 3) >= 0.72f || robot_.getWHEESpecific(RIGHT)(0, 3) >= 0.72f) ? 0.0f : 1.0f);

      if (false || oldDualMethod_) {

        freeMotionCtrl_.computeCoordinatedMotion2(robot_.getWHEE(),
                                                  object_.getWHGp(),
                                                  object_.getWHo(),
                                                  robot_.getVelDesEE(),
                                                  robot_.getQd(),
                                                  false);

        Eigen::Vector3f errPosAbs = object_.getWHo().block(0, 3, 3, 1) - Utils<float>::getAbs3D(robot_.getWHEE());
        Eigen::Vector3f objErrPosAbs = object_.getWHo().block<3, 3>(0, 0).transpose() * errPosAbs;
        Eigen::Vector3f objErrPosAbsParallel = Eigen::Vector3f(objErrPosAbs(0), 0.0f, objErrPosAbs(2));
        float cp_ap = Utils<float>::computeCouplingFactor(objErrPosAbsParallel, 50.0f, 0.17f, 1.0f, true);

        // Create impact at grabbing
        Vector6f* vDesEETest = robot_.getVelDesEE();
        vDesEE[LEFT].head(3) = vDesEE[LEFT].head(3) + dirImp_[LEFT] * cp_ap * desiredVelImp_;
        vDesEE[RIGHT].head(3) = vDesEE[RIGHT].head(3) + dirImp_[RIGHT] * cp_ap * desiredVelImp_;
      } else {

        freeMotionCtrl_.dualArmMotion(robot_.getWHEE(),
                                      robot_.getVelEE(),
                                      object_.getWHGp(),
                                      object_.getWHo(),
                                      object_.getWHDo(),
                                      objVelDes_,
                                      basisQ_,
                                      vdImpact_,
                                      false,
                                      0,
                                      robot_.getVelDesEE(),
                                      robot_.getQd(),
                                      releaseFlag_);
      }

      dsThrowing_.setRefVtoss(desiredVelImp_);

      // for data logging
      objVelDes_.setZero();

      if (freeMotionCtrl_.getActivationProximity() >= 0.2f) { betaVelModUnfilt_ = 1.0; }
    }

    if (isPlacing || isThrowing || isPlaceTossing) {
      // Force feedback to grab objects
      float gainForce = 0.02f;
      float absForceCorrection = nuWr0_ * gainForce * 0.5f
          * ((robot_.getFilteredWrench(LEFT).segment(0, 3) - CooperativeCtrl.getForceApplied(LEFT).head(3))
                 .dot(object_.getNormalVectSurfObjSpecific(LEFT))
             + (robot_.getFilteredWrench(RIGHT).segment(0, 3) - CooperativeCtrl.getForceApplied(RIGHT).head(3))
                   .dot(object_.getNormalVectSurfObjSpecific(RIGHT)));

      if (fabs(absForceCorrection) > 0.2f) {
        absForceCorrection = absForceCorrection / fabs(absForceCorrection) * 0.2f;
      }

      Vector6f* vDesEETest = robot_.getVelDesEE();
      vDesEE[LEFT].head(3) =
          vDesEE[LEFT].head(3) - 0.40 * absForceCorrection * object_.getNormalVectSurfObjSpecific(LEFT);
      vDesEE[RIGHT].head(3) =
          vDesEE[RIGHT].head(3) - 0.40 * absForceCorrection * object_.getNormalVectSurfObjSpecific(RIGHT);
    }

    // ---------- Adaptation ----------
    if (!isContact && (freeMotionCtrl_.getActivationProximity() >= 0.2f)) { betaVelModUnfiltered = 1.0; }

    float filBeta = 0.10;
    betaVelMod_ = (1.f - filBeta) * betaVelMod_ + filBeta * betaVelModUnfiltered;

    if ((target_.getVt().norm() >= 0.05 && (!releaseAndretract_) && (dsThrowing_.getActivationProximity() <= 0.99f))) {

      Vector6f* vDesEETest = robot_.getVelDesEE();

      vDesEE[LEFT].head(3) *=
          initSpeedScaling_ * ((float) adaptationActive_ * betaVelMod_ + (1. - (float) adaptationActive_));
      vDesEE[RIGHT].head(3) *=
          initSpeedScaling_ * ((float) adaptationActive_ * betaVelMod_ + (1. - (float) adaptationActive_));
    }

    // ---------- Compute the object's grasp points velocity ----------
    object_.getGraspPointVelocity();

    // ---------- Generate grasping force and apply it in velocity space ----------
    // Desired object's task wrench
    desiredObjectWrench_.head(3) = -12.64f * (object_.getVo() - freeMotionCtrl_.get_des_object_motion().head(3))
        - object_.getObjectMass() * gravity_;
    desiredObjectWrench_.tail(3) = -25.00f * (object_.getWo() - freeMotionCtrl_.get_des_object_motion().tail(3));

    CooperativeCtrl.getAppliedWrenches(goHome_,
                                       contactState_,
                                       object_.getWHo(),
                                       robot_.getWHEE(),
                                       object_.getWHGp(),
                                       desiredObjectWrench_,
                                       object_.getObjectMass(),
                                       qpWrenchGeneration_,
                                       isForceDetected);

    // Applied force in velocity space
    for (int i = 0; i < NB_ROBOTS; i++) {
      robot_.setFXC(1.0f / d1_[i] * CooperativeCtrl.getForceApplied(i).head(3), i);
    }
  }

  // Compute the velocity to avoid EE collision
  Vector6f vEEOA[NB_ROBOTS];
  vEEOA[0] = robot_.getVEEObstacleAvoidance(0);
  vEEOA[1] = robot_.getVEEObstacleAvoidance(1);
  freeMotionCtrl_.computeEEAvoidanceVelocity(robot_.getWHEE(), vEEOA);
  robot_.setVEEObstacleAvoidance(vEEOA);

  // Extract linear velocity commands and desired axis angle command
  this->prepareCommands(robot_.getVelDesEE(), robot_.getQd(), object_.getVGpO());

  // ---------- Control of conveyor belt speed ----------
  float omegaPert = 2.f * M_PI / 1;
  float deltaOmegaPert = 0.1f * (2.f * (float) std::rand() / RAND_MAX - 1.0f) * omegaPert;

  if (ctrlModeConveyorBelt_) {
    desSpeedConveyorBelt_ = (int) (nominalSpeedConveyorBelt_
                                   + (int) isDisturbTarget_ * magniturePertConveyorBelt_
                                       * sin((omegaPert + deltaOmegaPert) * dt_ * cycleCount_));
  }
}

// ---- Update

void DualArmControl::updateContactState() {
  robot_.getEstimatedAverageNormalForce();

  // Compute errors to object center position and dimension vector
  Eigen::Matrix4f leftEERightEE = robot_.getWHEESpecific(LEFT).inverse() * robot_.getWHEESpecific(RIGHT);
  Eigen::Matrix4f leftGripPoseRightGripPose = object_.getWHGpSpecific(LEFT).inverse() * object_.getWHGpSpecific(RIGHT);
  Eigen::Vector3f errorObjPosVect =
      Utils<float>::getAbs3D(object_.getWHGp()) - Utils<float>::getAbs3D(robot_.getWHEE());
  errorObjDim_ = fabs(leftEERightEE(2, 3)) - fabs(leftGripPoseRightGripPose(2, 3));
  errorObjPos_ = errorObjPosVect.norm();

  if ((robot_.getNormalForceAverage(LEFT) > 2.0f || robot_.getNormalForceAverage(RIGHT) > 2.0f) && errorObjDim_ < 0.065f
      && (errorObjPos_ < 0.065f || CooperativeCtrl.getContactConfidence() == 1.0f)) {
    contactState_ = CONTACT;
    isContact_ = 1.0f;
  } else if (!(robot_.getNormalForceAverage(LEFT) > 2.0f && robot_.getNormalForceAverage(RIGHT) > 2.0f)
             && errorObjDim_ < 0.05f && errorObjPos_ < 0.05f) {
    contactState_ = CLOSE_TO_CONTACT;
    isContact_ = 0.0f;
  } else {
    contactState_ = NO_CONTACT;
    isContact_ = 0.0f;
  }

  // Check contact
  sensedContact_ = ((fabs(robot_.getNormalForce(LEFT)) >= forceThreshold_)
                    || (fabs(robot_.getNormalForce(RIGHT)) >= forceThreshold_))
      && (isContact_ == 1.0f);
}

void DualArmControl::updateStatesMachines() {
  keyboard::nonBlock(1);

  if (keyboard::khBit() != 0) {
    char keyboardCommand = fgetc(stdin);
    fflush(stdin);

    switch (keyboardCommand) {
      case 'q': {
        goHome_ = !goHome_;
        if (goHome_) {
          goToAttractors_ = true;
          startlogging_ = false;
        } else if (!goHome_) {
          startlogging_ = true;
        }
      } break;
      case 'g': {
        goToAttractors_ = !goToAttractors_;
        if (goToAttractors_) {
          goHome_ = false;
          releaseAndretract_ = false;
        }
      } break;

      // Conveyor belt control
      case 'a': {
        if (ctrlModeConveyorBelt_) {
          modeConveyorBelt_ = 2;
          publishConveyorBeltCmds();
          startlogging_ = true;
        } else if (incrementReleasePos_) {
          deltaRelPos_(0) -= 0.025f;//[m]
        } else {
          deltaPos_(0) -= 0.01f;
        }
      } break;
      case 's': {
        if (ctrlModeConveyorBelt_) {
          modeConveyorBelt_ = 0;
          publishConveyorBeltCmds();
        } else if (incrementReleasePos_) {
          deltaRelPos_(0) += 0.025f;//[m]
        } else {
          deltaPos_(0) += 0.01f;
        }
      } break;
      case 'd': {
        if (ctrlModeConveyorBelt_) {
          modeConveyorBelt_ = 1;
          publishConveyorBeltCmds();
        } else if (incrementReleasePos_) {
          deltaRelPos_(1) -= 5.0f;//[deg]
        } else {
          deltaPos_(1) -= 0.01f;
        }
      } break;
      case 'f': {
        if (incrementReleasePos_) {
          deltaRelPos_(1) += 5.0f;//[deg]
        } else {
          deltaPos_(1) += 0.01f;
        }
      } break;
      case 'z': {
        if (ctrlModeConveyorBelt_) {
          trackingFactor_ -= 0.01f;
        } else if (incrementReleasePos_) {
          deltaRelPos_(2) -= 5.0f;//[deg]
        } else {
          deltaPos_(2) -= 0.01f;
        }
      } break;
      case 'w': {
        if (ctrlModeConveyorBelt_) {
          trackingFactor_ += 0.01f;
        } else if (incrementReleasePos_) {
          deltaRelPos_(2) += 5.0f;//[deg]
        } else {
          deltaPos_(2) += 0.01f;
        }
      } break;
      case 'h': {
        if (ctrlModeConveyorBelt_) {
          nominalSpeedConveyorBelt_ -= 50;
        } else {
          deltaAng_(0) -= 0.05f;
        }
      } break;
      case 'j': {
        if (ctrlModeConveyorBelt_) {
          nominalSpeedConveyorBelt_ += 50;
        } else {
          deltaAng_(0) += 0.05f;
        }
      } break;
      case 'k': {
        if (ctrlModeConveyorBelt_) {
          adaptationActive_ = !adaptationActive_;
        } else {
          deltaAng_(1) -= 0.05f;
        }
      } break;
      case 'm': {
        if (ctrlModeConveyorBelt_) {
          magniturePertConveyorBelt_ -= 50;
        } else {
          deltaAng_(2) -= 0.05f;
        }
      } break;
      case 'i': {
        if (ctrlModeConveyorBelt_) {
          magniturePertConveyorBelt_ += 50;
        } else {
          deltaAng_(2) += 0.05f;
        }
      } break;

      // Release or throwing
      case 'r': {
        releaseAndretract_ = !releaseAndretract_;
      } break;
      case 'l': {
        dualTaskSelector_ = PICK_AND_LIFT;
        hasCaughtOnce_ = false;
      } break;
      case 't': {
        isThrowing_ = !isThrowing_;
        if (isThrowing_) {
          dualTaskSelector_ = PICK_AND_TOSS;
          hasCaughtOnce_ = false;
        } else if (!isThrowing_) {
          dualTaskSelector_ = PICK_AND_LIFT;
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

      // Impact and tossing velocity
      case 'v': {
        desVtoss_ -= 0.05f;
        if (desVtoss_ < 0.2f) { desVtoss_ = 0.2f; }
        dsThrowing_.setTossLinearVelocity(desVtoss_ * tossVar_.releaseLinearVelocity.normalized());
        dsThrowingEstim_.setTossLinearVelocity(desVtoss_ * tossVar_.releaseLinearVelocity.normalized());
      } break;
      case 'b': {
        desVtoss_ += 0.05f;
        if (desVtoss_ > 2.0f) { desVtoss_ = 2.0f; }
        dsThrowing_.setTossLinearVelocity(desVtoss_ * tossVar_.releaseLinearVelocity.normalized());
        dsThrowingEstim_.setTossLinearVelocity(desVtoss_ * tossVar_.releaseLinearVelocity.normalized());
      } break;
      case 'y': {
        desiredVelImp_ -= 0.05f;
        if (desiredVelImp_ < 0.05f) { desiredVelImp_ = 0.05f; }
      } break;
      case 'u': {
        desiredVelImp_ += 0.05f;
        if (desiredVelImp_ > 0.6f) { desiredVelImp_ = 0.6f; }
      } break;

      // Reset the data logging
      case 'c': {
        startlogging_ = false;
        dataLog_.reset(ros::package::getPath(std::string("dual_arm_control")) + "/Data");
      } break;

      // Disturb the target speed
      case 'e': {
        isDisturbTarget_ = !isDisturbTarget_;
      } break;

      // Placing hight
      case 'x': {
        if (dualTaskSelector_ == PICK_AND_TOSS) {
          tossVar_.releasePosition(1) -= 0.01;
        } else {
          xPlacing_(2) -= 0.01;
        }
      } break;
      case 'n': {
        if (dualTaskSelector_ == PICK_AND_TOSS) {
          tossVar_.releasePosition(1) += 0.01;
        } else {
          xPlacing_(2) += 0.01;
        }
      } break;
    }
  }
  keyboard::nonBlock(0);
}

void DualArmControl::updatePoses() {
  if (initPoseCount_ < 100) {
    // Get stanby transformation of the EEs wrt. the world frame
    robot_.getStandbyHmgTransformInWorld();
    freeMotionCtrl_.setWHEEStandby(robot_.getWHEEStandbySpecific(LEFT), LEFT);
    freeMotionCtrl_.setWHEEStandby(robot_.getWHEEStandbySpecific(RIGHT), RIGHT);

    // Set attractor of lifting task
    object_.setXDo(Eigen::Vector3f(object_.getXoSpecific(0), object_.getXoSpecific(1), xLifting_(2)));
    object_.setQDo(object_.getQo());
    object_.getDesiredHmgTransform();

    target_.setXIntercept(Eigen::Vector3f(object_.getXoSpecific(0), 0.0, object_.getXoSpecific(2)));
    // For catching
    freeMotionCtrl_.setVirtualObjectFrame(Utils<float>::pose2HomoMx(target_.getXIntercept(), object_.getQo()));

    initPoseCount_++;
  }

  // Update the object position or its desired position (attractor) through keyboard
  if (objCtrlKey_) {
    this->keyboardVirtualObjectControl();
  } else {
    this->keyboardReferenceObjectControl();
  }

  if (incrementReleasePos_) { this->updateReleasePosition(); }

  // Homogeneous transformations associated with the reaching task
  robot_.getEndEffectorHmgTransform();
  object_.getGraspPointHTransform();
  object_.getGraspPointDesiredHTransform();
  object_.updateGraspNormals();

  for (int k = 0; k < NB_ROBOTS; k++) {
    err_[k] = (robot_.getWHEESpecific(k).block(0, 3, 3, 1) - object_.getWHGpSpecific(k).block(0, 3, 3, 1)).norm();
    oHEE_[k] = object_.getWHo().inverse() * robot_.getWHEESpecific(k);

    if (CooperativeCtrl.getContactConfidence() == 1.0) {
      oHEE_[k](1, 3) *= 0.95f;
      object_.setWHDgp(object_.getWHDo() * oHEE_[k], k);
    }
  }
}

void DualArmControl::getPassiveDSDamping() {
  std::vector<float> paramValues;

  ros::param::getCached(dsDampingTopicParams_[LEFT], paramValues);
  ros::param::getCached(dsDampingTopicParams_[RIGHT], paramValues);

  d1_[LEFT] = paramValues[0];
  if (d1_[LEFT] < FLT_EPSILON) { d1_[LEFT] = 150.0f; }

  d1_[RIGHT] = paramValues[0];
  if (d1_[RIGHT] < FLT_EPSILON) { d1_[RIGHT] = 150.0f; }
}

void DualArmControl::prepareCommands(Vector6f vDesEE[], Eigen::Vector4f qd[], Vector6f velGraspPos[]) {
  Eigen::Matrix<float, 3, 1> axisDes[NB_ROBOTS];
  float angleDes[NB_ROBOTS];

  for (int i = 0; i < NB_ROBOTS; i++) {
    Vector6f VdEE = robot_.getTwistEEToolCenterPoint(i).inverse() * (vDesEE[i] + velGraspPos[i]);
    robot_.setVDes(VdEE.head(3) + 1.0 * robot_.getVEEObstacleAvoidance(i).head(3), i);
    robot_.setOmegaDes(VdEE.tail(3), i);

    Utils<float>::quaternionToAxisAngle(qd[i], axisDes[i], angleDes[i]);
    robot_.setAxisAngleDes(angleDes[i] * axisDes[i], i);
  }

  if (goToAttractors_ && sensedContact_ && CooperativeCtrl.getContactConfidence() == 1.0f) {
    nuWr0_ = 0.80f * nuWr0_ + 0.20f;
    nuWr1_ = 0.92f * nuWr1_ + 0.08f;
  } else {
    nuWr0_ = 0.0f;
    nuWr1_ = 0.0f;
  }

  if (releaseAndretract_) {
    robot_.setFXC(robot_.getFXC(LEFT).setZero(), LEFT);
    robot_.setFXC(robot_.getFXC(RIGHT).setZero(), RIGHT);
    nuWr0_ = 0.0f;
    nuWr1_ = 0.0f;
  }

  if (goToAttractors_) {
    applyVelo_ = 1.f;
  } else {
    applyVelo_ = 0.0f;

    // Keep the current orientation if not going to the attractor
    robot_.getQdSpecific(LEFT) = robot_.getQ(LEFT);
    robot_.getQdSpecific(RIGHT) = robot_.getQ(RIGHT);
  }

  // Set the command to send
  robot_.getDesiredLinTaskVelocity(applyVelo_, nuWr0_);
}

void DualArmControl::mirrorTargetToObjectOrientation(Eigen::Vector4f qt,
                                                     Eigen::Vector4f& qo,
                                                     Eigen::Vector3f angleLimit) {
  Eigen::Vector3f eulerAngleTarget =
      Utils<float>::getEulerAnglesXYZ_FixedFrame(Utils<float>::quaternionToRotationMatrix(qt));
  Eigen::Vector3f eulerAngleDesired =
      Utils<float>::getEulerAnglesXYZ_FixedFrame(Utils<float>::quaternionToRotationMatrix(qo));

  filtDeltaAngMir_ = 0.95 * filtDeltaAngMir_ + 0.05 * (eulerAngleTarget - eulerAngleDesired);
  eulerAngleTarget(0) = eulerAngleDesired(0) + filtDeltaAngMir_(0);
  eulerAngleTarget(2) = eulerAngleDesired(2) + filtDeltaAngMir_(2);

  if (eulerAngleTarget(0) >= angleLimit(0)) {
    eulerAngleTarget(0) = angleLimit(0);
  } else if (eulerAngleTarget(0) <= -angleLimit(0)) {
    eulerAngleTarget(0) = -angleLimit(0);
  }

  if (eulerAngleTarget(2) >= angleLimit(2)) {
    eulerAngleTarget(2) = angleLimit(2);
  } else if (eulerAngleTarget(2) <= -angleLimit(2)) {
    eulerAngleTarget(2) = -angleLimit(2);
  }

  qo = Utils<float>::rotationMatrixToQuaternion(
      Utils<float>::eulerAnglesToRotationMatrix(eulerAngleDesired(0), eulerAngleDesired(1), eulerAngleTarget(2)));
}

// control of object position through keyboad
void DualArmControl::keyboardVirtualObjectControl() {
  Eigen::Matrix4f wHo;
  wHo = object_.getWHo();
  wHo(0, 3) += deltaPos_(0);
  object_.setWHo(wHo);

  wHo = object_.getWHo();
  wHo(1, 3) += deltaPos_(1);
  object_.setWHo(wHo);

  wHo = object_.getWHo();
  wHo(2, 3) += deltaPos_(2);
  object_.setWHo(wHo);
}

// control of attractor position through keyboad
void DualArmControl::keyboardReferenceObjectControl() {
  xLifting_(0) += deltaPos_(0);
  xLifting_(1) += deltaPos_(1);
  xLifting_(2) += deltaPos_(2);
  deltaPos_.setZero();
}

Eigen::Vector3f
DualArmControl::getImpactDirection(Eigen::Vector3f objectDesiredForce, Eigen::Vector3f objNormal, float coeffFriction) {
  float theta = 0.0f;
  Eigen::Matrix3f R1;
  Eigen::Matrix3f R0;
  Utils<float>::Orthobasis(objectDesiredForce, objNormal, R1, R0);

  if (this->impactDirPreset_) {
    theta = M_PI / 180.f * coeffFriction;
  } else {
    float vzo = objectDesiredForce.transpose() * R0.col(2);
    float vxo = objectDesiredForce.transpose() * objNormal;
    theta = std::atan2(vzo, vxo);
  }

  if (fabs(theta) >= M_PI / 180.f * this->frictionAngleMax_) {
    theta = theta / fabs(theta) * M_PI / 180.f * frictionAngleMax_;
  }

  Eigen::Vector3f impactDir = R0 * Eigen::Vector3f(std::cos(theta), 0.0f, std::sin(theta));

  return impactDir.normalized();
}

void DualArmControl::resetVariables() {

  releaseAndretract_ = false;
  isThrowing_ = false;
  isPlacing_ = false;
  isPlaceTossing_ = false;
  isPickupSet_ = false;
  isIntercepting_ = false;
  nuWr0_ = nuWr1_ = 0.0f;
  this->refVreach_ = 0.0f;

  freeMotionCtrl_.setRefVelReach(0.0f, LEFT);
  freeMotionCtrl_.setRefVelReach(0.0f, RIGHT);
  dsThrowing_.setRefVtoss(desiredVelImp_);
  dsThrowing_.resetReleaseFlag();
  objVelDes_.setZero();

  Vector6f* newVGpO = object_.getVGpO();
  for (int i = 0; i < NB_ROBOTS; i++) {
    Eigen::Vector3f newFXC = robot_.getFXC(i);
    object_.setVGpO(newVGpO[i].setZero(), i);
    robot_.setFXC(newFXC.setZero(), i);
  }

  if (adaptationActive_) {
    tossVar_.releasePosition(1) = 0.0f;
    xPlacing_(1) = 0.0f;// TBC !!!! TODO
  }
}

void DualArmControl::updateReleasePosition() {
  releasePos_.r += deltaRelPos_(0);
  releasePos_.theta += M_PI / 180.0f * deltaRelPos_(1);
  releasePos_.phi += M_PI / 180.0f * deltaRelPos_(2);
  deltaRelPos_.setZero();

  Eigen::Vector3f posXo;
  releasePos_.toCartesian(posXo);
  tossVar_.releasePosition = posXo + xLifting_;

  if (tossVar_.releasePosition(0) > 0.70) { tossVar_.releasePosition(0) = 0.70; }
}

void DualArmControl::set2DPositionBoxConstraints(Eigen::Vector3f& position_Vect, float limits[]) {
  if (position_Vect(0) < limits[0]) position_Vect(0) = limits[0];// x_min
  if (position_Vect(0) > limits[1]) position_Vect(0) = limits[1];// x_max
  if (position_Vect(1) < limits[2]) position_Vect(1) = limits[2];// y_min
  if (position_Vect(1) > limits[3]) position_Vect(1) = limits[3];// y_max
}

Eigen::Vector3f DualArmControl::computeInterceptWithTarget(const Eigen::Vector3f& xTarget,
                                                           const Eigen::Vector3f& vTarget,
                                                           float phiInit) {
  float phiTarget = 0.0f;
  if (vTarget.head(2).norm() > 1e-2) { phiTarget = std::atan2(vTarget(1), vTarget(0)); }

  float phiTargetTangent = std::tan(phiTarget);
  if (phiTargetTangent > 1e4) { phiTargetTangent = 1e4; }
  if (-phiTargetTangent < -1e4) { phiTargetTangent = -1e4; }

  Eigen::Vector3f xIntercept = xTarget;

  float xCoordLand = xTarget(0);
  float yCoordLand = xTarget(1);
  float phiInitTangent = std::tan(phiInit);

  if (vTarget.head(2).norm() > 1e-2) {
    float phiConveyor = std::atan2(vTarget(1), vTarget(0));
    float phiConveyorTangent = std::tan(phiConveyor);

    xCoordLand = ((phiConveyorTangent * xTarget(0) - xTarget(1)) / (phiConveyorTangent - phiInitTangent));
  } else {
    xCoordLand = xTarget(0);
  }
  yCoordLand = xCoordLand * phiInitTangent;

  xIntercept << xCoordLand, yCoordLand, xTarget(2);

  return xIntercept;
}

float DualArmControl::getDesiredYawAngleTarget(const Eigen::Vector4f& qt, const Eigen::Vector3f& angLimit) {
  Eigen::Vector3f eulerAngTarget =
      Utils<float>::getEulerAnglesXYZ_FixedFrame(Utils<float>::quaternionToRotationMatrix(qt));

  float phiTargetRot = eulerAngTarget(2);

  if (phiTargetRot >= angLimit(2)) {
    phiTargetRot = angLimit(2);
  } else if (phiTargetRot <= -angLimit(2)) {
    phiTargetRot = -angLimit(2);
  }

  return phiTargetRot;
}

void DualArmControl::findDesiredLandingPosition(bool isPlacing, bool isPlaceTossing, bool isThrowing) {
  Eigen::Vector3f xDesiredLand = target_.getXt();
  float phiThrowing = 0.0;

  // Determine the intercept or desired landing position
  if (userSelect_) {
    if (isPlacing || isPlaceTossing) { xDesiredLand.head(2) = xPlacing_.head(2); }
    if (isThrowing) { xDesiredLand.head(2) = tossVar_.releasePosition.head(2); }
    phiThrowing = std::atan2(xDesiredLand(1), xDesiredLand(0));

    if (isTargetFixed_) {
      target_.setXdLanding(xDesiredLand);
    } else {
      target_.setXdLanding(this->computeInterceptWithTarget(target_.getXt(), target_.getVt(), phiThrowing));
    }
  } else {
    phiThrowing = this->getDesiredYawAngleTarget(target_.getQt(), dualAngularLimit_);

    target_.setXdLanding(this->computeInterceptWithTarget(target_.getXt(), target_.getVt(), phiThrowing));
  }
}

void DualArmControl::updateInterceptPosition(float flyTimeObj, float intercepLimits[]) {

  Eigen::Vector3f xTargetIntercept = target_.getXt() + target_.getVt() * (timeToInterceptBot_ + flyTimeObj);

  // Apply constraints
  this->set2DPositionBoxConstraints(xTargetIntercept, intercepLimits);

  if (adaptationActive_) {
    Eigen::Vector3f newXtLanding = target_.getXdLanding();
    newXtLanding.head(2) = xTargetIntercept.head(2);
    target_.setXdLanding(newXtLanding);
    if (isPlaceTossing_ || (dualTaskSelector_ == PLACE_TOSSING)) {
      xPlacing_.head(2) = target_.getXt().head(2) + 0.35 * target_.getVt().head(2) * (timeToInterceptBot_ + flyTimeObj);
    }
  }
}

void DualArmControl::findReleaseConfiguration() {
  // Basic release configuration
  if (feasibleAlgo_) {
    // Generate using feasibilty algorithm
    tossVar_.releasePosition.head(2) = target_.getXdLanding().head(2);
  } else if (pickupBased_) {
    Eigen::Vector3f xReleaseBar = tossVar_.releasePosition - object_.getXPickup();
    float phiThrowBar = std::atan2(xReleaseBar(1), xReleaseBar(0));

    tossVar_.releaseLinearVelocity << tossVar_.releaseLinearVelocity.head(2).norm() * std::cos(phiThrowBar),
        tossVar_.releaseLinearVelocity.head(2).norm() * std::sin(phiThrowBar), tossVar_.releaseLinearVelocity(2);
    tossVar_.releaseLinearVelocity = desVtoss_ * tossVar_.releaseLinearVelocity.normalized();
  } else {
    // user-defined
    tossVar_.releasePosition = tossVar_.releasePosition;
    tossVar_.releaseLinearVelocity = desVtoss_ * tossVar_.releaseLinearVelocity.normalized();
  }
}

void DualArmControl::setReleaseState() {
  // Set the release state and the object pickup position
  dsThrowing_.setTossPose(tossVar_.releasePosition, tossVar_.releaseOrientation);
  dsThrowing_.setTossLinearVelocity(tossVar_.releaseLinearVelocity);
  dsThrowing_.setPickupObjectPose(object_.getXPickup(), object_.getQo());
}

void DualArmControl::estimateTargetStateToGo(Eigen::Vector2f lengthPathAvgSpeedRobot,
                                             Eigen::Vector2f lengthPathAvgSpeedTarget,
                                             float flyTimeObj) {
  // Estimation of the target state-to-go
  target_.setXtStateToGo(tossParamEstimator_.estimateTargetStateToGo(target_.getVt(),
                                                                     target_.getXdLanding(),
                                                                     lengthPathAvgSpeedRobot,
                                                                     lengthPathAvgSpeedTarget,
                                                                     flyTimeObj));

  // Boolean robot's motion trigger
  Eigen::Vector3f xtBar = target_.getXt() - target_.getXdLanding();
  Eigen::Vector3f xtToGoBar = target_.getXtStateToGo() - target_.getXdLanding();
  isMotionTriggered_ =
      (-xtBar.dot(target_.getVt()) > 0) && ((initPoseCount_ > 50) && ((xtBar - xtToGoBar).norm() < 0.04f));
}

void DualArmControl::computeAdaptationFactors(Eigen::Vector2f lengthPathAvgSpeedRobot,
                                              Eigen::Vector2f lengthPathAvgSpeedTarget,
                                              float flyTimeObj) {
  if (isMotionTriggered_) { isIntercepting_ = true; }

  float beta_vel_mod_max = min(2.0f,
                               min((vMax_ / robot_.getVelDesEESpecific(LEFT).head(3).norm()),
                                   (vMax_ / robot_.getVelDesEESpecific(RIGHT).head(3).norm())));

  if (isIntercepting_ && !releaseAndretract_) {
    float epsilon = 1e-6;
    timeToInterceptTgt_ = fabs(fabs(lengthPathAvgSpeedTarget(0) + epsilon - lengthPathAvgSpeedTarget(1) * flyTimeObj)
                               / (lengthPathAvgSpeedTarget(1) + epsilon));
    timeToInterceptBot_ = lengthPathAvgSpeedRobot(0) / lengthPathAvgSpeedRobot(1);

    // Velocity-based adaptation factor
    if (isRatioFactor_) {
      betaVelModUnfilt_ =
          fabs((lengthPathAvgSpeedTarget(1) / (lengthPathAvgSpeedRobot(1) + epsilon))
               * (lengthPathAvgSpeedRobot(0)
                  / fabs(lengthPathAvgSpeedTarget(0) + epsilon - lengthPathAvgSpeedTarget(1) * flyTimeObj)));
    } else {
      betaVelModUnfilt_ = (std::tanh(7.0f * (timeToInterceptBot_ - timeToInterceptTgt_)) + 1.0);
    }

    if (betaVelModUnfilt_ >= beta_vel_mod_max) { betaVelModUnfilt_ = beta_vel_mod_max; }

    // Attractor-based adaptation factor
    freeMotionCtrl_.setActivationAperture(
        adaptationActive_ ? 0.5f
                * (std::tanh(
                       switchSlopeAdapt_
                       * ((target_.getXdLanding() - target_.getXt()).normalized().dot(target_.getVt()) - tolAttractor_))
                   + 1.0)
                          : 1.0f);
  } else {
    betaVelModUnfilt_ = 1.0f;
    freeMotionCtrl_.setActivationAperture(1.0f);
  }
}

// ---- Callback functions

void DualArmControl::objectPoseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
  Eigen::Vector3f xom, tXoXom;
  Eigen::Vector4f newQo;
  if (!isSimulation_) {
    tXoXom << 0.0f, 0.0f, -object_.getObjectDimSpecific(2) / 2.0f;
  } else {
    tXoXom << 0.0f, 0.0f, 0.0f;
  }
  xom << msg->position.x, msg->position.y, msg->position.z;
  newQo << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
  object_.setQo(newQo);

  Eigen::Matrix3f w_R_o = Utils<float>::quaternionToRotationMatrix(object_.getQo());
  object_.setXo(xom + w_R_o * tXoXom);
  object_.getHmgTransform();
}

void DualArmControl::targetPoseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
  Eigen::Vector3f newXt;
  newXt << msg->position.x, msg->position.y, msg->position.z;
  target_.setXt(newXt);

  Eigen::Vector4f newQt;
  newQt << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
  target_.setQt(newQt);

  // filtered object position
  target_.getFilteredState();
}

void DualArmControl::updateBasePoseCallback(const geometry_msgs::Pose::ConstPtr& msg, int k) {
  Eigen::Vector3f xB = Eigen::Vector3f(msg->position.x, msg->position.y, msg->position.z);
  Eigen::Vector4f q = Eigen::Vector4f(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  robot_.getRobotBaseFrameInWorld(xB, q, k);
}

void DualArmControl::updateEEPoseCallback(const geometry_msgs::Pose::ConstPtr& msg, int k) {
  // Update end effecotr pose (position+orientation)
  Eigen::Vector3f xB = Eigen::Vector3f(msg->position.x, msg->position.y, msg->position.z);
  Eigen::Vector4f q = Eigen::Vector4f(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  robot_.updateEndEffectorPosesInWorld(xB, q, k);
}

void DualArmControl::updateEETwistCallback(const geometry_msgs::Twist::ConstPtr& msg, int k) {
  Eigen::Vector3f vE = Eigen::Vector3f(msg->linear.x, msg->linear.y, msg->linear.z);
  Eigen::Vector3f wE = Eigen::Vector3f(msg->angular.x, msg->angular.y, msg->angular.z);
  robot_.update_EndEffectorVelocity(vE, wE, k);
}

void DualArmControl::updateRobotWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg, int k) {
  Eigen::Matrix<float, 6, 1> raw;
  raw(0) = msg->wrench.force.x;
  raw(1) = msg->wrench.force.y;
  raw(2) = msg->wrench.force.z;
  raw(3) = msg->wrench.torque.x;
  raw(4) = msg->wrench.torque.y;
  raw(5) = msg->wrench.torque.z;

  robot_.updateEndEffectorWrench(raw, object_.getNormalVectSurfObj(), filteredForceGain_, wrenchBiasOK_, k);
}

void DualArmControl::updateRobotStatesCallback(const sensor_msgs::JointState::ConstPtr& msg, int k) {
  Vector7f jointPosition, jointVelocity, jointTorques;

  for (int i = 0; i < robot_.getNbJoints(k); i++) {
    jointPosition(i) = (float) msg->position[i];
    jointVelocity(i) = (float) msg->velocity[i];
    jointTorques(i) = (float) msg->effort[i];
    if (i == robot_.getNbJoints(k) - 1) {
      robot_.setJointsPositions(jointPosition, k);
      robot_.setJointsVelocities(jointVelocity, k);
      robot_.setJointsTorques(jointTorques, k);
      robot_.getEstimatedJointAccelerations(k);
    }
  }
}

// ---- Publish commands and data

void DualArmControl::publishCommands() {
  geometry_msgs::Pose vel_quat[NB_ROBOTS];

  for (int k = 0; k < NB_ROBOTS; k++) {
    Eigen::Vector3f axisAngleDes = robot_.getAxisAngleDes(k);
    Eigen::Vector3f vDes = robot_.getVDes(k);
    pubVel_[k].data.clear();
    pubVel_[k].data.push_back(axisAngleDes(0));// axis angle pose_x
    pubVel_[k].data.push_back(axisAngleDes(1));// axis angle pose_y
    pubVel_[k].data.push_back(axisAngleDes(2));// axis angle pose_z
    pubVel_[k].data.push_back(vDes(0));        // linear velocity v_x
    pubVel_[k].data.push_back(vDes(1));        // linear velocity v_y
    pubVel_[k].data.push_back(vDes(2));        // linear velocity v_z

    vel_quat[k].position.x = vDes(0);// desired velocity x
    vel_quat[k].position.y = vDes(1);// desired velocity y
    vel_quat[k].position.z = vDes(2);// desired velocity z

    Eigen::Vector4f qd = robot_.getQdSpecific(k);
    vel_quat[k].orientation.w = qd(0);// desired pose
    vel_quat[k].orientation.x = qd(1);
    vel_quat[k].orientation.y = qd(2);
    vel_quat[k].orientation.z = qd(3);
  }

  pubTSCommands_[LEFT].publish(pubVel_[LEFT]);
  pubTSCommands_[RIGHT].publish(pubVel_[RIGHT]);
  pubDesiredVelQuat_[LEFT].publish(vel_quat[LEFT]);
  pubDesiredVelQuat_[RIGHT].publish(vel_quat[RIGHT]);
}

void DualArmControl::publishData() {

  for (int k = 0; k < NB_ROBOTS; k++) {
    Eigen::Vector3f vDes = robot_.getVDes(k);
    Eigen::Vector3f omegaDes = robot_.getOmegaDes(k);

    // Publish desired twist
    geometry_msgs::Twist msgDesiredTwist;
    msgDesiredTwist.linear.x = vDes(0);
    msgDesiredTwist.linear.y = vDes(1);
    msgDesiredTwist.linear.z = vDes(2);
    // Convert desired end effector frame angular velocity to world frame
    msgDesiredTwist.angular.x = omegaDes(0);
    msgDesiredTwist.angular.y = omegaDes(1);
    msgDesiredTwist.angular.z = omegaDes(2);
    pubDesiredTwist_[k].publish(msgDesiredTwist);

    // Publish desired orientation
    geometry_msgs::Quaternion msgDesiredOrientation;
    Eigen::Vector4f qd = robot_.getQdSpecific(k);
    msgDesiredOrientation.w = qd(0);
    msgDesiredOrientation.x = qd(1);
    msgDesiredOrientation.y = qd(2);
    msgDesiredOrientation.z = qd(3);
    pubDesiredOrientation_[k].publish(msgDesiredOrientation);

    // Filtered wrench
    geometry_msgs::WrenchStamped msgFilteredWrench;
    msgFilteredWrench.header.frame_id = "world";
    msgFilteredWrench.header.stamp = ros::Time::now();
    msgFilteredWrench.wrench.force.x = robot_.getFilteredWrench(k)(0);
    msgFilteredWrench.wrench.force.y = robot_.getFilteredWrench(k)(1);
    msgFilteredWrench.wrench.force.z = robot_.getFilteredWrench(k)(2);
    msgFilteredWrench.wrench.torque.x = robot_.getFilteredWrench(k)(3);
    msgFilteredWrench.wrench.torque.y = robot_.getFilteredWrench(k)(4);
    msgFilteredWrench.wrench.torque.z = robot_.getFilteredWrench(k)(5);
    pubFilteredWrench_[k].publish(msgFilteredWrench);

    // Normal forces
    std_msgs::Float64 msg;
    msg.data = normalForce_[k];
    pubNormalForce_[k].publish(msg);

    // Distance EE - attractor
    msg.data = err_[k];
    pubDistAttractorEE_[k].publish(msg);

    // Attractor info
    Eigen::Matrix4f getWHGpSpecific = object_.getWHGpSpecific(k);
    geometry_msgs::Pose msgPose;
    msgPose.position.x = getWHGpSpecific(0, 3);
    msgPose.position.y = getWHGpSpecific(1, 3);
    msgPose.position.z = getWHGpSpecific(2, 3);
    Eigen::Matrix3f Rgr = getWHGpSpecific.block(0, 0, 3, 3);
    Eigen::Quaternionf qgr(Rgr);
    msgPose.orientation.x = qgr.x();
    msgPose.orientation.y = qgr.y();
    msgPose.orientation.z = qgr.z();
    msgPose.orientation.w = qgr.w();
    pubAttractor_[k].publish(msgPose);

    // Norm of desired velocity
    std_msgs::Float64 msgVel;
    msgVel.data = robot_.getVelEESpecific(k).head(3).norm();
    pubNormLinVel_[k].publish(msgVel);

    // Applied wrench
    geometry_msgs::Wrench msgAppliedWrench;
    msgAppliedWrench.force.x = -nuWr0_ * CooperativeCtrl.getForceApplied(k)(0);
    msgAppliedWrench.force.y = -nuWr0_ * CooperativeCtrl.getForceApplied(k)(1);
    msgAppliedWrench.force.z = -nuWr0_ * CooperativeCtrl.getForceApplied(k)(2);
    msgAppliedWrench.torque.x = -nuWr0_ * CooperativeCtrl.getForceApplied(k)(3);
    msgAppliedWrench.torque.y = -nuWr0_ * CooperativeCtrl.getForceApplied(k)(4);
    msgAppliedWrench.torque.z = -nuWr0_ * CooperativeCtrl.getForceApplied(k)(5);
    pubAppliedWrench_[k].publish(msgAppliedWrench);

    // Contact normal and applied moment
    Eigen::Vector3f normalVectSurfObj = object_.getNormalVectSurfObjSpecific(k);
    geometry_msgs::Wrench msgFnormMoment;
    msgFnormMoment.force.x = normalVectSurfObj(0);
    msgFnormMoment.force.y = normalVectSurfObj(1);
    msgFnormMoment.force.z = normalVectSurfObj(2);
    msgFnormMoment.torque.x = -CooperativeCtrl.getForceApplied(k)(3);
    msgFnormMoment.torque.y = -CooperativeCtrl.getForceApplied(k)(4);
    msgFnormMoment.torque.z = -CooperativeCtrl.getForceApplied(k)(5);
    pubAppliedFNormMoment_[k].publish(msgFnormMoment);
  }

  // Send speed command to the conveyor belt
  std_msgs::Int32 speedMsgConveyorBelt;
  speedMsgConveyorBelt.data = desSpeedConveyorBelt_;
  if (ctrlModeConveyorBelt_ && (fmod(cycleCount_, 30) == 0)) { pubConveyorBeltSpeed_.publish(speedMsgConveyorBelt); }
}

void DualArmControl::saveData() {
  Eigen::Vector3f xgrL = object_.getWHGpSpecific(LEFT).block(0, 3, 3, 1);
  Eigen::Vector3f xgrR = object_.getWHGpSpecific(RIGHT).block(0, 3, 3, 1);
  Eigen::Vector4f qgrL = Utils<float>::rotationMatrixToQuaternion(object_.getWHGpSpecific(LEFT).block(0, 0, 3, 3)); //
  Eigen::Vector4f qgrR = Utils<float>::rotationMatrixToQuaternion(object_.getWHGpSpecific(RIGHT).block(0, 0, 3, 3));//
  //
  Eigen::MatrixXf power_left = robot_.getJointsTorques(LEFT).transpose() * robot_.getJointsVelocities(LEFT);
  Eigen::MatrixXf power_right = robot_.getJointsTorques(RIGHT).transpose() * robot_.getJointsVelocities(RIGHT);
  Eigen::Matrix4f wHDoObject = object_.getWHDo();

  dataLog_.outRecordPose << (float) (cycleCount_ * dt_) << ", ";// cycle time
  dataLog_.outRecordPose << robot_.getX(LEFT).transpose().format(CSVFormat) << " , "
                         << robot_.getQ(LEFT).transpose().format(CSVFormat) << " , ";// left end-effector
  dataLog_.outRecordPose << robot_.getX(RIGHT).transpose().format(CSVFormat) << " , "
                         << robot_.getQ(RIGHT).transpose().format(CSVFormat) << " , ";// right end-effector
  dataLog_.outRecordPose << object_.getXo().transpose().format(CSVFormat) << " , "
                         << object_.getQo().transpose().format(CSVFormat) << " , ";// object
  dataLog_.outRecordPose << wHDoObject(0, 3) << " , " << wHDoObject(1, 3) << " , " << wHDoObject(2, 3)
                         << " , ";// desired object
  dataLog_.outRecordPose << xgrL.transpose().format(CSVFormat) << " , " << qgrL.transpose().format(CSVFormat)
                         << " , ";// left  grasping point
  dataLog_.outRecordPose << xgrR.transpose().format(CSVFormat) << " , " << qgrR.transpose().format(CSVFormat)
                         << " , ";// right grasping point
  dataLog_.outRecordPose << tossVar_.releasePosition.transpose().format(CSVFormat) << " , "
                         << tossVar_.releaseOrientation.transpose().format(CSVFormat) << " , ";// release pose
  dataLog_.outRecordPose << tossVar_.restPosition.transpose().format(CSVFormat) << " , "
                         << tossVar_.restOrientation.transpose().format(CSVFormat) << " , ";// rest pose
  dataLog_.outRecordPose << target_.getXt().transpose().format(CSVFormat) << " , "
                         << target_.getQt().transpose().format(CSVFormat) << " , ";// target pose
  dataLog_.outRecordPose << target_.getXdLanding().transpose().format(CSVFormat) << " , "
                         << target_.getXIntercept().transpose().format(CSVFormat)
                         << " , ";// landing and intercept position
  dataLog_.outRecordPose << target_.getXtStateToGo().transpose().format(CSVFormat) << " , "
                         << xPlacing_.transpose().format(CSVFormat) << std::endl;// target state to go

  dataLog_.outRecordVel << (float) (cycleCount_ * dt_) << ", ";
  dataLog_.outRecordVel << robot_.getVelDesEESpecific(LEFT).transpose().format(CSVFormat) << " , "
                        << robot_.getVelDesEESpecific(RIGHT).transpose().format(CSVFormat) << " , ";
  dataLog_.outRecordVel << robot_.getVelEESpecific(LEFT).transpose().format(CSVFormat) << " , "
                        << robot_.getVelEESpecific(RIGHT).transpose().format(CSVFormat) << " , ";
  dataLog_.outRecordVel << robot_.getVDes(LEFT).transpose().format(CSVFormat) << " , "
                        << robot_.getVDes(RIGHT).transpose().format(CSVFormat) << " , ";
  dataLog_.outRecordVel << robot_.getOmegaDes(LEFT).transpose().format(CSVFormat) << " , "
                        << robot_.getOmegaDes(RIGHT).transpose().format(CSVFormat) << " , ";
  dataLog_.outRecordVel << object_.getVo().transpose().format(CSVFormat) << " , "
                        << object_.getWo().transpose().format(CSVFormat) << " , ";
  dataLog_.outRecordVel << objVelDes_.transpose().format(CSVFormat) << " , ";
  dataLog_.outRecordVel << tossVar_.releaseLinearVelocity.transpose().format(CSVFormat) << " , "
                        << tossVar_.releaseAngularVelocity.transpose().format(CSVFormat) << " , ";
  dataLog_.outRecordVel << target_.getVt().transpose().format(CSVFormat) << std::endl;

  dataLog_.outRecordEfforts << (float) (cycleCount_ * dt_) << ", ";
  dataLog_.outRecordEfforts << robot_.getFilteredWrench(LEFT).transpose().format(CSVFormat) << " , ";
  dataLog_.outRecordEfforts << robot_.getFilteredWrench(RIGHT).transpose().format(CSVFormat) << " , ";
  dataLog_.outRecordEfforts << CooperativeCtrl.getForceApplied(LEFT).transpose().format(CSVFormat) << " , ";
  dataLog_.outRecordEfforts << CooperativeCtrl.getForceApplied(RIGHT).transpose().format(CSVFormat) << std::endl;

  dataLog_.outRecordTasks << (float) (cycleCount_ * dt_) << ", ";
  dataLog_.outRecordTasks << desiredVelImp_ << " , " << desVtoss_ << " , ";
  dataLog_.outRecordTasks << goHome_ << " , " << goToAttractors_ << " , " << releaseAndretract_ << " , " << isThrowing_
                          << " , " << isPlacing_ << " , " << isContact_ << " , ";
  dataLog_.outRecordTasks << freeMotionCtrl_.getActivationProximity() << " , " << freeMotionCtrl_.getActivationNormal()
                          << " , " << freeMotionCtrl_.getActivationTangent() << " , "
                          << freeMotionCtrl_.getActivationRelease() << " , " << freeMotionCtrl_.getActivationRetract()
                          << " , ";
  dataLog_.outRecordTasks << dsThrowing_.getActivationProximity() << " , " << dsThrowing_.getActivationNormal() << " , "
                          << dsThrowing_.getActivationTangent() << " , " << dsThrowing_.getActivationToss() << " , ";
  dataLog_.outRecordTasks << betaVelMod_ << " , " << dualPathLenAvgSpeed_.transpose() << std::endl;

  dataLog_.outRecordJointStates << (float) (cycleCount_ * dt_) << ", ";
  dataLog_.outRecordJointStates << robot_.getJointsPositions(LEFT).transpose().format(CSVFormat) << " , "
                                << robot_.getJointsPositions(RIGHT).transpose().format(CSVFormat) << " , ";
  dataLog_.outRecordJointStates << robot_.getJointsVelocities(LEFT).transpose().format(CSVFormat) << " , "
                                << robot_.getJointsVelocities(RIGHT).transpose().format(CSVFormat) << " , ";
  dataLog_.outRecordJointStates << robot_.getJointsAccelerations(LEFT).transpose().format(CSVFormat) << " , "
                                << robot_.getJointsAccelerations(RIGHT).transpose().format(CSVFormat) << " , ";
  dataLog_.outRecordJointStates << robot_.getJointsTorques(LEFT).transpose().format(CSVFormat) << " , "
                                << robot_.getJointsTorques(RIGHT).transpose().format(CSVFormat) << " , ";
  dataLog_.outRecordJointStates << power_left(0, 0) << " , " << power_right(0, 0) << std::endl;
}

void DualArmControl::publishConveyorBeltCmds() {
  // Desired conveyor belt control mode
  std_msgs::Int32 modeMessage;
  modeMessage.data = modeConveyorBelt_;
  pubConveyorBeltMode_.publish(modeMessage);
}
