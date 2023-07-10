

#include "dual_arm_control_iam/DualArmControlSim.hpp"

DualArmControlSim::DualArmControlSim() { this->reset(); }

DualArmControlSim::~DualArmControlSim() {}

bool DualArmControlSim::initObjectParam(YAML::Node config) {
  // Object desired grasping points
  Eigen::Matrix3f oRGraspPosLeft;
  Eigen::Matrix3f oRGraspPosRight;
  oRGraspPosLeft.setZero();
  oRGraspPosRight.setZero();
  oRGraspPosLeft(0, 0) = 1.0f;//TODO YAML FILE ?
  oRGraspPosLeft(2, 1) = -1.0f;
  oRGraspPosLeft(1, 2) = 1.0f;
  oRGraspPosRight(0, 0) = 1.0f;
  oRGraspPosRight(2, 1) = 1.0f;
  oRGraspPosRight(1, 2) = -1.0f;

  // Savitzky-Golay Filter params TODO YAML FILE?
  int sgfP[3];
  int sgfO[3];
  sgfP[0] = 3;
  sgfP[1] = 3;
  sgfP[2] = 6;
  sgfO[0] = 4;
  sgfO[1] = 3;
  sgfO[2] = 10;

  // Object get param
  std::string objectName = config["object"]["name"].as<std::string>();
  float objectMassVect = config["object"][objectName]["mass"].as<float>();
  std::vector<float> graspOffsetLeftVect = config["object"][objectName]["graspOffset_L"].as<std::vector<float>>();
  std::vector<float> graspOffsetRightVect = config["object"][objectName]["graspOffset_R"].as<std::vector<float>>();
  std::vector<float> objectDimVect = config["object"][objectName]["dimension"].as<std::vector<float>>();
  Eigen::Vector3f graspOffsetLeft =
      Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(graspOffsetLeftVect.data(), graspOffsetLeftVect.size());
  Eigen::Vector3f graspOffsetRight =
      Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(graspOffsetRightVect.data(), graspOffsetRightVect.size());

  object_.init(sgfP, sgfO, periodT_, oRGraspPosLeft, oRGraspPosRight);
  object_.setObjectMass(objectMassVect);
  object_.setObjectDim(Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(objectDimVect.data(), objectDimVect.size()));

  // Relative grasping positons
  object_.setXGpO(Eigen::Vector3f(0.0f, -object_.getObjectDimSpecific(1) / 2.0f, 0.0f) + graspOffsetLeft, 0);
  object_.setXGpO(Eigen::Vector3f(0.0f, object_.getObjectDimSpecific(1) / 2.0f, 0.0f) + graspOffsetRight, 1);

  return true;
}

bool DualArmControlSim::initRobotParam(YAML::Node config) {

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

  toolOffsetFromEE[LEFT] = config["dual_system"]["tool"]["offsetEE"]["left"].as<float>();
  toolOffsetFromEE[RIGHT] = config["dual_system"]["tool"]["offsetEE"]["right"].as<float>();
  paramToolMass[LEFT] = config["dual_system"]["tool"]["mass"]["left"].as<float>();
  paramToolMass[RIGHT] = config["dual_system"]["tool"]["mass"]["right"].as<float>();

  toolComPositionFromSensorVect[LEFT] =
      config["dual_system"]["tool"]["com_position_from_sensor"]["left"].as<std::vector<float>>();
  toolComPositionFromSensorVect[RIGHT] =
      config["dual_system"]["tool"]["com_position_from_sensor"]["right"].as<std::vector<float>>();
  toolComPositionFromSensor[LEFT] =
      Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(toolComPositionFromSensorVect[LEFT].data(),
                                                    toolComPositionFromSensorVect[LEFT].size());
  toolComPositionFromSensor[RIGHT] =
      Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(toolComPositionFromSensorVect[RIGHT].data(),
                                                    toolComPositionFromSensorVect[RIGHT].size());

  xrbStandbyVect[LEFT] = config["dual_arm_task"]["standby_pose"]["robot_left"]["position"].as<std::vector<float>>();
  xrbStandbyVect[RIGHT] = config["dual_arm_task"]["standby_pose"]["robot_right"]["position"].as<std::vector<float>>();
  xrbStandby[LEFT] =
      Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(xrbStandbyVect[LEFT].data(), xrbStandbyVect[LEFT].size());
  xrbStandby[RIGHT] =
      Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(xrbStandbyVect[RIGHT].data(), xrbStandbyVect[RIGHT].size());

  qrbStandbyVect[LEFT] = config["dual_arm_task"]["standby_pose"]["robot_left"]["orientation"].as<std::vector<float>>();
  qrbStandbyVect[RIGHT] =
      config["dual_arm_task"]["standby_pose"]["robot_right"]["orientation"].as<std::vector<float>>();
  qrbStandby[LEFT] =
      Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(qrbStandbyVect[LEFT].data(), qrbStandbyVect[LEFT].size());
  qrbStandby[RIGHT] =
      Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(qrbStandbyVect[RIGHT].data(), qrbStandbyVect[RIGHT].size());

  // Savitzky-Golay Filter params // TODO IN PARAMS?
  int sgf_dq[3];
  sgf_dq[0] = 7;// dim
  sgf_dq[1] = 3;// order
  sgf_dq[2] = 6;// window length

  robot_.init(sgf_dq, periodT_, gravity_);

  robot_.setInitParameters(paramToolMass, toolOffsetFromEE, toolComPositionFromSensor, xrbStandby, qrbStandby);

  return true;
}

bool DualArmControlSim::initFreeMotionCtrl(YAML::Node config) {
  bool modulatedReaching = config["dual_arm_task"]["modulated_reaching"].as<bool>();
  bool isNormImpactVel = config["dual_arm_task"]["isNorm_impact_vel"].as<bool>();

  desVreach_ = config["dual_arm_task"]["reach_to_grasp"]["desVreach"].as<float>();
  heightViaPoint_ = config["dual_arm_task"]["placing"]["height_via_point"].as<float>();

  freeMotionCtrl_.init(robot_.getWHEEStandby(), this->gainAbs_, this->gainRel_);
  freeMotionCtrl_.setDt(periodT_);
  freeMotionCtrl_.setObjectDim(object_.getObjectDim());
  freeMotionCtrl_.setDesVelReach(desVreach_);
  freeMotionCtrl_.setRefVelReach(refVreach_, LEFT);
  freeMotionCtrl_.setRefVelReach(refVreach_, RIGHT);
  freeMotionCtrl_.setModulatedReaching(modulatedReaching);
  freeMotionCtrl_.setIsNormImpactVel(isNormImpactVel);
  freeMotionCtrl_.setHeightViaPoint(heightViaPoint_);

  return true;
}

bool DualArmControlSim::initTossVar(YAML::Node config) {
  std::vector<float> releasePosVect = config["dual_arm_task"]["tossing"]["releasePos"].as<std::vector<float>>();
  std::vector<float> releaseOrientVect = config["dual_arm_task"]["tossing"]["releaseOrient"].as<std::vector<float>>();
  std::vector<float> releaseLinVelDirVect =
      config["dual_arm_task"]["tossing"]["releaseLinVel_dir"].as<std::vector<float>>();
  std::vector<float> releaseAngVelVect = config["dual_arm_task"]["tossing"]["releaseAngVel"].as<std::vector<float>>();
  std::vector<float> restOrientVect = config["dual_arm_task"]["tossing"]["restOrient"].as<std::vector<float>>();
  std::vector<float> restPosVect = config["dual_arm_task"]["tossing"]["restPos"].as<std::vector<float>>();

  desVtoss_ = config["dual_arm_task"]["tossing"]["desVtoss"].as<float>();

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

bool DualArmControlSim::initDesTasksPosAndLimits(YAML::Node config) {
  std::vector<float> paramAbsGains =
      config["dual_arm_task"]["coordination"]["ds_absolute_gains"].as<std::vector<float>>();
  std::vector<float> paramRelGains =
      config["dual_arm_task"]["coordination"]["ds_relative_gains"].as<std::vector<float>>();
  std::vector<float> paramXDoLifting = config["dual_arm_task"]["lifting"]["position"].as<std::vector<float>>();
  std::vector<float> paramQDoLifting = config["dual_arm_task"]["lifting"]["orientation"].as<std::vector<float>>();
  std::vector<float> paramXDoPlacing = config["dual_arm_task"]["placing"]["position"].as<std::vector<float>>();
  std::vector<float> paramQDoPlacing = config["dual_arm_task"]["placing"]["orientation"].as<std::vector<float>>();
  std::vector<float> paramDualAngularLimit =
      config["dual_arm_task"]["tossing"]["dual_angular_limit"].as<std::vector<float>>();

  gainAbs_.diagonal() = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(paramAbsGains.data(), paramAbsGains.size());
  gainRel_.diagonal() = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(paramRelGains.data(), paramRelGains.size());
  xLifting_ = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(paramXDoLifting.data(), paramXDoLifting.size());
  qLifting_ = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(paramQDoLifting.data(), paramQDoLifting.size());
  xPlacing_ = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(paramXDoPlacing.data(), paramXDoPlacing.size());
  qPlacing_ = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(paramQDoPlacing.data(), paramQDoPlacing.size());
  dualAngularLimit_ =
      Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(paramDualAngularLimit.data(), paramDualAngularLimit.size());

  Eigen::Matrix3f RDo_lifting = Utils<float>::quaternionToRotationMatrix(qLifting_);
  filtDeltaAngMir_ = Utils<float>::getEulerAnglesXYZ_FixedFrame(RDo_lifting);

  return true;
}

bool DualArmControlSim::initTossParamEstimator(const std::string pathLearnedModelfolder) {
  std::string file_gmm[3];
  std::string dataType = "/throwingParam";

  file_gmm[0] = pathLearnedModelfolder + dataType + "_prio.txt";
  file_gmm[1] = pathLearnedModelfolder + dataType + "_mu.txt";
  file_gmm[2] = pathLearnedModelfolder + dataType + "_sigma.txt";

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

bool DualArmControlSim::initDSThrowing() {

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

bool DualArmControlSim::loadParamFromFile(const std::string pathToYamlFile, const std::string pathLearnedModelfolder) {
  YAML::Node config = YAML::LoadFile(pathToYamlFile);
  // TODO COMPLETE AND PUT IN YAML FILE

  qpWrenchGeneration_ = config["dual_arm_task"]["isQP_wrench_generation"].as<bool>();
  isTargetFixed_ = config["dual_arm_task"]["isTargetFixed"].as<bool>();

  desiredVelImp_ = config["dual_arm_task"]["reach_to_grasp"]["impact"]["desVimp"].as<float>();
  frictionAngle_ =
      config["dual_arm_task"]["reach_to_grasp"]["impact"]["impact_direction"]["friction_angle"].as<float>();
  frictionAngleMax_ =
      config["dual_arm_task"]["reach_to_grasp"]["impact"]["impact_direction"]["max_friction_angle"].as<float>();
  impactDirPreset_ =
      config["dual_arm_task"]["reach_to_grasp"]["impact"]["impact_direction"]["impact_dir_preset"].as<bool>();

  incrementReleasePos_ = config["dual_arm_task"]["tossing"]["increment_release_pos"].as<bool>();
  dualTaskSelector_ = config["dual_arm_task"]["dualTaskSelector"].as<int>();
  oldDualMethod_ = config["dual_arm_task"]["old_dual_method"].as<bool>();

  // User interaction
  objCtrlKey_ = config["dual_arm_task"]["objCtrlKey"].as<bool>();
  userSelect_ = config["dual_arm_task"]["userSelect"].as<bool>();

  // Get params and init
  initTossVar(config);
  initDesTasksPosAndLimits(config);
  initRobotParam(config);
  initObjectParam(config);
  target_.init(3, 3, 10, periodT_);

  initFreeMotionCtrl(config);
  CooperativeCtrl.init();
  initTossParamEstimator(pathLearnedModelfolder);

  // Object tossing DS
  target_.setXdLanding(Eigen::Vector3f(1.0f, 0.0f, 0.0f));
  target_.setXIntercept(target_.getXdLanding());
  object_.setXPickup(object_.getXo());

  // Initialize throwing object
  initDSThrowing();
  dsThrowingEstim_ = dsThrowing_;
  freeMotionCtrlEstim_ = freeMotionCtrl_;

  bool isSim = config["dual_system"]["simulation"].as<bool>();
  std::cout << "Load param from file TODO " << isSim << std::endl;

  return true;
}

void DualArmControlSim::reset() { refVreach_ = 0.0f; }

bool DualArmControlSim::init() {

  // TODO PUT IN YAML FILE?

  cycleCount_ = 0;
  gravity_ << 0.0f, 0.0f, -9.80665f;

  for (int k = 0; k < NB_ROBOTS; k++) {
    oHEE_[k].setIdentity();
    d1_[k] = 1.0f;
    err_[k] = 1.0f;
    wrenchBiasOK_[k] = false;

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

  // Forces
  filteredForceGain_ = 0.9f;
  sensedContact_ = false;
  forceThreshold_ = 2.0f;
  nuWr0_ = 0.0f;
  nuWr1_ = 0.0f;

  applyVelo_ = 0.0f;
  refVreach_ = 0.0f;

  goHome_ = true;
  goToAttractors_ = true;
  releaseAndretract_ = false;
  isThrowing_ = false;
  isPlacing_ = false;
  isPlaceTossing_ = false;
  releaseFlag_ = false;
  isPickupSet_ = false;
  dualTaskSelector_ = 1;

  dualPathLenAvgSpeed_.setZero();

  isIntercepting_ = false;
  betaVelMod_ = 1.0f;
  initSpeedScaling_ = 1.0f;//0.75;
  trackingFactor_ = 0.40f; //0.17f; 	// 0.35f better

  adaptationActive_ = false;

  feasibleAlgo_ = false;// ? TODO
  pickupBased_ = true;
  trackTargetRotation_ = true;
  isMotionTriggered_ = false;
  isRatioFactor_ = false;
  tolAttractor_ = 0.07f;
  switchSlopeAdapt_ = 100.0f;
  betaVelModUnfilt_ = 1.0f;
  timeToInterceptTgt_ = 0.0f;
  timeToInterceptBot_ = 0.0f;

  // TODO LOGGING?
  // startlogging_ = false;

  return true;
}

bool DualArmControlSim::updateSim(Eigen::Matrix<float, 6, 1> robotWrench,
                                  Eigen::Vector3f eePose,
                                  Eigen::Vector4f eeOrientation,
                                  Eigen::Vector3f objectPose,
                                  Eigen::Vector4f objectOrientation,
                                  Eigen::Vector3f targetPose,
                                  Eigen::Vector4f targetOrientation,
                                  Eigen::Vector3f eeVelLin,
                                  Eigen::Vector3f eeVelAng,
                                  Vector7f jointPosition,
                                  Vector7f jointVelocity,
                                  Vector7f jointTorques,
                                  Eigen::Vector3f robotBasePos,
                                  Eigen::Vector4f robotBaseOrientation) {

  // TODO in the base pose needed? In which frame should the variables be send?

  for (int k = 0; k < NB_ROBOTS; k++) {
    robot_.updateEndEffectorWrench(robotWrench, object_.getNormalVectSurfObj(), filteredForceGain_, wrenchBiasOK_, k);
    robot_.updateEndEffectorPosesInWorld(eePose, eeOrientation, k);
    robot_.updateEndEffectorVelocity(eeVelLin, eeVelAng, k);

    robot_.setJointsPositions(jointPosition, k);
    robot_.setJointsVelocities(jointVelocity, k);
    robot_.setJointsTorques(jointTorques, k);
    robot_.getEstimatedJointAccelerations(k);

    robot_.getRobotBaseFrameInWorld(robotBasePos, robotBaseOrientation, k);
  }

  object_.setXo(objectPose);
  object_.setQo(objectOrientation);
  object_.getHmgTransform();//TODO change name?

  target_.setXt(targetPose);
  target_.setQt(targetOrientation);
  // filtered object position
  target_.getFilteredState();//TODO change name?

  return true;
}

void DualArmControlSim::updateContactState() {
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

CommandStruct DualArmControlSim::generateCommands(float firstEigenPassiveDamping[],
                                                  Eigen::Matrix<float, 6, 1> robotWrench,
                                                  Eigen::Vector3f eePose,
                                                  Eigen::Vector4f eeOrientation,
                                                  Eigen::Vector3f objectPose,
                                                  Eigen::Vector4f objectOrientation,
                                                  Eigen::Vector3f targetPose,
                                                  Eigen::Vector4f targetOrientation,
                                                  Eigen::Vector3f eeVelLin,
                                                  Eigen::Vector3f eeVelAng,
                                                  Vector7f jointPosition,
                                                  Vector7f jointVelocity,
                                                  Vector7f jointTorques,
                                                  Eigen::Vector3f robotBasePos,
                                                  Eigen::Vector4f robotBaseOrientation) {

  d1_[LEFT] = firstEigenPassiveDamping[LEFT];
  d1_[RIGHT] = firstEigenPassiveDamping[RIGHT];

  updateSim(robotWrench,
            eePose,
            eeOrientation,
            objectPose,
            objectOrientation,
            targetPose,
            targetOrientation,
            eeVelLin,
            eeVelAng,
            jointPosition,
            jointVelocity,
            jointTorques,
            robotBasePos,
            robotBaseOrientation);

  computeCommands(eePose, eeOrientation);

  for (int k = 0; k < NB_ROBOTS; k++) {
    commandGenerated_.axisAngleDes = robot_.getAxisAngleDes(k);
    commandGenerated_.vDes = robot_.getVDes(k);
    commandGenerated_.qd = robot_.getQdSpecific(k);
  }

  return commandGenerated_;
}

void DualArmControlSim::computeCommands(Eigen::Vector3f eePose, Eigen::Vector4f eeOrientation) {
  // Update contact state
  updateContactState();

  // Self imposed limits on intercept region (placing on moving target)
  float betaVelModUnfiltered = 1.0f;

  bool isContact = true && sensedContact_ && CooperativeCtrl.getContactConfidence() == 1.0f;
  bool isPlacing = isPlacing_ || (dualTaskSelector_ == PICK_AND_PLACE);
  bool isThrowing = isThrowing_ || (dualTaskSelector_ == TOSSING) || (dualTaskSelector_ == PICK_AND_TOSS);
  bool isPlaceTossing = isPlaceTossing_ || (dualTaskSelector_ == PLACE_TOSSING);
  // bool isClose2Release = (dsThrowing_.getActivationTangent() > 0.99f); TODO

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
    this->reset();//TODO

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
}

Eigen::Vector3f DualArmControlSim::computeInterceptWithTarget(const Eigen::Vector3f& xTarget,
                                                              const Eigen::Vector3f& vTarget,
                                                              float phiInit) {
  // TODO CHECK IF CAN DELETE
  // float phiTarget = 0.0f;
  // if (vTarget.head(2).norm() > 1e-2) { phiTarget = std::atan2(vTarget(1), vTarget(0)); }

  // float phiTargetTangent = std::tan(phiTarget);
  // if (phiTargetTangent > 1e4) { phiTargetTangent = 1e4; }
  // if (-phiTargetTangent < -1e4) { phiTargetTangent = -1e4; }

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

void DualArmControlSim::findDesiredLandingPosition(bool isPlacing, bool isPlaceTossing, bool isThrowing) {
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

float DualArmControlSim::getDesiredYawAngleTarget(const Eigen::Vector4f& qt, const Eigen::Vector3f& angLimit) {
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

void DualArmControlSim::estimateTargetStateToGo(Eigen::Vector2f lengthPathAvgSpeedRobot,
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

void DualArmControlSim::updateInterceptPosition(float flyTimeObj, float intercepLimits[]) {

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

void DualArmControlSim::findReleaseConfiguration() {
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

void DualArmControlSim::setReleaseState() {
  // Set the release state and the object pickup position
  dsThrowing_.setTossPose(tossVar_.releasePosition, tossVar_.releaseOrientation);
  dsThrowing_.setTossLinearVelocity(tossVar_.releaseLinearVelocity);
  dsThrowing_.setPickupObjectPose(object_.getXPickup(), object_.getQo());
}

void DualArmControlSim::set2DPositionBoxConstraints(Eigen::Vector3f& position_Vect, float limits[]) {
  if (position_Vect(0) < limits[0]) position_Vect(0) = limits[0];// x_min
  if (position_Vect(0) > limits[1]) position_Vect(0) = limits[1];// x_max
  if (position_Vect(1) < limits[2]) position_Vect(1) = limits[2];// y_min
  if (position_Vect(1) > limits[3]) position_Vect(1) = limits[3];// y_max
}

void DualArmControlSim::computeAdaptationFactors(Eigen::Vector2f lengthPathAvgSpeedRobot,
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

Eigen::Vector3f DualArmControlSim::getImpactDirection(Eigen::Vector3f objectDesiredForce,
                                                      Eigen::Vector3f objNormal,
                                                      float coeffFriction) {
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

void DualArmControlSim::mirrorTargetToObjectOrientation(Eigen::Vector4f qt,
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

void DualArmControlSim::prepareCommands(Vector6f vDesEE[], Eigen::Vector4f qd[], Vector6f velGraspPos[]) {
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

bool DualArmControlSim::getReleaseFlag() { return releaseAndretract_; }
double DualArmControlSim::getPeriod() { return periodT_; }

// void DualArmControlSim::updateStatesMachines() {
//   keyboard::nonBlock(1);

//   if (keyboard::khBit() != 0) {
//     char keyboardCommand = fgetc(stdin);
//     fflush(stdin);

//     switch (keyboardCommand) {
//       case 'q': {
//         goHome_ = !goHome_;
//         if (goHome_) {
//           goToAttractors_ = true;
//           startlogging_ = false;
//         } else if (!goHome_) {
//           startlogging_ = true;
//         }
//       } break;
//       case 'g': {
//         goToAttractors_ = !goToAttractors_;
//         if (goToAttractors_) {
//           goHome_ = false;
//           releaseAndretract_ = false;
//         }
//       } break;

//       // Conveyor belt control
//       case 'a': {
//         if (ctrlModeConveyorBelt_) {
//           modeConveyorBelt_ = 2;
//           publishConveyorBeltCmds();
//           startlogging_ = true;
//         } else if (incrementReleasePos_) {
//           deltaRelPos_(0) -= 0.025f;//[m]
//         } else {
//           deltaPos_(0) -= 0.01f;
//         }
//       } break;
//       case 's': {
//         if (ctrlModeConveyorBelt_) {
//           modeConveyorBelt_ = 0;
//           publishConveyorBeltCmds();
//         } else if (incrementReleasePos_) {
//           deltaRelPos_(0) += 0.025f;//[m]
//         } else {
//           deltaPos_(0) += 0.01f;
//         }
//       } break;
//       case 'd': {
//         if (ctrlModeConveyorBelt_) {
//           modeConveyorBelt_ = 1;
//           publishConveyorBeltCmds();
//         } else if (incrementReleasePos_) {
//           deltaRelPos_(1) -= 5.0f;//[deg]
//         } else {
//           deltaPos_(1) -= 0.01f;
//         }
//       } break;
//       case 'f': {
//         if (incrementReleasePos_) {
//           deltaRelPos_(1) += 5.0f;//[deg]
//         } else {
//           deltaPos_(1) += 0.01f;
//         }
//       } break;
//       case 'z': {
//         if (ctrlModeConveyorBelt_) {
//           trackingFactor_ -= 0.01f;
//         } else if (incrementReleasePos_) {
//           deltaRelPos_(2) -= 5.0f;//[deg]
//         } else {
//           deltaPos_(2) -= 0.01f;
//         }
//       } break;
//       case 'w': {
//         if (ctrlModeConveyorBelt_) {
//           trackingFactor_ += 0.01f;
//         } else if (incrementReleasePos_) {
//           deltaRelPos_(2) += 5.0f;//[deg]
//         } else {
//           deltaPos_(2) += 0.01f;
//         }
//       } break;
//       case 'h': {
//         if (ctrlModeConveyorBelt_) {
//           nominalSpeedConveyorBelt_ -= 50;
//         } else {
//           deltaAng_(0) -= 0.05f;
//         }
//       } break;
//       case 'j': {
//         if (ctrlModeConveyorBelt_) {
//           nominalSpeedConveyorBelt_ += 50;
//         } else {
//           deltaAng_(0) += 0.05f;
//         }
//       } break;
//       case 'k': {
//         if (ctrlModeConveyorBelt_) {
//           adaptationActive_ = !adaptationActive_;
//         } else {
//           deltaAng_(1) -= 0.05f;
//         }
//       } break;
//       case 'm': {
//         if (ctrlModeConveyorBelt_) {
//           magniturePertConveyorBelt_ -= 50;
//         } else {
//           deltaAng_(2) -= 0.05f;
//         }
//       } break;
//       case 'i': {
//         if (ctrlModeConveyorBelt_) {
//           magniturePertConveyorBelt_ += 50;
//         } else {
//           deltaAng_(2) += 0.05f;
//         }
//       } break;

//       // Release or throwing
//       case 'r': {
//         releaseAndretract_ = !releaseAndretract_;
//       } break;
//       case 'l': {
//         dualTaskSelector_ = PICK_AND_LIFT;
//         hasCaughtOnce_ = false;
//       } break;
//       case 't': {
//         isThrowing_ = !isThrowing_;
//         if (isThrowing_) {
//           dualTaskSelector_ = PICK_AND_TOSS;
//           hasCaughtOnce_ = false;
//         } else if (!isThrowing_) {
//           dualTaskSelector_ = PICK_AND_LIFT;
//         }
//       } break;
//       case 'p': {
//         isPlacing_ = !isPlacing_;
//         if (isPlacing_) {
//           dualTaskSelector_ = PICK_AND_PLACE;
//           hasCaughtOnce_ = false;
//         } else if (!isPlacing_) {
//           dualTaskSelector_ = PICK_AND_LIFT;
//         }
//       } break;
//       case 'o': {
//         isPlaceTossing_ = !isPlaceTossing_;
//         if (isPlaceTossing_) {
//           dualTaskSelector_ = PLACE_TOSSING;
//           hasCaughtOnce_ = false;
//         } else if (!isPlaceTossing_) {
//           dualTaskSelector_ = PICK_AND_LIFT;
//         }
//       } break;

//       // Impact and tossing velocity
//       case 'v': {
//         desVtoss_ -= 0.05f;
//         if (desVtoss_ < 0.2f) { desVtoss_ = 0.2f; }
//         dsThrowing_.setTossLinearVelocity(desVtoss_ * tossVar_.releaseLinearVelocity.normalized());
//         dsThrowingEstim_.setTossLinearVelocity(desVtoss_ * tossVar_.releaseLinearVelocity.normalized());
//       } break;
//       case 'b': {
//         desVtoss_ += 0.05f;
//         if (desVtoss_ > 2.0f) { desVtoss_ = 2.0f; }
//         dsThrowing_.setTossLinearVelocity(desVtoss_ * tossVar_.releaseLinearVelocity.normalized());
//         dsThrowingEstim_.setTossLinearVelocity(desVtoss_ * tossVar_.releaseLinearVelocity.normalized());
//       } break;
//       case 'y': {
//         desiredVelImp_ -= 0.05f;
//         if (desiredVelImp_ < 0.05f) { desiredVelImp_ = 0.05f; }
//       } break;
//       case 'u': {
//         desiredVelImp_ += 0.05f;
//         if (desiredVelImp_ > 0.6f) { desiredVelImp_ = 0.6f; }
//       } break;

//       // Reset the data logging
//       case 'c': {
//         startlogging_ = false;
//         dataLog_.reset(ros::package::getPath(std::string("dual_arm_control")) + "/Data");
//       } break;

//       // Disturb the target speed
//       case 'e': {
//         isDisturbTarget_ = !isDisturbTarget_;
//       } break;

//       // Placing hight
//       case 'x': {
//         if (dualTaskSelector_ == PICK_AND_TOSS) {
//           tossVar_.releasePosition(1) -= 0.01;
//         } else {
//           xPlacing_(2) -= 0.01;
//         }
//       } break;
//       case 'n': {
//         if (dualTaskSelector_ == PICK_AND_TOSS) {
//           tossVar_.releasePosition(1) += 0.01;
//         } else {
//           xPlacing_(2) += 0.01;
//         }
//       } break;
//     }
//   }
//   keyboard::nonBlock(0);
// }