

#include "dual_arm_control_iam/DualArmControlSim.hpp"

DualArmControlSim::DualArmControlSim() {}// NEED TO RESET VARIABLES

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

bool DualArmControlSim::loadParamFromFile(const std::string pathToYamlFile) {
  YAML::Node config = YAML::LoadFile(pathToYamlFile);
  // TODO COMPLETE AND PUT IN YAML FILE

  // ==============================================================
  // DS Param
  // ==============================================================
  bool isSim = config["dual_system"]["simulation"].as<bool>();

  double toolOffsetFromEE_[NB_ROBOTS];
  toolOffsetFromEE_[LEFT] = config["dual_system"]["tool"]["offsetEE"]["left"].as<double>();
  toolOffsetFromEE_[RIGHT] = config["dual_system"]["tool"]["offsetEE"]["right"].as<double>();

  // Forces
  filteredForceGain_ = 0.9f;
  for (int k = 0; k < NB_ROBOTS; k++) { wrenchBiasOK_[k] = false; }

  // Object
  initObjectParam(config);

  std::cout << "Load param from file TODO " << isSim << std::endl;

  return true;
}

bool DualArmControlSim::updateSim(Eigen::Matrix<float, 6, 1> robotWrench) {

  for (int k = 0; k < NB_ROBOTS; k++) {
    robot_.updateEndEffectorWrench(robotWrench, object_.getNormalVectSurfObj(), filteredForceGain_, wrenchBiasOK_, k);
  }

  return true;
}

void DualArmControlSim::updateContactState(Eigen::Vector3f EEPose, Eigen::Vector4f EEOrientation) {
  robot_.getEstimatedAverageNormalForce();

  //   // Compute errors to object center position and dimension vector
  //   Eigen::Matrix4f leftEERightEE = robot_.getWHEESpecific(LEFT).inverse() * robot_.getWHEESpecific(RIGHT);
  //   Eigen::Matrix4f leftGripPoseRightGripPose = object_.getWHGpSpecific(LEFT).inverse() * object_.getWHGpSpecific(RIGHT);
  //   Eigen::Vector3f errorObjPosVect =
  //       Utils<float>::getAbs3D(object_.getWHGp()) - Utils<float>::getAbs3D(robot_.getWHEE());
  //   errorObjDim_ = fabs(leftEERightEE(2, 3)) - fabs(leftGripPoseRightGripPose(2, 3));
  //   errorObjPos_ = errorObjPosVect.norm();

  //   if ((robot_.getNormalForceAverage(LEFT) > 2.0f || robot_.getNormalForceAverage(RIGHT) > 2.0f) && errorObjDim_ < 0.065f
  //       && (errorObjPos_ < 0.065f || CooperativeCtrl.getContactConfidence() == 1.0f)) {
  //     contactState_ = CONTACT;
  //     isContact_ = 1.0f;
  //   } else if (!(robot_.getNormalForceAverage(LEFT) > 2.0f && robot_.getNormalForceAverage(RIGHT) > 2.0f)
  //              && errorObjDim_ < 0.05f && errorObjPos_ < 0.05f) {
  //     contactState_ = CLOSE_TO_CONTACT;
  //     isContact_ = 0.0f;
  //   } else {
  //     contactState_ = NO_CONTACT;
  //     isContact_ = 0.0f;
  //   }

  //   // Check contact
  //   sensedContact_ = ((fabs(robot_.getNormalForce(LEFT)) >= forceThreshold_)
  //                     || (fabs(robot_.getNormalForce(RIGHT)) >= forceThreshold_))
  //       && (isContact_ == 1.0f);
}

void DualArmControlSim::generateCommands(Eigen::Vector3f EEPose, Eigen::Vector4f EEOrientation) {
  computeCommands(EEPose, EEOrientation);
}

void DualArmControlSim::computeCommands(Eigen::Vector3f EEPose, Eigen::Vector4f EEOrientation) {
  // Update contact state
  updateContactState(EEPose, EEOrientation);

  // // Self imposed limits on intercept region (placing on moving target)
  // float betaVelModUnfiltered = 1.0f;

  // bool isContact = true && sensedContact_ && CooperativeCtrl.getContactConfidence() == 1.0f;
  // bool isPlacing = isPlacing_ || (dualTaskSelector_ == PICK_AND_PLACE);
  // bool isThrowing = isThrowing_ || (dualTaskSelector_ == TOSSING) || (dualTaskSelector_ == PICK_AND_TOSS);
  // bool isPlaceTossing = isPlaceTossing_ || (dualTaskSelector_ == PLACE_TOSSING);
  // bool isClose2Release = (dsThrowing_.getActivationTangent() > 0.99f);

  // bool placingDone = (releaseFlag_) || ((object_.getWHo().block<3, 1>(0, 3) - xPlacing_).norm() <= 0.08);//0.05
  // bool placeTossingDone = (releaseFlag_)
  //     || (((object_.getWHo().block<3, 1>(0, 3) - tossVar_.releasePosition).norm() <= 0.07)
  //         || ((object_.getWHo().block<2, 1>(0, 3) - xPlacing_.head(2)).norm() <= 0.05));
  // bool tossingDone =
  //     (releaseFlag_) || (((object_.getWHo().block<3, 1>(0, 3) - tossVar_.releasePosition).norm() <= 0.035));
  // bool isForceDetected =
  //     (robot_.getNormalForceAverage(LEFT) > forceThreshold_ || robot_.getNormalForceAverage(RIGHT) > forceThreshold_);

  // Vector6f vDesEE[NB_ROBOTS];
  // // ---------- Intercept/ landing location ----------
  // // Compute intercept position with yaw angle limits for throwing object
  // Eigen::Vector3f interceptMin =
  //     this->computeInterceptWithTarget(target_.getXt(), target_.getVt(), -dualAngularLimit_(2));
  // Eigen::Vector3f interceptMax =
  //     this->computeInterceptWithTarget(target_.getXt(), target_.getVt(), dualAngularLimit_(2));

  // // Self imposed limits on intercept region (placing on moving target)
  // float intercepLimits[4];
  // intercepLimits[0] = 0.60f;          // x_min
  // intercepLimits[1] = 0.75f;          // x_max
  // intercepLimits[2] = interceptMin(1);// y_min
  // intercepLimits[3] = interceptMax(1);// y_max

  // // ---------- Application ----------
  // if ((!releaseAndretract_) && (fmod(cycleCount_, 20) == 0)) {
  //   dualPathLenAvgSpeed_ = freeMotionCtrlEstim_.predictRobotTranslation(robot_.getWHEE(),
  //                                                                       object_.getWHGp(),
  //                                                                       robot_.getWHEEStandby(),
  //                                                                       object_.getWHo(),
  //                                                                       tossVar_.releasePosition,
  //                                                                       desVtoss_,
  //                                                                       0.05f,
  //                                                                       0.100f,
  //                                                                       initSpeedScaling_);
  // }

  // Eigen::Vector2f lengthPathAvgSpeedRobot = {dualPathLenAvgSpeed_(0), trackingFactor_ * dualPathLenAvgSpeed_(1)};
  // Eigen::Vector2f lengthPathAvgSpeedTarget =
  //     tossParamEstimator_.estimateTargetSimplePathLengthAverageSpeed(target_.getXt(),
  //                                                                    target_.getXdLanding(),
  //                                                                    target_.getVt());

  // float flyTimeObj = 0.200f;
  // timeToInterceptTgt_ = 0.0f;
  // timeToInterceptBot_ = 0.0f;

  // // Determine the desired landing position
  // this->findDesiredLandingPosition(isPlacing, isPlaceTossing, isThrowing);

  // // Estimate the target state to go
  // this->estimateTargetStateToGo(lengthPathAvgSpeedRobot, lengthPathAvgSpeedTarget, flyTimeObj);

  // // Set at pickup instant
  // if (!isPickupSet_ && !releaseAndretract_) {
  //   if (sensedContact_ && (CooperativeCtrl.getContactConfidence() == 1.0)) {
  //     // Update intercept (desired landing) position
  //     this->updateInterceptPosition(flyTimeObj, intercepLimits);
  //     // Determination of the release configuration
  //     this->findReleaseConfiguration();
  //     // Set the release state and the object pickup position
  //     this->setReleaseState();

  //     isPickupSet_ = true;
  //   } else {
  //     object_.setXPickup(object_.getXo());
  //     dsThrowing_.setPickupObjectPose(object_.getXPickup(), object_.getQo());
  //   }
  // }

  // // Adaptation of the desired motion
  // this->computeAdaptationFactors(lengthPathAvgSpeedRobot, lengthPathAvgSpeedTarget, flyTimeObj);

  // if (goHome_) {
  //   freeMotionCtrl_.computeAsyncMotion(robot_.getWHEE(),
  //                                      robot_.getWHEEStandby(),
  //                                      object_.getWHo(),
  //                                      robot_.getVelDesEE(),
  //                                      robot_.getQd(),
  //                                      true);

  //   objVelDes_ =
  //       dsThrowing_.apply(object_.getXo(), object_.getQo(), object_.getVo(), Eigen::Vector3f(0.0f, 0.0f, 0.0f));

  //   for (int i = 0; i < NB_ROBOTS; i++) {
  //     dirImp_[i] =
  //         this->getImpactDirection(objVelDes_.head(3), object_.getNormalVectSurfObjSpecific(i), frictionAngle_);
  //     vdImpact_[i] = desiredVelImp_ * dirImp_[i];

  //     // Orthogonal Basis of Modulated Dual-arm DS
  //     basisQ_[i] = Utils<float>::create3dOrthonormalMatrixFromVector(dirImp_[i]);
  //   }

  //   // Reset some controller variables
  //   this->resetVariables();

  // } else {//  release_and_retract || release
  //   if (releaseAndretract_) {

  //     freeMotionCtrl_.computeReleaseAndRetractMotion(robot_.getWHEE(),
  //                                                    object_.getWHDgp(),
  //                                                    object_.getWHo(),
  //                                                    robot_.getVelDesEE(),
  //                                                    robot_.getQd(),
  //                                                    true);
  //     isThrowing_ = false;
  //     isPlacing_ = false;
  //     isPickupSet_ = false;
  //     isPlaceTossing_ = false;
  //     nuWr0_ = nuWr1_ = 0.0f;
  //     dsThrowing_.resetReleaseFlag();
  //     isIntercepting_ = false;
  //   } else if (isContact) {// Constraint motion phase (Cooperative control)
  //     objVelDes_ =
  //         dsThrowing_.apply(object_.getXo(), object_.getQo(), object_.getVo(), Eigen::Vector3f(0.0f, 0.0f, 0.0f));

  //     // Desired task position and orientation vectors
  //     Eigen::Vector3f xDesTask = xLifting_;
  //     Eigen::Vector4f qDesTask = qLifting_;
  //     if (isPlacing) {
  //       xDesTask = xPlacing_;
  //       qDesTask = qPlacing_;
  //     }
  //     if (isPlaceTossing) {
  //       xDesTask = xPlacing_;
  //       qDesTask = qPlacing_;
  //     }
  //     if (isThrowing) {
  //       xDesTask = tossVar_.releasePosition;
  //       qDesTask = tossVar_.releaseOrientation;
  //     }

  //     // Target to object Orientation Adaptation
  //     if (trackTargetRotation_) {
  //       this->mirrorTargetToObjectOrientation(target_.getQt(), qDesTask, dualAngularLimit_);
  //       dsThrowing_.setTossPose(tossVar_.releasePosition, qDesTask);
  //     }

  //     // Desired object pose
  //     Eigen::Matrix4f desObjPosHomogTransfo = Utils<float>::pose2HomoMx(xDesTask, qDesTask);
  //     object_.setWHDo(Utils<float>::pose2HomoMx(xDesTask, qDesTask));

  //     // Desired pose of the grasping points
  //     for (int k = 0; k < NB_ROBOTS; k++) {
  //       object_.setWHDgp(object_.getWHDo() * oHEE_[k], k);
  //       object_.getWHDgpSpecific(k).block(0, 0, 3, 3) = desObjPosHomogTransfo.block(0, 0, 3, 3)
  //           * Utils<float>::pose2HomoMx(object_.getXGpO(k), object_.getQGpO(k)).block(0, 0, 3, 3);
  //     }

  //     // Motion generation
  //     freeMotionCtrl_.dualArmMotion(robot_.getWHEE(),
  //                                   robot_.getVelEE(),
  //                                   object_.getWHDgp(),
  //                                   object_.getWHo(),
  //                                   object_.getWHDo(),
  //                                   objVelDes_,
  //                                   basisQ_,
  //                                   vdImpact_,
  //                                   false,
  //                                   dualTaskSelector_,
  //                                   robot_.getVelDesEE(),
  //                                   robot_.getQd(),
  //                                   releaseFlag_);

  //     // Release and Retract condition
  //     if ((isPlacing && placingDone) || (isPlaceTossing && placeTossingDone) || (isThrowing && tossingDone)) {
  //       releaseAndretract_ = true;
  //     }

  //   } else {// Unconstraint (Free) motion phase
  //     freeMotionCtrl_.setReachableP(
  //         (robot_.getWHEESpecific(LEFT)(0, 3) >= 0.72f || robot_.getWHEESpecific(RIGHT)(0, 3) >= 0.72f) ? 0.0f : 1.0f);

  //     if (false || oldDualMethod_) {

  //       freeMotionCtrl_.computeCoordinatedMotion2(robot_.getWHEE(),
  //                                                 object_.getWHGp(),
  //                                                 object_.getWHo(),
  //                                                 robot_.getVelDesEE(),
  //                                                 robot_.getQd(),
  //                                                 false);

  //       Eigen::Vector3f errPosAbs = object_.getWHo().block(0, 3, 3, 1) - Utils<float>::getAbs3D(robot_.getWHEE());
  //       Eigen::Vector3f objErrPosAbs = object_.getWHo().block<3, 3>(0, 0).transpose() * errPosAbs;
  //       Eigen::Vector3f objErrPosAbsParallel = Eigen::Vector3f(objErrPosAbs(0), 0.0f, objErrPosAbs(2));
  //       float cp_ap = Utils<float>::computeCouplingFactor(objErrPosAbsParallel, 50.0f, 0.17f, 1.0f, true);

  //       // Create impact at grabbing
  //       Vector6f* vDesEETest = robot_.getVelDesEE();
  //       vDesEE[LEFT].head(3) = vDesEE[LEFT].head(3) + dirImp_[LEFT] * cp_ap * desiredVelImp_;
  //       vDesEE[RIGHT].head(3) = vDesEE[RIGHT].head(3) + dirImp_[RIGHT] * cp_ap * desiredVelImp_;
  //     } else {

  //       freeMotionCtrl_.dualArmMotion(robot_.getWHEE(),
  //                                     robot_.getVelEE(),
  //                                     object_.getWHGp(),
  //                                     object_.getWHo(),
  //                                     object_.getWHDo(),
  //                                     objVelDes_,
  //                                     basisQ_,
  //                                     vdImpact_,
  //                                     false,
  //                                     0,
  //                                     robot_.getVelDesEE(),
  //                                     robot_.getQd(),
  //                                     releaseFlag_);
  //     }

  //     dsThrowing_.setRefVtoss(desiredVelImp_);

  //     // for data logging
  //     objVelDes_.setZero();

  //     if (freeMotionCtrl_.getActivationProximity() >= 0.2f) { betaVelModUnfilt_ = 1.0; }
  //   }

  //   if (isPlacing || isThrowing || isPlaceTossing) {
  //     // Force feedback to grab objects
  //     float gainForce = 0.02f;
  //     float absForceCorrection = nuWr0_ * gainForce * 0.5f
  //         * ((robot_.getFilteredWrench(LEFT).segment(0, 3) - CooperativeCtrl.getForceApplied(LEFT).head(3))
  //                .dot(object_.getNormalVectSurfObjSpecific(LEFT))
  //            + (robot_.getFilteredWrench(RIGHT).segment(0, 3) - CooperativeCtrl.getForceApplied(RIGHT).head(3))
  //                  .dot(object_.getNormalVectSurfObjSpecific(RIGHT)));

  //     if (fabs(absForceCorrection) > 0.2f) {
  //       absForceCorrection = absForceCorrection / fabs(absForceCorrection) * 0.2f;
  //     }

  //     Vector6f* vDesEETest = robot_.getVelDesEE();
  //     vDesEE[LEFT].head(3) =
  //         vDesEE[LEFT].head(3) - 0.40 * absForceCorrection * object_.getNormalVectSurfObjSpecific(LEFT);
  //     vDesEE[RIGHT].head(3) =
  //         vDesEE[RIGHT].head(3) - 0.40 * absForceCorrection * object_.getNormalVectSurfObjSpecific(RIGHT);
  //   }

  //   // ---------- Adaptation ----------
  //   if (!isContact && (freeMotionCtrl_.getActivationProximity() >= 0.2f)) { betaVelModUnfiltered = 1.0; }

  //   float filBeta = 0.10;
  //   betaVelMod_ = (1.f - filBeta) * betaVelMod_ + filBeta * betaVelModUnfiltered;

  //   if ((target_.getVt().norm() >= 0.05 && (!releaseAndretract_) && (dsThrowing_.getActivationProximity() <= 0.99f))) {

  //     Vector6f* vDesEETest = robot_.getVelDesEE();

  //     vDesEE[LEFT].head(3) *=
  //         initSpeedScaling_ * ((float) adaptationActive_ * betaVelMod_ + (1. - (float) adaptationActive_));
  //     vDesEE[RIGHT].head(3) *=
  //         initSpeedScaling_ * ((float) adaptationActive_ * betaVelMod_ + (1. - (float) adaptationActive_));
  //   }

  //   // ---------- Compute the object's grasp points velocity ----------
  //   object_.getGraspPointVelocity();

  //   // ---------- Generate grasping force and apply it in velocity space ----------
  //   // Desired object's task wrench
  //   desiredObjectWrench_.head(3) = -12.64f * (object_.getVo() - freeMotionCtrl_.get_des_object_motion().head(3))
  //       - object_.getObjectMass() * gravity_;
  //   desiredObjectWrench_.tail(3) = -25.00f * (object_.getWo() - freeMotionCtrl_.get_des_object_motion().tail(3));

  //   CooperativeCtrl.getAppliedWrenches(goHome_,
  //                                      contactState_,
  //                                      object_.getWHo(),
  //                                      robot_.getWHEE(),
  //                                      object_.getWHGp(),
  //                                      desiredObjectWrench_,
  //                                      object_.getObjectMass(),
  //                                      qpWrenchGeneration_,
  //                                      isForceDetected);

  //   // Applied force in velocity space
  //   for (int i = 0; i < NB_ROBOTS; i++) {
  //     robot_.setFXC(1.0f / d1_[i] * CooperativeCtrl.getForceApplied(i).head(3), i);
  //   }
  // }

  // // Compute the velocity to avoid EE collision
  // Vector6f vEEOA[NB_ROBOTS];
  // vEEOA[0] = robot_.getVEEObstacleAvoidance(0);
  // vEEOA[1] = robot_.getVEEObstacleAvoidance(1);
  // freeMotionCtrl_.computeEEAvoidanceVelocity(robot_.getWHEE(), vEEOA);
  // robot_.setVEEObstacleAvoidance(vEEOA);

  // // Extract linear velocity commands and desired axis angle command
  // this->prepareCommands(robot_.getVelDesEE(), robot_.getQd(), object_.getVGpO());

  // // ---------- Control of conveyor belt speed ----------
  // float omegaPert = 2.f * M_PI / 1;
  // float deltaOmegaPert = 0.1f * (2.f * (float) std::rand() / RAND_MAX - 1.0f) * omegaPert;

  // if (ctrlModeConveyorBelt_) {
  //   desSpeedConveyorBelt_ = (int) (nominalSpeedConveyorBelt_
  //                                  + (int) isDisturbTarget_ * magniturePertConveyorBelt_
  //                                      * sin((omegaPert + deltaOmegaPert) * dt_ * cycleCount_));
  // }
}

bool DualArmControlSim::getReleaseFlag() { return releaseAndretract_; }
double DualArmControlSim::getPeriod() { return periodT_; }