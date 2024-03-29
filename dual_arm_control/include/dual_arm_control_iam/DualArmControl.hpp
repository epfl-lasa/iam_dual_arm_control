//|
//|    Copyright (C) 2021-2023 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Authors: Michael Bombile (maintainer)
//|
//|    email:   michael.bombile@epfl.ch/micbombile@gmail.com
//|
//|    Other contributors:
//|             Elise Jeandupeux (elise.jeandupeux@epfl.ch)
//|
//|    website: lasa.epfl.ch
//|
//|    This file is part of iam_dual_arm_control.
//|    This work was supported by the European Community's Horizon 2020 Research and Innovation
//|    programme (call: H2020-ICT-09-2019-2020, RIA), grant agreement 871899 Impact-Aware Manipulation.
//|
//|    iam_dual_arm_control is free software: you can redistribute it and/or modify  it under the terms
//|    of the GNU General Public License as published by  the Free Software Foundation,
//|    either version 3 of the License, or  (at your option) any later version.
//|
//|    iam_dual_arm_control is distributed in the hope that it will be useful,
//|    but WITHOUT ANY WARRANTY; without even the implied warranty of
//|    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//|    GNU General Public License for more details.
//|

#pragma once

#include <chrono>
#include <ctime>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <pthread.h>
#include <signal.h>
#include <sstream>
#include <stdio.h>
#include <termios.h>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "sg_filter.h"

// #include "dual_arm_control_iam/DataLogging.hpp"
#include "dual_arm_control_iam/DualArmCooperativeController.hpp"
#include "dual_arm_control_iam/DualArmFreeMotionController.hpp"
#include "dual_arm_control_iam/ObjectToGrasp.hpp"
#include "dual_arm_control_iam/RobotVariables.hpp"
#include "dual_arm_control_iam/ThrowingDS.hpp"
#include "dual_arm_control_iam/TossTaskParamEstimator.hpp"
#include "dual_arm_control_iam/TossingTarget.hpp"
#include "dual_arm_control_iam/tools/Utils.hpp"

#define NB_ROBOTS 2                // Number of robots
#define NB_FT_SENSOR_SAMPLES 50    // Number of FT sensors' samples used for initial calibration (compute the offsets)
#define NB_TRACKED_OBJECTS 6       // Number of objects tracked by the motion capture system (optitrack)
#define MOVING_FORCE_WINDOW_SIZE 10// Window's size used to average the force data and detect peristent contact
#define NB_OBJECTS 3               // Number of objects

typedef Eigen::Matrix<float, 6, 1> Vector6f;
typedef Eigen::Matrix<float, 6, 6> Matrix6f;
typedef Eigen::Matrix<float, 7, 1> Vector7f;

struct CommandStruct {
  Eigen::Vector3f axisAngleDes[NB_ROBOTS];
  Eigen::Vector3f vDes[NB_ROBOTS];
  Eigen::Vector3f omegaDes[NB_ROBOTS];
  Eigen::Vector4f qd[NB_ROBOTS];
  Vector6f filteredWrench[NB_ROBOTS];
  Eigen::Matrix4f whgpSpecific[NB_ROBOTS];
  Vector6f velEESpecific[NB_ROBOTS];
  Vector6f appliedWrench[NB_ROBOTS];
  Eigen::Vector3f normalVectSurfObj[NB_ROBOTS];
  float err[NB_ROBOTS];
  float nuWr0;
};

struct StateMachine {
  bool goHome;
  bool goToAttractors;
  bool isThrowing;
  bool isPlacing;
  bool isPlaceTossing;
  bool releaseAndretract;
  int dualTaskSelector;
  float desVtoss;
  float desiredVelImp;
  float placingPosHeight;
  float releasePosY;

  //disturbance
  bool incrementReleasePos;
  Eigen::Vector3f deltaRelPos;
  Eigen::Vector3f deltaPos;
  float trackingFactor;
  bool adaptationActive;
};

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

struct tossingTaskVariables {

  Eigen::Vector3f releasePosition;
  Eigen::Vector4f releaseOrientation;
  Eigen::Vector3f releaseLinearVelocity;
  Eigen::Vector3f releaseAngularVelocity;
  Eigen::Vector3f restPosition;
  Eigen::Vector4f restOrientation;
};

struct DataToSave {
  // Robot
  Eigen::Vector3f robotX[NB_ROBOTS];
  Eigen::Vector4f robotQ[NB_ROBOTS];
  Vector6f robotVelDesEE[NB_ROBOTS];
  Vector6f robotVelEE[NB_ROBOTS];
  Eigen::Vector3f robotVDes[NB_ROBOTS];
  Eigen::Vector3f robotOmegaDes[NB_ROBOTS];
  Vector6f robotFilteredWrench[NB_ROBOTS];
  Vector6f robotWrench[NB_ROBOTS];
  Vector7f robotJointsPositions[NB_ROBOTS];
  Vector7f robotJointsVelocities[NB_ROBOTS];
  Vector7f robotJointsAccelerations[NB_ROBOTS];
  Vector7f robotJointsTorques[NB_ROBOTS];

  // Object
  Eigen::Matrix4f objectWHGpSpecific[NB_ROBOTS];
  Eigen::Matrix4f objectWHDo;
  Eigen::Vector3f objectXo;
  Eigen::Vector4f objectQo;
  Eigen::Vector3f objectVo;
  Eigen::Vector3f objectWo;
  Vector6f objVelDes;

  // Target
  Eigen::Vector3f targetXt;
  Eigen::Vector4f targetQt;
  Eigen::Vector3f targetVt;
  Eigen::Vector3f targetXdLanding;
  Eigen::Vector3f targetXIntercept;
  Eigen::Vector3f targetXtStateToGo;

  // Tossing
  tossingTaskVariables tossVar;

  // Task
  Eigen::Vector3f taskXPlacing;
  float desiredVelImp;
  float betaVelMod;
  Eigen::Vector2f dualPathLenAvgSpeed;

  // Cooperative control
  Vector6f cooperativeCtrlForceApplied[NB_ROBOTS];

  // Free motion control
  float freeMotionCtrlActivationProximity;
  float freeMotionCtrlActivationNormal;
  float freeMotionCtrlActivationTangent;
  float freeMotionCtrlActivationRelease;
  float freeMotionCtrlActivationRetract;

  // DS throwing
  float dsThrowingActivationProximity;
  float dsThrowingActivationNormal;
  float dsThrowingActivationTangent;
  float dsThrowingActivationToss;

  // State machine
  bool goHome;
  bool goToAttractors;
  bool releaseAndretract;
  bool isThrowing;
  bool isPlacing;
  bool isContact;
  float desVtoss;
};

class DualArmControl {

private:
  double periodT_;

  // ---- Robot
  RobotVariable robot_;
  Vector6f vDesEE_[NB_ROBOTS];

  // ---- Object
  ObjectToGrasp object_;

  // ----  User interaction
  bool objCtrlKey_;
  bool userSelect_ = true;

  // ====================================================================================================================

  int contactState_;            // Contact state with the object
  float isContact_;             // Contact value (1 = CONTACT, 0 otherwise)
  bool wrenchBiasOK_[NB_ROBOTS];// Check if computation of force/torque sensor bias is OK
  Eigen::Vector3f gravity_;
  float errorObjDim_;// Error to object dimension vector [m]
  float errorObjPos_;// Error to object center position [m]

  Eigen::Matrix4f oHEE_[NB_ROBOTS];

  // Passive DS controller
  float d1_[NB_ROBOTS];
  float err_[NB_ROBOTS];
  bool qpWrenchGeneration_;

  bool sensedContact_;

  // ---- Object
  Eigen::Vector3f filtDeltaAngMir_;

  Vector6f objVelDes_;// desired object velocity (toss)
  Vector6f desiredObjectWrench_;

  // ---- Tossing target
  TossingTarget target_;

  int initPoseCount_;// Counter of received initial poses measurements

  Matrix6f gainAbs_ = Eigen::ArrayXXf::Zero(6, 6);
  Matrix6f gainRel_ = Eigen::ArrayXXf::Zero(6, 6);

  // ---- Task
  Eigen::Vector3f xLifting_;
  Eigen::Vector4f qLifting_;
  Eigen::Vector3f xPlacing_;
  Eigen::Vector4f qPlacing_;

  float vMax_;
  float filteredForceGain_;
  float forceThreshold_;
  float nuWr0_;
  float nuWr1_;
  float applyVelo_;
  float desVtoss_;
  float desiredVelImp_;
  float desVreach_;
  float refVreach_;
  float frictionAngle_ = 0.0f;
  float frictionAngleMax_ = 0.0f;
  float heightViaPoint_;

  bool goHome_;
  bool releaseAndretract_;
  bool isThrowing_;    // if true execute throwing of the object
  bool goToAttractors_;// send the robots to their attractors
  bool isPlacing_;
  bool isPickupSet_;
  bool isPlaceTossing_;// fast interrupted placing motion
  bool impactDirPreset_ = true;
  int dualTaskSelector_ = 1;
  bool oldDualMethod_ = false;

  Eigen::Matrix3f basisQ_[NB_ROBOTS];
  Eigen::Vector3f dirImp_[NB_ROBOTS];
  Eigen::Vector3f vdImpact_[NB_ROBOTS];
  Eigen::Vector3f dualAngularLimit_;
  bool releaseFlag_;// 0=reach, 1=pick, 2=toss, 3=pick_and_toss, 4=pick_and_place

  float trackingFactor_;

  bool incrementReleasePos_ = false;
  SphericalPosition releasePos_;

  Eigen::Vector2f dualPathLenAvgSpeed_;
  bool isIntercepting_ = false;
  float betaVelMod_;
  float initSpeedScaling_;
  std::deque<float> windowSpeedEE_;
  float movingAvgSpeedEE_;
  bool adaptationActive_ = false;
  bool isTargetFixed_ = true;

  bool feasibleAlgo_ = false;
  bool pickupBased_ = true;
  bool trackTargetRotation_ = false;
  bool isMotionTriggered_ = false;
  bool isRatioFactor_ = false;
  float tolAttractor_ = 0.07f;
  float switchSlopeAdapt_ = 100.0f;
  float betaVelModUnfilt_ = 1.0f;
  float timeToInterceptTgt_;
  float timeToInterceptBot_;

  // ---- Keyboard interaction
  Eigen::Vector3f deltaRelPos_;
  Eigen::Vector3f deltaPos_;// variation of object position

  // ---- Unconstrained and contrained motion and force generation
  DualArmFreeMotionController freeMotionCtrl_; // Motion generation
  DualArmCooperativeController CooperativeCtrl;// Force generation
  ThrowingDS dsThrowing_;

  TossTaskParamEstimator tossParamEstimator_;// tossing task param estimator
  DualArmFreeMotionController freeMotionCtrlEstim_;
  ThrowingDS dsThrowingEstim_;

  tossingTaskVariables tossVar_;

  CommandStruct commandGenerated_;
  // ====================================================================================================================

public:
  /** 
   * Robot ID: left or right 
  */
  enum Robot { LEFT = 0, RIGHT = 1 };

  /**
   * Contact state:
   * CONTACT: Both robots are in contact with the object
   * CLOSE_TO_CONTACT: Both robots are close to make contact with the object
   * NO_CONTACT: Both robots are not in contact with the object
  */
  enum ContactState { CONTACT = 0, CLOSE_TO_CONTACT = 1, NO_CONTACT = 2 };

  /**
   * TaskType: dual-arm reaching and manipulation tasks
   * 0=reach, 1=pick, 2=toss, 3=pick_and_toss, 4=pick_and_place
   * 5=place_tossing 6=throwing, 7=Handing_over 8=pause_motion
  */
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

  DualArmControl(double dt);

  ~DualArmControl();

  bool initObjectParam(YAML::Node config);
  bool initTargetParam();
  bool initRobotParam(YAML::Node config);
  bool initFreeMotionCtrl(YAML::Node config);
  bool initTossVar(YAML::Node config);
  bool initDesTasksPosAndLimits(YAML::Node config);
  bool initDampingTopicCtrl(YAML::Node config);
  bool initTossParamEstimator(const std::string pathLearnedModelfolder);
  bool initDSThrowing();
  bool init();

  void reset();

  void asyncMotion(Eigen::Vector3f targetPose);
  void releaseRetractMotion();
  void constraintMotion(bool isPlacing, bool isPlaceTossing, bool isThrowing);
  void adaptToForce();
  void graspingForceToVelSpace();
  void unconstraintMotion();

  bool loadParamFromFile(const std::string pathToYamlFile, const std::string pathLearnedModelfolder);
  bool updateSim(std::vector<Eigen::Matrix<float, 6, 1>> robotWrench,
                 std::vector<Eigen::Vector3f> eePose,
                 std::vector<Eigen::Vector4f> eeOrientation,
                 Eigen::Vector3f objectPose,
                 Eigen::Vector4f objectOrientation,
                 Eigen::Vector3f targetPose,
                 Eigen::Vector4f targetOrientation,
                 std::vector<Eigen::Vector3f> eeVelLin,
                 std::vector<Eigen::Vector3f> eeVelAng,
                 std::vector<Vector7f> jointPosition,
                 std::vector<Vector7f> jointVelocity,
                 std::vector<Vector7f> jointTorques,
                 std::vector<Eigen::Vector3f> robotBasePos,
                 std::vector<Eigen::Vector4f> robotBaseOrientation);

  CommandStruct generateCommands(std::vector<float> firstEigenPassiveDamping,
                                 std::vector<Eigen::Matrix<float, 6, 1>> robotWrench,
                                 std::vector<Eigen::Vector3f> eePose,
                                 std::vector<Eigen::Vector4f> eeOrientation,
                                 Eigen::Vector3f objectPose,
                                 Eigen::Vector4f objectOrientation,
                                 Eigen::Vector3f targetPose,
                                 Eigen::Vector4f targetOrientation,
                                 std::vector<Eigen::Vector3f> eeVelLin,
                                 std::vector<Eigen::Vector3f> eeVelAng,
                                 std::vector<Vector7f> jointPosition,
                                 std::vector<Vector7f> jointVelocity,
                                 std::vector<Vector7f> jointTorques,
                                 std::vector<Eigen::Vector3f> robotBasePos,
                                 std::vector<Eigen::Vector4f> robotBaseOrientation,
                                 int cycleCount);
  void updateContactState();
  void computeCommands(std::vector<Eigen::Vector3f> eePose,
                       std::vector<Eigen::Vector4f> eeOrientation,
                       Eigen::Vector3f targetPose,
                       int cycleCount);
  void updatePoses();

  void updateReleasePosition();
  Eigen::Vector3f
  computeInterceptWithTarget(const Eigen::Vector3f& xTarget, const Eigen::Vector3f& vTarget, float phiInit);
  void findDesiredLandingPosition(bool isPlacing, bool isPlaceTossing, bool isThrowing);
  float getDesiredYawAngleTarget(const Eigen::Vector4f& qt, const Eigen::Vector3f& angLimit);
  void estimateTargetStateToGo(Eigen::Vector2f lengthPathAvgSpeedRobot,
                               Eigen::Vector2f lengthPathAvgSpeedTarget,
                               float flyTimeObj);
  void updateInterceptPosition(float flyTimeObj, float intercepLimits[]);
  void findReleaseConfiguration();
  void setReleaseState();
  void set2DPositionBoxConstraints(Eigen::Vector3f& positionVect, float limits[]);
  void computeAdaptationFactors(Eigen::Vector2f lengthPathAvgSpeedRobot,
                                Eigen::Vector2f lengthPathAvgSpeedTarget,
                                float flyTimeObj);
  Eigen::Vector3f
  getImpactDirection(Eigen::Vector3f objectDesiredForce, Eigen::Vector3f objNormal, float coeffFriction);
  void mirrorTargetToObjectOrientation(Eigen::Vector4f qt, Eigen::Vector4f& qo, Eigen::Vector3f angleLimit);
  void prepareCommands(Vector6f vDesEE[], Eigen::Vector4f qd[], Vector6f velGraspPos[]);

  bool getReleaseFlag();
  double getPeriod();

  // ---- Keyboard commands
  void keyboardVirtualObjectControl();
  void keyboardReferenceObjectControl();
  StateMachine getStateMachine();
  void updateStateMachine(StateMachine stateMachine);

  // ---- Get data for log
  DataToSave getDataToSave();

  float getContactConfidenceDual() {return CooperativeCtrl.getContactConfidence();}
};
