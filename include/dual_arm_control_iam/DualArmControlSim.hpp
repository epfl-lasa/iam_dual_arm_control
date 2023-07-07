#pragma once

#include <chrono>
#include <ctime>
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
#include "dual_arm_control_iam/tools/Keyboard.h"
#include "dual_arm_control_iam/tools/Utils.hpp"

#define NB_ROBOTS 2                // Number of robots
#define NB_FT_SENSOR_SAMPLES 50    // Number of FT sensors' samples used for initial calibration (compute the offsets)
#define NB_TRACKED_OBJECTS 6       // Number of objects tracked by the motion capture system (optitrack)
#define MOVING_FORCE_WINDOW_SIZE 10// Window's size used to average the force data and detect peristent contact
#define NB_OBJECTS 3               // Number of objects

typedef Eigen::Matrix<float, 6, 1> Vector6f;
typedef Eigen::Matrix<float, 6, 6> Matrix6f;
typedef Eigen::Matrix<float, 7, 1> Vector7f;

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

class DualArmControlSim {

private:
  int cycleCount_;
  double periodT_;

  // ---- Robot
  RobotVariable robot_;

  // ---- Object
  ObjectToGrasp object_;

  // ----  User interaction
  bool objCtrlKey_;
  bool userSelect_ = true;

  // ---- Target Disturbance
  bool isDisturbTarget_ = false;

  // ====================================================================================================================

  int contactState_;            // Contact state with the object
  float toolMass_;              // [kg]
  float normalForce_[NB_ROBOTS];// Normal force to the surface [N]
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
  bool startlogging_;

  // ---- Object
  Eigen::Vector3f filtDeltaAngMir_;

  Vector6f objVelDes_;// desired object velocity (toss)
  Vector6f desiredObjectWrench_;

  // ---- Tossing target
  TossingTarget target_;

  int initPoseCount_;// Counter of received initial poses measurements

  Matrix6f gainAbs_;
  Matrix6f gainRel_;

  std::deque<Eigen::Vector3f> windowVelTarget_;
  Eigen::Vector3f movingAvgVelTarget_;

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

  // create data logging object
  // DataLogging dataLog_;

  Eigen::Matrix3f basisQ_[NB_ROBOTS];
  Eigen::Vector3f dirImp_[NB_ROBOTS];
  Eigen::Vector3f vdImpact_[NB_ROBOTS];
  Eigen::Vector3f dualAngularLimit_;
  bool releaseFlag_;// 0=reach, 1=pick, 2=toss, 3=pick_and_toss, 4=pick_and_place

  float trackingFactor_;

  bool incrementReleasePos_ = false;
  bool ctrlModeConveyorBelt_ = false;
  SphericalPosition releasePos_;

  int desSpeedConveyorBelt_;
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

  // ---- Unconstrained and contrained motion and force generation
  DualArmFreeMotionController freeMotionCtrl_; // Motion generation
  DualArmCooperativeController CooperativeCtrl;// Force generation
  ThrowingDS dsThrowing_;

  TossTaskParamEstimator tossParamEstimator_;// tossing task param estimator
  DualArmFreeMotionController freeMotionCtrlEstim_;
  ThrowingDS dsThrowingEstim_;

  tossingTaskVariables tossVar_;
  // ====================================================================================================================

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

  DualArmControlSim();

  ~DualArmControlSim();

  bool initObjectParam(YAML::Node config);
  bool initRobotParam(YAML::Node config);
  bool initFreeMotionCtrl(YAML::Node config);
  bool initTossVar(YAML::Node config);
  bool initDesTasksPosAndLimits(YAML::Node config);
  bool initDampingTopicCtrl(YAML::Node config);
  bool initTossParamEstimator(const std::string pathLearnedModelfolder);
  bool initDSThrowing();
  bool init();

  void reset();

  bool loadParamFromFile(const std::string path_to_yaml_file, const std::string pathLearnedModelfolder);
  bool updateSim(Eigen::Matrix<float, 6, 1> robotWrench,
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
                 Vector7f jointTorques);

  void generateCommands(float firstEigenPassiveDamping[],
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
                        Vector7f jointTorques);
  void updateContactState();
  void computeCommands(Eigen::Vector3f eePose, Eigen::Vector4f eeOrientation);

  Eigen::Vector3f
  computeInterceptWithTarget(const Eigen::Vector3f& x_target, const Eigen::Vector3f& v_target, float phi_i);
  void findDesiredLandingPosition(bool isPlacing, bool isPlaceTossing, bool isThrowing);
  float getDesiredYawAngleTarget(const Eigen::Vector4f& qt, const Eigen::Vector3f& ang_lim);
  void estimateTargetStateToGo(Eigen::Vector2f lengthPathAvgSpeedRobot,
                               Eigen::Vector2f lengthPathAvgSpeedTarget,
                               float flyTimeObj);
  void updateInterceptPosition(float flyTimeObj, float intercep_limits[]);
  void findReleaseConfiguration();
  void setReleaseState();
  void set2DPositionBoxConstraints(Eigen::Vector3f& position_vec, float limits[]);
  void computeAdaptationFactors(Eigen::Vector2f lengthPathAvgSpeedRobot,
                                Eigen::Vector2f lengthPathAvgSpeedTarget,
                                float flyTimeObj);
  Eigen::Vector3f
  getImpactDirection(Eigen::Vector3f objectDesiredForce, Eigen::Vector3f objNormal, float coeffFriction);
  void mirrorTargetToObjectOrientation(Eigen::Vector4f qt, Eigen::Vector4f& qo, Eigen::Vector3f ang_lim);
  void prepareCommands(Vector6f Vd_ee[], Eigen::Vector4f qd[], Vector6f V_gpo[]);

  bool getReleaseFlag();
  double getPeriod();
};