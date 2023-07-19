#pragma once

#include <chrono>
#include <ctime>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <pthread.h>
#include <ros/package.h>
#include <signal.h>
#include <sstream>
#include <stdio.h>
#include <termios.h>
#include <vector>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
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
#include "sg_filter.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"

#include "iam_dual_arm_control/DataLogging.hpp"
#include "iam_dual_arm_control/DualArmCooperativeController.hpp"
#include "iam_dual_arm_control/DualArmFreeMotionController.hpp"
#include "iam_dual_arm_control/ObjectToGrasp.hpp"
#include "iam_dual_arm_control/RobotVariables.hpp"
#include "iam_dual_arm_control/ThrowingDS.hpp"
#include "iam_dual_arm_control/TossTaskParamEstimator.hpp"
#include "iam_dual_arm_control/TossingTarget.hpp"
#include "iam_dual_arm_control/tools/Keyboard.hpp"
#include "iam_dual_arm_control/tools/Utils.hpp"

#define NB_ROBOTS 2// Number of robots
#define NB_FT_SENSOR_SAMPLES                                                                                           \
  50// Number of force torque sensors' samples used for initial calibration (compute the offsets)
#define NB_OPTITRACK_SAMPLES 100   // Number of optitrack samples used for initial objects' pose estimation
#define NB_TRACKED_OBJECTS 6       // Number of objects tracked by the motion capture system (optitrack)
#define MOVING_FORCE_WINDOW_SIZE 10// Window's size used to average the force data and detect peristent contact

#define NB_OBJECTS 3// Number of objects

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

class DualArmControl {
private:
  // ---- ROS
  ros::NodeHandle nh_;// Ros node handle
  ros::Rate loopRate_;// Ros loop rate [Hz]
  SGF::real dt_;

  float timeStartRun_;
  int cycleCount_;

  std::string topicPoseTarget_;

  // Velocity commands to be sent to the robots
  std_msgs::Float64MultiArray pubVel_[NB_ROBOTS];

  // ---- Subscriber
  ros::Subscriber subObjectPose_;
  ros::Subscriber subTargetPose_;
  ros::Subscriber subBasePoseLeft_;
  ros::Subscriber subBasePoseRight_;
  ros::Subscriber subEEPoseLeft_;
  ros::Subscriber subEEPoseRight_;
  ros::Subscriber subEEVelLeft_;
  ros::Subscriber subEEVelRight_;
  ros::Subscriber subForceTorqueSensorLeft_;
  ros::Subscriber subForceTorqueSensorRight_;
  ros::Subscriber subJointStateLeft_;
  ros::Subscriber subJointStateRight_;

  // ---- Publishers:
  ros::Publisher pubTSCommands_[NB_ROBOTS];        // Publisher of the End effectors velocity twist
  ros::Publisher pubDesiredTwist_[NB_ROBOTS];      // Publish desired twist to DS-impdedance controller
  ros::Publisher pubDesiredOrientation_[NB_ROBOTS];// Publish desired orientation to DS-impedance controller
  ros::Publisher pubFilteredWrench_[NB_ROBOTS];    // Publish filtered measured wrench
  ros::Publisher pubNormalForce_[NB_ROBOTS];       // Publish measured normal force to the surface
  ros::Publisher pubDesiredVelQuat_[NB_ROBOTS];    // Publish desired EE linear velocity and quaternion
  ros::Publisher pubDistAttractorEE_[NB_ROBOTS];
  ros::Publisher pubAttractor_[NB_ROBOTS];
  ros::Publisher pubNormLinVel_[NB_ROBOTS];        // Publish norms of EE linear velocities
  ros::Publisher pubAppliedWrench_[NB_ROBOTS];     // Publish applied EE wrench
  ros::Publisher pubAppliedFNormMoment_[NB_ROBOTS];// Publish the contact normal and the moment of the applied wrench
  ros::Publisher pubConveyorBeltMode_;             // Publish conveyor belt mode
  ros::Publisher pubConveyorBeltSpeed_;            // Publish conveyor belt Speed

  // ---- List of the topics
  std::string topicPoseObject_;
  std::string topicPoseRobotBase_[NB_ROBOTS];
  std::string topicPoseRobotEE_[NB_ROBOTS];
  std::string topicEECommands_[NB_ROBOTS];
  std::string topicFTSensor_[NB_ROBOTS];

  // ---- Robot
  RobotVariable robot_;
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

  std::string dsDampingTopicParams_[NB_ROBOTS];

  // ---- Object
  Eigen::Vector3f deltaPos_;// variation of object position
  Eigen::Vector3f deltaAng_;// variation of object orientation euler angles
  Eigen::Vector3f filtDeltaAng_;
  Eigen::Vector3f filtDeltaAngMir_;
  bool objCtrlKey_;

  ObjectToGrasp object_;
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
  DataLogging dataLog_;

  Eigen::Matrix3f basisQ_[NB_ROBOTS];
  Eigen::Vector3f dirImp_[NB_ROBOTS];
  Eigen::Vector3f vdImpact_[NB_ROBOTS];
  Eigen::Vector3f dualAngularLimit_;
  bool releaseFlag_;// 0=reach, 1=pick, 2=toss, 3=pick_and_toss, 4=pick_and_place

  float trackingFactor_;

  Eigen::Vector3f deltaRelPos_;
  bool incrementReleasePos_ = false;
  bool ctrlModeConveyorBelt_ = false;
  SphericalPosition releasePos_;

  int modeConveyorBelt_;
  int desSpeedConveyorBelt_;
  int nominalSpeedConveyorBelt_;
  int magniturePertConveyorBelt_;
  Eigen::Vector2f dualPathLenAvgSpeed_;
  bool hasCaughtOnce_ = false;
  bool isIntercepting_ = false;
  float betaVelMod_;
  bool isDisturbTarget_ = false;
  float initSpeedScaling_;
  std::deque<float> windowSpeedEE_;
  float movingAvgSpeedEE_;
  int winLengthAvgSpeedEE_;
  bool adaptationActive_ = false;
  bool isTargetFixed_ = true;
  bool userSelect_ = true;

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
  bool isSimulation_;

  tossingTaskVariables tossVar_;

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

public:
  std::mutex mutex;

  DualArmControl(ros::NodeHandle& n,
                 double frequency,
                 std::string topicPoseObject,
                 std::string topicPoseRobotBase[],
                 std::string topicPoseRobotEE[],
                 std::string topicEECommands[],
                 std::string topicFTSensorSub[]);
  ~DualArmControl();

  // ---- Init
  bool init();
  bool initRosSubscribers();
  bool initRosPublisher();
  bool initRobotParam();
  bool initObjectParam();
  bool initFreeMotionCtrl();
  bool initTossVar();
  bool initDesTasksPosAndLimits();
  bool initDampingTopicCtrl();
  bool initConveyorBelt();
  bool initUserInteraction();
  bool initTossParamEstimator();
  bool initDSThrowing();

  void run();
  void computeCommands();

  // ---- ROS Callbacks
  void objectPoseCallback(const geometry_msgs::Pose::ConstPtr& msg);
  void targetPoseCallback(const geometry_msgs::Pose::ConstPtr& msg);
  void updateBasePoseCallback(const geometry_msgs::Pose::ConstPtr& msg, int k);
  void updateEEPoseCallback(const geometry_msgs::Pose::ConstPtr& msg, int k);
  void updateEETwistCallback(const geometry_msgs::Twist::ConstPtr& msg, int k);
  void updateRobotWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg, int k);
  void updateContactState();
  void updateRobotStatesCallback(const sensor_msgs::JointState::ConstPtr& msg, int k);

  // ---- Publish commands and data
  void publishCommands();
  void publishData();
  void saveData();
  void publishConveyorBeltCmds();
  void printData();

  // ---- Keyboard commands
  void updateStatesMachines();
  void keyboardReferenceObjectControl();
  void keyboardVirtualObjectControl();

  float getDesiredYawAngleTarget(const Eigen::Vector4f& qt, const Eigen::Vector3f& angLimit);
  void getPassiveDSDamping();
  void prepareCommands(Vector6f vDesEE[], Eigen::Vector4f qd[], Vector6f vGpo[]);
  void updatePoses();
  void resetVariables();
  void updateReleasePosition();
  void set2DPositionBoxConstraints(Eigen::Vector3f& positionVec, float limits[]);
  void mirrorTargetToObjectOrientation(Eigen::Vector4f qt, Eigen::Vector4f& qo, Eigen::Vector3f angLimit);
  void findDesiredLandingPosition(bool isPlacing, bool isPlaceTossing, bool isThrowing);
  void updateInterceptPosition(float flyTimeObj, float intercepLimits[]);
  void findReleaseConfiguration();
  void setReleaseState();
  void estimateTargetStateToGo(Eigen::Vector2f lengthPathAvgSpeedRobot,
                               Eigen::Vector2f lengthPathAvgSpeedTarget,
                               float flyTimeObj);
  void computeAdaptationFactors(Eigen::Vector2f lengthPathAvgSpeedRobot,
                                Eigen::Vector2f lengthPathAvgSpeedTarget,
                                float flyTimeObj);
  Eigen::Vector3f
  getImpactDirection(Eigen::Vector3f objectDesiredForce, Eigen::Vector3f objNormal, float coeffFriction);
  Eigen::Vector3f
  computeInterceptWithTarget(const Eigen::Vector3f& xTarget, const Eigen::Vector3f& vTarget, float phiInit);
};
