#pragma once

#include "ros/ros.h"
#include <iostream>
#include <ros/package.h>

class RosDualArmCommunication {

  // private:
  //   // ---- ROS
  //   ros::NodeHandle nh_;// Ros node handle
  //   ros::Rate loopRate_;// Ros loop rate [Hz]
  //   SGF::real dt_;

  //   // Velocity commands to be sent to the robots
  //   std_msgs::Float64MultiArray pubVel_[NB_ROBOTS];

  //   // ---- Subscriber
  //   ros::Subscriber subObjectPose_;
  //   ros::Subscriber subTargetPose_;
  //   ros::Subscriber subBasePoseLeft_;
  //   ros::Subscriber subBasePoseRight_;
  //   ros::Subscriber subEEPoseLeft_;
  //   ros::Subscriber subEEPoseRight_;
  //   ros::Subscriber subEEVelLeft_;
  //   ros::Subscriber subEEVelRight_;
  //   ros::Subscriber subForceTorqueSensorLeft_;
  //   ros::Subscriber subForceTorqueSensorRight_;
  //   ros::Subscriber subJointStateLeft_;
  //   ros::Subscriber subJointStateRight_;

  //   // ---- Publishers:
  //   ros::Publisher pubTSCommands_[NB_ROBOTS];        // Publisher of the End effectors velocity twist
  //   ros::Publisher pubDesiredTwist_[NB_ROBOTS];      // Publish desired twist to DS-impdedance controller
  //   ros::Publisher pubDesiredOrientation_[NB_ROBOTS];// Publish desired orientation to DS-impedance controller
  //   ros::Publisher pubFilteredWrench_[NB_ROBOTS];    // Publish filtered measured wrench
  //   ros::Publisher pubNormalForce_[NB_ROBOTS];       // Publish measured normal force to the surface
  //   ros::Publisher pubDesiredVelQuat_[NB_ROBOTS];    // Publish desired EE linear velocity and quaternion
  //   ros::Publisher pubDistAttractorEE_[NB_ROBOTS];
  //   ros::Publisher pubAttractor_[NB_ROBOTS];
  //   ros::Publisher pubNormLinVel_[NB_ROBOTS];        // Publish norms of EE linear velocities
  //   ros::Publisher pubAppliedWrench_[NB_ROBOTS];     // Publish applied EE wrench
  //   ros::Publisher pubAppliedFNormMoment_[NB_ROBOTS];// Publish the contact normal and the moment of the applied wrench
  //   ros::Publisher pubConveyorBeltMode_;             // Publish conveyor belt mode
  //   ros::Publisher pubConveyorBeltSpeed_;            // Publish conveyor belt Speed

  // public:
  //   RosDualArmCommunication(ros::NodeHandle& n,
  //                           double frequency,
  //                           std::string topicPoseObject,
  //                           std::string topicPoseRobotBase[],
  //                           std::string topicPoseRobotEE[],
  //                           std::string topicEECommands[],
  //                           std::string topicFTSensorSub[]) :
  //       nh_(n),
  //       loopRate_(frequency), dt_(1.0f / frequency){};

  //   ~RosDualArmCommunication();

  //   bool initRosSubscribers() {

  //     std::string topicSubEEVel[NB_ROBOTS];
  //     std::string topicSubJointState[NB_ROBOTS];

  //     if (!nh_.getParam("vel/robot_ee/robot_left", topicSubEEVel[LEFT])) {
  //       ROS_ERROR("Topic vel/robot_ee/robot_left not found");
  //     }
  //     if (!nh_.getParam("vel/robot_ee/robot_right", topicSubEEVel[RIGHT])) {
  //       ROS_ERROR("Topic vel/robot_ee/robot_right not found");
  //     }
  //     if (!nh_.getParam("pose/joints/robot_left", topicSubJointState[LEFT])) {
  //       ROS_ERROR("Topic pose/joints/robot_left not found");
  //     }
  //     if (!nh_.getParam("pose/joints/robot_right", topicSubJointState[RIGHT])) {
  //       ROS_ERROR("Topic pose/joints/robot_right not found");
  //     }

  //     subObjectPose_ = nh_.subscribe(topicPoseObject_,
  //                                    1,
  //                                    &DualArmControl::objectPoseCallback,
  //                                    this,
  //                                    ros::TransportHints().reliable().tcpNoDelay());
  //     subTargetPose_ = nh_.subscribe(topic_pose_target_,
  //                                    1,
  //                                    &DualArmControl::targetPoseCallback,
  //                                    this,
  //                                    ros::TransportHints().reliable().tcpNoDelay());
  //     subBasePoseLeft_ =
  //         nh_.subscribe<geometry_msgs::Pose>(topicPoseRobotBase_[LEFT],
  //                                            1,
  //                                            boost::bind(&DualArmControl::updateBasePoseCallback, this, _1, LEFT),
  //                                            ros::VoidPtr(),
  //                                            ros::TransportHints().reliable().tcpNoDelay());
  //     subBasePoseRight_ =
  //         nh_.subscribe<geometry_msgs::Pose>(topicPoseRobotBase_[RIGHT],
  //                                            1,
  //                                            boost::bind(&DualArmControl::updateBasePoseCallback, this, _1, RIGHT),
  //                                            ros::VoidPtr(),
  //                                            ros::TransportHints().reliable().tcpNoDelay());
  //     subEEPoseLeft_ =
  //         nh_.subscribe<geometry_msgs::Pose>(topicPoseRobotEE_[LEFT],
  //                                            1,
  //                                            boost::bind(&DualArmControl::updateEEPoseCallback, this, _1, LEFT),
  //                                            ros::VoidPtr(),
  //                                            ros::TransportHints().reliable().tcpNoDelay());
  //     subEEPoseRight_ =
  //         nh_.subscribe<geometry_msgs::Pose>(topicPoseRobotEE_[RIGHT],
  //                                            1,
  //                                            boost::bind(&DualArmControl::updateEEPoseCallback, this, _1, RIGHT),
  //                                            ros::VoidPtr(),
  //                                            ros::TransportHints().reliable().tcpNoDelay());
  //     subEEVelLeft_ =
  //         nh_.subscribe<geometry_msgs::Twist>(topicSubEEVel[LEFT],
  //                                             1,
  //                                             boost::bind(&DualArmControl::updateEETwistCallback, this, _1, LEFT),
  //                                             ros::VoidPtr(),
  //                                             ros::TransportHints().reliable().tcpNoDelay());
  //     subEEVelRight_ =
  //         nh_.subscribe<geometry_msgs::Twist>(topicSubEEVel[RIGHT],
  //                                             1,
  //                                             boost::bind(&DualArmControl::updateEETwistCallback, this, _1, RIGHT),
  //                                             ros::VoidPtr(),
  //                                             ros::TransportHints().reliable().tcpNoDelay());
  //     subForceTorqueSensorLeft_ = nh_.subscribe<geometry_msgs::WrenchStamped>(
  //         topicFTSensor_[LEFT],
  //         1,
  //         boost::bind(&DualArmControl::updateRobotWrenchCallback, this, _1, LEFT),
  //         ros::VoidPtr(),
  //         ros::TransportHints().reliable().tcpNoDelay());
  //     subForceTorqueSensorRight_ = nh_.subscribe<geometry_msgs::WrenchStamped>(
  //         topicFTSensor_[RIGHT],
  //         1,
  //         boost::bind(&DualArmControl::updateRobotWrenchCallback, this, _1, RIGHT),
  //         ros::VoidPtr(),
  //         ros::TransportHints().reliable().tcpNoDelay());
  //     subJointStateLeft_ =
  //         nh_.subscribe<sensor_msgs::JointState>(topicSubJointState[LEFT],
  //                                                1,
  //                                                boost::bind(&DualArmControl::updateRobotStatesCallback, this, _1, LEFT),
  //                                                ros::VoidPtr(),
  //                                                ros::TransportHints().reliable().tcpNoDelay());
  //     subJointStateRight_ =
  //         nh_.subscribe<sensor_msgs::JointState>(topicSubJointState[RIGHT],
  //                                                1,
  //                                                boost::bind(&DualArmControl::updateRobotStatesCallback, this, _1, RIGHT),
  //                                                ros::VoidPtr(),
  //                                                ros::TransportHints().reliable().tcpNoDelay());

  //     return true;
  //   }

  //   bool initRosPublisher() {
  //     // Commands
  //     pubTSCommands_[LEFT] = nh_.advertise<std_msgs::Float64MultiArray>(topicEECommands_[LEFT], 1);
  //     pubTSCommands_[RIGHT] = nh_.advertise<std_msgs::Float64MultiArray>(topicEECommands_[RIGHT], 1);

  //     // Desired orientation
  //     std::string topicDesiredOrientation[NB_ROBOTS];
  //     while (!nh_.getParam("orientation/ee_desired/robot_left", topicDesiredOrientation[LEFT])) {
  //       ROS_INFO("Waitinng for param: orientation/ee_desired/robot_left ");
  //     }
  //     while (!nh_.getParam("orientation/ee_desired/robot_right", topicDesiredOrientation[RIGHT])) {
  //       ROS_INFO("Waitinng for param: orientation/ee_desired/robot_right ");
  //     }
  //     pubDesiredOrientation_[LEFT] = nh_.advertise<geometry_msgs::Quaternion>(topicDesiredOrientation[LEFT], 1);
  //     pubDesiredOrientation_[RIGHT] = nh_.advertise<geometry_msgs::Quaternion>(topicDesiredOrientation[RIGHT], 1);

  //     // Wrench topics
  //     std::string topicFilteredWrench[NB_ROBOTS], topicAppliedWrench[NB_ROBOTS];
  //     while (!nh_.getParam("wrench/filtered/robot_left", topicFilteredWrench[LEFT])) {
  //       ROS_INFO("Waitinng for param: wrench/filtered/robot_left ");
  //     }
  //     while (!nh_.getParam("wrench/filtered/robot_right", topicFilteredWrench[RIGHT])) {
  //       ROS_INFO("Waitinng for param: wrench/filtered/robot_right ");
  //     }
  //     while (!nh_.getParam("wrench/applied/robot_left", topicAppliedWrench[LEFT])) {
  //       ROS_INFO("Waitinng for param: wrench/applied/robot_left ");
  //     }
  //     while (!nh_.getParam("wrench/applied/robot_right", topicAppliedWrench[RIGHT])) {
  //       ROS_INFO("Waitinng for param: wrench/applied/robot_right ");
  //     }
  //     pubFilteredWrench_[LEFT] = nh_.advertise<geometry_msgs::WrenchStamped>(topicFilteredWrench[LEFT], 1);
  //     pubFilteredWrench_[RIGHT] = nh_.advertise<geometry_msgs::WrenchStamped>(topicFilteredWrench[RIGHT], 1);
  //     pubAppliedWrench_[LEFT] = nh_.advertise<geometry_msgs::Wrench>(topicAppliedWrench[LEFT], 1);
  //     pubAppliedWrench_[RIGHT] = nh_.advertise<geometry_msgs::Wrench>(topicAppliedWrench[RIGHT], 1);

  //     // Forces topics
  //     std::string topicNormalForce[NB_ROBOTS], topicAppliedFNormMoment[NB_ROBOTS];
  //     while (!nh_.getParam("force/normal/robot_left", topicNormalForce[LEFT])) {
  //       ROS_INFO("Waitinng for param: force/normal/robot_left ");
  //     }
  //     while (!nh_.getParam("force/normal/robot_right", topicNormalForce[RIGHT])) {
  //       ROS_INFO("Waitinng for param: force/normal/robot_right ");
  //     }
  //     while (!nh_.getParam("force/applied_ext/robot_left", topicAppliedFNormMoment[LEFT])) {
  //       ROS_INFO("Waitinng for param: force/applied_ext/robot_left ");
  //     }
  //     while (!nh_.getParam("force/applied_ext/robot_right", topicAppliedFNormMoment[RIGHT])) {
  //       ROS_INFO("Waitinng for param: force/applied_ext/robot_right ");
  //     }
  //     pubNormalForce_[LEFT] = nh_.advertise<std_msgs::Float64>(topicNormalForce[LEFT], 1);
  //     pubNormalForce_[RIGHT] = nh_.advertise<std_msgs::Float64>(topicNormalForce[RIGHT], 1);
  //     pubAppliedFNormMoment_[LEFT] = nh_.advertise<geometry_msgs::Wrench>(topicAppliedFNormMoment[LEFT], 1);
  //     pubAppliedFNormMoment_[RIGHT] = nh_.advertise<geometry_msgs::Wrench>(topicAppliedFNormMoment[RIGHT], 1);

  //     // Desired velocities
  //     std::string topicDesiredVelQuat[NB_ROBOTS], topicDesiredTwist[NB_ROBOTS], topicNormLinVel[NB_ROBOTS];
  //     while (!nh_.getParam("veloctiy/quat_desired/robot_left", topicDesiredVelQuat[LEFT])) {
  //       ROS_INFO("Waitinng for param: veloctiy/quat_desired/robot_left ");
  //     }
  //     while (!nh_.getParam("veloctiy/quat_desired/robot_right", topicDesiredVelQuat[RIGHT])) {
  //       ROS_INFO("Waitinng for param: veloctiy/quat_desired/robot_right ");
  //     }
  //     while (!nh_.getParam("veloctiy/ee_desired/robot_left", topicDesiredTwist[LEFT])) {
  //       ROS_INFO("Waitinng for param: veloctiy/ee_desired/robot_left ");
  //     }
  //     while (!nh_.getParam("veloctiy/ee_desired/robot_right", topicDesiredTwist[RIGHT])) {
  //       ROS_INFO("Waitinng for param: veloctiy/ee_desired/robot_right ");
  //     }
  //     while (!nh_.getParam("veloctiy/linear_vel_norm/robot_left", topicNormLinVel[LEFT])) {
  //       ROS_INFO("Waitinng for param: veloctiy/ee_desired/robot_left ");
  //     }
  //     while (!nh_.getParam("veloctiy/linear_vel_norm/robot_right", topicNormLinVel[RIGHT])) {
  //       ROS_INFO("Waitinng for param: veloctiy/ee_desired/robot_right ");
  //     }
  //     pubDesiredVelQuat_[LEFT] = nh_.advertise<geometry_msgs::Pose>(topicDesiredVelQuat[LEFT], 1);
  //     pubDesiredVelQuat_[RIGHT] = nh_.advertise<geometry_msgs::Pose>(topicDesiredVelQuat[RIGHT], 1);
  //     pubDesiredTwist_[LEFT] = nh_.advertise<geometry_msgs::Twist>(topicDesiredTwist[LEFT], 1);
  //     pubDesiredTwist_[RIGHT] = nh_.advertise<geometry_msgs::Twist>(topicDesiredTwist[RIGHT], 1);
  //     pubNormLinVel_[LEFT] = nh_.advertise<std_msgs::Float64>(topicNormLinVel[LEFT], 1);
  //     pubNormLinVel_[RIGHT] = nh_.advertise<std_msgs::Float64>(topicNormLinVel[RIGHT], 1);

  //     // Attractor
  //     std::string topicAttractor[NB_ROBOTS], topicDistAttractorEE[NB_ROBOTS];
  //     while (!nh_.getParam("attractor/pos/robot_left", topicAttractor[LEFT])) {
  //       ROS_INFO("Waitinng for param: attractor/pos/robot_left ");
  //     }
  //     while (!nh_.getParam("attractor/pos/robot_right", topicAttractor[RIGHT])) {
  //       ROS_INFO("Waitinng for param: attractor/pos/robot_right ");
  //     }
  //     while (!nh_.getParam("attractor/error/robot_left", topicDistAttractorEE[LEFT])) {
  //       ROS_INFO("Waitinng for param: attractor/error/robot_left ");
  //     }
  //     while (!nh_.getParam("attractor/error/robot_right", topicDistAttractorEE[RIGHT])) {
  //       ROS_INFO("Waitinng for param: attractor/error/robot_right ");
  //     }
  //     pubAttractor_[LEFT] = nh_.advertise<geometry_msgs::Pose>(topicAttractor[LEFT], 1);
  //     pubAttractor_[RIGHT] = nh_.advertise<geometry_msgs::Pose>(topicAttractor[RIGHT], 1);
  //     pubDistAttractorEE_[LEFT] = nh_.advertise<std_msgs::Float64>(topicDistAttractorEE[LEFT], 1);
  //     pubDistAttractorEE_[RIGHT] = nh_.advertise<std_msgs::Float64>(topicDistAttractorEE[RIGHT], 1);

  //     // Conveyor Belt
  //     std::string topicConveyorBeltMode, topicConveyorBeltSpeed;
  //     while (!nh_.getParam("conveyor_belt/desired_mode", topicConveyorBeltMode)) {
  //       ROS_INFO("Waitinng for param: conveyor_belt/desired_mode ");
  //     }
  //     while (!nh_.getParam("conveyor_belt/desired_speed", topicConveyorBeltSpeed)) {
  //       ROS_INFO("Waitinng for param: conveyor_belt/desired_speed ");
  //     }
  //     pubConveyorBeltMode_ = nh_.advertise<std_msgs::Int32>(topicConveyorBeltMode, 1);
  //     pubConveyorBeltSpeed_ = nh_.advertise<std_msgs::Int32>(topicConveyorBeltSpeed, 1);

  //     return true;
  //   }
};