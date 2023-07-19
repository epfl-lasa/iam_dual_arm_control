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

#include "ros/ros.h"
#include <iostream>
#include <ros/package.h>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/WrenchStamped.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int32.h"

#include "dual_arm_control_iam/tools/Utils.hpp"

#define NB_ROBOTS 2// Number of robots

typedef Eigen::Matrix<float, 7, 1> Vector7f;
typedef Eigen::Matrix<float, 6, 1> Vector6f;

struct ConveyorBeltState {
  bool ctrlModeConveyorBelt;
  bool isDisturbTarget;
  int modeConveyorBelt;
  int nominalSpeedConveyorBelt;
  int magniturePertConveyorBelt;
};

class RosDualArmCommunication {
private:
  // ---- ROS
  ros::NodeHandle nh_;// Ros node handle
  ros::Rate loopRate_;// Ros loop rate [Hz]
  double frequency_;

  bool isSimulation_ = true;
  std::vector<float> objectDimVect_;

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
  ros::Publisher pubTSCommands_[NB_ROBOTS];    // Publisher of the End effectors velocity twist
  ros::Publisher pubDesiredVelQuat_[NB_ROBOTS];// Publish desired EE linear velocity and quaternion

  ros::Publisher pubDesiredTwist_[NB_ROBOTS];      // Publish desired twist to DS-impdedance controller
  ros::Publisher pubDesiredOrientation_[NB_ROBOTS];// Publish desired orientation to DS-impedance controller
  ros::Publisher pubFilteredWrench_[NB_ROBOTS];    // Publish filtered measured wrench
  ros::Publisher pubNormalForce_[NB_ROBOTS];       // Publish measured normal force to the surface
  ros::Publisher pubDistAttractorEE_[NB_ROBOTS];
  ros::Publisher pubAttractor_[NB_ROBOTS];
  ros::Publisher pubNormLinVel_[NB_ROBOTS];        // Publish norms of EE linear velocities
  ros::Publisher pubAppliedWrench_[NB_ROBOTS];     // Publish applied EE wrench
  ros::Publisher pubAppliedFNormMoment_[NB_ROBOTS];// Publish the contact normal and the moment of the applied wrench
  ros::Publisher pubConveyorBeltMode_;             // Publish conveyor belt mode
  ros::Publisher pubConveyorBeltSpeed_;            // Publish conveyor belt Speed

  // params update
  std::string dsDampingTopicParams_[NB_ROBOTS];

  // ---- Conveyor belt control
  bool ctrlModeConveyorBelt_;
  bool isDisturbTarget_;
  int modeConveyorBelt_;
  int nominalSpeedConveyorBelt_;
  int magniturePertConveyorBelt_;
  int desSpeedConveyorBelt_;

  // For controllers
  float firstEigenPassiveDamping_[NB_ROBOTS];
  Eigen::Vector3f eePose_[NB_ROBOTS], objectPose_, targetPose_, eeVelLin_[NB_ROBOTS], eeVelAng_[NB_ROBOTS],
      robotBasePos_[NB_ROBOTS];
  Eigen::Vector4f eeOrientation_[NB_ROBOTS], objectOrientation_, targetOrientation_, robotBaseOrientation_[NB_ROBOTS];
  Vector7f jointPosition_[NB_ROBOTS], jointVelocity_[NB_ROBOTS], jointTorques_[NB_ROBOTS];
  Eigen::Matrix<float, 6, 1> robotWrench_[NB_ROBOTS];

public:
  // Robot ID: left or right
  enum Robot { LEFT = 0, RIGHT = 1 };

  // // For controllers
  // // float firstEigenPassiveDamping_[NB_ROBOTS];
  // Eigen::Vector3f eePose_[NB_ROBOTS], objectPose_, targetPose_, eeVelLin_[NB_ROBOTS], eeVelAng_[NB_ROBOTS],
  //     robotBasePos_[NB_ROBOTS];
  // Eigen::Vector4f eeOrientation_[NB_ROBOTS], objectOrientation_, targetOrientation_, robotBaseOrientation_[NB_ROBOTS];
  // Vector7f jointPosition_[NB_ROBOTS], jointVelocity_[NB_ROBOTS], jointTorques_[NB_ROBOTS];
  // float toolOffsetFromEE_[NB_ROBOTS];

  RosDualArmCommunication(ros::NodeHandle& n, double frequency) : nh_(n), loopRate_(frequency), frequency_(frequency){};

  ~RosDualArmCommunication(){};

  bool init() {
    if (!nh_.getParam(nh_.getNamespace() + "/dual_system/simulation", isSimulation_)) {
      ROS_ERROR("Topic dual_system/simulation not found");
    }

    if (!isSimulation_) {
      std::string objectName;
      while (!nh_.getParam(nh_.getNamespace() + "/object/name", objectName)) {
        ROS_INFO("Waitinng for param: object/name");
      }
      while (!nh_.getParam(nh_.getNamespace() + "/object/" + objectName + "/dimension", objectDimVect_)) {
        ROS_INFO("Waiting for param: object dimension ");
      }
    }
    initDampingTopicCtrl();
    initRosSubscribers();
    initRosPublisher();
    initConveyorBelt();

    for (int k = 0; k < NB_ROBOTS; k++) { firstEigenPassiveDamping_[k] = 1.0f; }

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

  bool initRosSubscribers() {

    std::string topicSubEEVel[NB_ROBOTS];
    std::string topicSubJointState[NB_ROBOTS];
    std::string topicPoseRobotBase[NB_ROBOTS];
    std::string topicPoseRobotEE[NB_ROBOTS];
    std::string topicFTSensor[NB_ROBOTS];
    std::string topicPoseObject;
    std::string topicPoseTarget;

    if (!nh_.getParam(nh_.getNamespace() + "/vel/robot_ee/robot_left", topicSubEEVel[LEFT])) {
      ROS_ERROR("Topic vel/robot_ee/robot_left not found");
    }
    if (!nh_.getParam(nh_.getNamespace() + "/vel/robot_ee/robot_right", topicSubEEVel[RIGHT])) {
      ROS_ERROR("Topic vel/robot_ee/robot_right not found");
    }
    if (!nh_.getParam(nh_.getNamespace() + "/pose/joints/robot_left", topicSubJointState[LEFT])) {
      ROS_ERROR("Topic pose/joints/robot_left not found");
    }
    if (!nh_.getParam(nh_.getNamespace() + "/pose/joints/robot_right", topicSubJointState[RIGHT])) {
      ROS_ERROR("Topic pose/joints/robot_right not found");
    }
    if (!nh_.getParam(nh_.getNamespace() + "/pose/robot_base/robot_left", topicPoseRobotBase[LEFT])) {
      ROS_ERROR("Topic pose/robot_base/robot_left not found");
    }
    if (!nh_.getParam(nh_.getNamespace() + "/pose/robot_base/robot_right", topicPoseRobotBase[RIGHT])) {
      ROS_ERROR("Topic pose/robot_base/robot_right not found");
    }
    if (!nh_.getParam(nh_.getNamespace() + "/pose/robot_ee/robot_left", topicPoseRobotEE[LEFT])) {
      ROS_ERROR("Topic pose/robot_ee/robot_left not found");
    }
    if (!nh_.getParam(nh_.getNamespace() + "/pose/robot_ee/robot_right", topicPoseRobotEE[RIGHT])) {
      ROS_ERROR("Topic pose/robot_ee/robot_right not found");
    }

    if (isSimulation_) {
      if (!nh_.getParam(nh_.getNamespace() + "/ft_sensors/simulation/sensor_left", topicFTSensor[0])) {
        ROS_ERROR("Topic /ft_sensors/simulation/sensor_left not found");
      }
      if (!nh_.getParam(nh_.getNamespace() + "/ft_sensors/simulation/sensor_right", topicFTSensor[1])) {
        ROS_ERROR("Topic /ft_sensors/simulation/sensor_right not found");
      }
    } else {
      if (!nh_.getParam(nh_.getNamespace() + "/ft_sensors/real/sensor_left", topicFTSensor[0])) {
        ROS_ERROR("Topic /ft_sensors/real/sensor_left not found");
      }
      if (!nh_.getParam(nh_.getNamespace() + "/ft_sensors/real/sensor_right", topicFTSensor[1])) {
        ROS_ERROR("Topic /ft_sensors/real/sensor_right not found");
      }
    }

    if (!nh_.getParam(nh_.getNamespace() + "/pose/object", topicPoseObject)) {
      ROS_ERROR("Topic /passive_control not found");
    }
    if (!nh_.getParam(nh_.getNamespace() + "/pose/target", topicPoseTarget)) {
      ROS_ERROR("Topic /passive_control not found");
    }

    subObjectPose_ = nh_.subscribe(topicPoseObject,
                                   1,
                                   &RosDualArmCommunication::objectPoseCallback,
                                   this,
                                   ros::TransportHints().reliable().tcpNoDelay());
    subTargetPose_ = nh_.subscribe(topicPoseTarget,
                                   1,
                                   &RosDualArmCommunication::targetPoseCallback,
                                   this,
                                   ros::TransportHints().reliable().tcpNoDelay());
    subBasePoseLeft_ = nh_.subscribe<geometry_msgs::Pose>(
        topicPoseRobotBase[LEFT],
        1,
        boost::bind(&RosDualArmCommunication::updateBasePoseCallback, this, _1, LEFT),
        ros::VoidPtr(),
        ros::TransportHints().reliable().tcpNoDelay());
    subBasePoseRight_ = nh_.subscribe<geometry_msgs::Pose>(
        topicPoseRobotBase[RIGHT],
        1,
        boost::bind(&RosDualArmCommunication::updateBasePoseCallback, this, _1, RIGHT),
        ros::VoidPtr(),
        ros::TransportHints().reliable().tcpNoDelay());
    subEEPoseLeft_ =
        nh_.subscribe<geometry_msgs::Pose>(topicPoseRobotEE[LEFT],
                                           1,
                                           boost::bind(&RosDualArmCommunication::updateEEPoseCallback, this, _1, LEFT),
                                           ros::VoidPtr(),
                                           ros::TransportHints().reliable().tcpNoDelay());
    subEEPoseRight_ =
        nh_.subscribe<geometry_msgs::Pose>(topicPoseRobotEE[RIGHT],
                                           1,
                                           boost::bind(&RosDualArmCommunication::updateEEPoseCallback, this, _1, RIGHT),
                                           ros::VoidPtr(),
                                           ros::TransportHints().reliable().tcpNoDelay());
    subEEVelLeft_ = nh_.subscribe<geometry_msgs::Twist>(
        topicSubEEVel[LEFT],
        1,
        boost::bind(&RosDualArmCommunication::updateEETwistCallback, this, _1, LEFT),
        ros::VoidPtr(),
        ros::TransportHints().reliable().tcpNoDelay());
    subEEVelRight_ = nh_.subscribe<geometry_msgs::Twist>(
        topicSubEEVel[RIGHT],
        1,
        boost::bind(&RosDualArmCommunication::updateEETwistCallback, this, _1, RIGHT),
        ros::VoidPtr(),
        ros::TransportHints().reliable().tcpNoDelay());
    subForceTorqueSensorLeft_ = nh_.subscribe<geometry_msgs::WrenchStamped>(
        topicFTSensor[LEFT],
        1,
        boost::bind(&RosDualArmCommunication::updateRobotWrenchCallback, this, _1, LEFT),
        ros::VoidPtr(),
        ros::TransportHints().reliable().tcpNoDelay());
    subForceTorqueSensorRight_ = nh_.subscribe<geometry_msgs::WrenchStamped>(
        topicFTSensor[RIGHT],
        1,
        boost::bind(&RosDualArmCommunication::updateRobotWrenchCallback, this, _1, RIGHT),
        ros::VoidPtr(),
        ros::TransportHints().reliable().tcpNoDelay());
    subJointStateLeft_ = nh_.subscribe<sensor_msgs::JointState>(
        topicSubJointState[LEFT],
        1,
        boost::bind(&RosDualArmCommunication::updateRobotStatesCallback, this, _1, LEFT),
        ros::VoidPtr(),
        ros::TransportHints().reliable().tcpNoDelay());
    subJointStateRight_ = nh_.subscribe<sensor_msgs::JointState>(
        topicSubJointState[RIGHT],
        1,
        boost::bind(&RosDualArmCommunication::updateRobotStatesCallback, this, _1, RIGHT),
        ros::VoidPtr(),
        ros::TransportHints().reliable().tcpNoDelay());

    return true;
  }

  bool initRosPublisher() {
    // Commands
    std::string topicEECommands[NB_ROBOTS];
    if (!nh_.getParam(nh_.getNamespace() + "/commands/robot_ee/robot_left", topicEECommands[LEFT])) {
      ROS_ERROR("Topic pose/robot_ee/robot_left not found");
    }
    if (!nh_.getParam(nh_.getNamespace() + "/commands/robot_ee/robot_right", topicEECommands[RIGHT])) {
      ROS_ERROR("Topic pose/robot_ee/robot_right not found");
    }
    pubTSCommands_[LEFT] = nh_.advertise<std_msgs::Float64MultiArray>(topicEECommands[LEFT], 1);
    pubTSCommands_[RIGHT] = nh_.advertise<std_msgs::Float64MultiArray>(topicEECommands[RIGHT], 1);
    std::string topicDesiredVelQuat[NB_ROBOTS];
    while (!nh_.getParam(nh_.getNamespace() + "/veloctiy/quat_desired/robot_left", topicDesiredVelQuat[LEFT])) {
      ROS_INFO("Waitinng for param: veloctiy/quat_desired/robot_left ");
    }
    while (!nh_.getParam(nh_.getNamespace() + "/veloctiy/quat_desired/robot_right", topicDesiredVelQuat[RIGHT])) {
      ROS_INFO("Waitinng for param: veloctiy/quat_desired/robot_right ");
    }

    pubDesiredVelQuat_[LEFT] = nh_.advertise<geometry_msgs::Pose>(topicDesiredVelQuat[LEFT], 1);
    pubDesiredVelQuat_[RIGHT] = nh_.advertise<geometry_msgs::Pose>(topicDesiredVelQuat[RIGHT], 1);

    // Desired orientation
    std::string topicDesiredOrientation[NB_ROBOTS];
    while (!nh_.getParam(nh_.getNamespace() + "/orientation/ee_desired/robot_left", topicDesiredOrientation[LEFT])) {
      ROS_INFO("Waitinng for param: orientation/ee_desired/robot_left ");
    }
    while (!nh_.getParam(nh_.getNamespace() + "/orientation/ee_desired/robot_right", topicDesiredOrientation[RIGHT])) {
      ROS_INFO("Waitinng for param: orientation/ee_desired/robot_right ");
    }
    pubDesiredOrientation_[LEFT] = nh_.advertise<geometry_msgs::Quaternion>(topicDesiredOrientation[LEFT], 1);
    pubDesiredOrientation_[RIGHT] = nh_.advertise<geometry_msgs::Quaternion>(topicDesiredOrientation[RIGHT], 1);

    // Wrench topics
    std::string topicFilteredWrench[NB_ROBOTS], topicAppliedWrench[NB_ROBOTS];
    while (!nh_.getParam(nh_.getNamespace() + "/wrench/filtered/robot_left", topicFilteredWrench[LEFT])) {
      ROS_INFO("Waitinng for param: wrench/filtered/robot_left ");
    }
    while (!nh_.getParam(nh_.getNamespace() + "/wrench/filtered/robot_right", topicFilteredWrench[RIGHT])) {
      ROS_INFO("Waitinng for param: wrench/filtered/robot_right ");
    }
    while (!nh_.getParam(nh_.getNamespace() + "/wrench/applied/robot_left", topicAppliedWrench[LEFT])) {
      ROS_INFO("Waitinng for param: wrench/applied/robot_left ");
    }
    while (!nh_.getParam(nh_.getNamespace() + "/wrench/applied/robot_right", topicAppliedWrench[RIGHT])) {
      ROS_INFO("Waitinng for param: wrench/applied/robot_right ");
    }
    pubFilteredWrench_[LEFT] = nh_.advertise<geometry_msgs::WrenchStamped>(topicFilteredWrench[LEFT], 1);
    pubFilteredWrench_[RIGHT] = nh_.advertise<geometry_msgs::WrenchStamped>(topicFilteredWrench[RIGHT], 1);
    pubAppliedWrench_[LEFT] = nh_.advertise<geometry_msgs::Wrench>(topicAppliedWrench[LEFT], 1);
    pubAppliedWrench_[RIGHT] = nh_.advertise<geometry_msgs::Wrench>(topicAppliedWrench[RIGHT], 1);

    // Forces topics
    std::string topicNormalForce[NB_ROBOTS], topicAppliedFNormMoment[NB_ROBOTS];
    while (!nh_.getParam(nh_.getNamespace() + "/force/normal/robot_left", topicNormalForce[LEFT])) {
      ROS_INFO("Waitinng for param: force/normal/robot_left ");
    }
    while (!nh_.getParam(nh_.getNamespace() + "/force/normal/robot_right", topicNormalForce[RIGHT])) {
      ROS_INFO("Waitinng for param: force/normal/robot_right ");
    }
    while (!nh_.getParam(nh_.getNamespace() + "/force/applied_ext/robot_left", topicAppliedFNormMoment[LEFT])) {
      ROS_INFO("Waitinng for param: force/applied_ext/robot_left ");
    }
    while (!nh_.getParam(nh_.getNamespace() + "/force/applied_ext/robot_right", topicAppliedFNormMoment[RIGHT])) {
      ROS_INFO("Waitinng for param: force/applied_ext/robot_right ");
    }
    pubNormalForce_[LEFT] = nh_.advertise<std_msgs::Float64>(topicNormalForce[LEFT], 1);
    pubNormalForce_[RIGHT] = nh_.advertise<std_msgs::Float64>(topicNormalForce[RIGHT], 1);
    pubAppliedFNormMoment_[LEFT] = nh_.advertise<geometry_msgs::Wrench>(topicAppliedFNormMoment[LEFT], 1);
    pubAppliedFNormMoment_[RIGHT] = nh_.advertise<geometry_msgs::Wrench>(topicAppliedFNormMoment[RIGHT], 1);

    // Desired velocities
    std::string topicDesiredTwist[NB_ROBOTS], topicNormLinVel[NB_ROBOTS];
    while (!nh_.getParam(nh_.getNamespace() + "/veloctiy/ee_desired/robot_left", topicDesiredTwist[LEFT])) {
      ROS_INFO("Waitinng for param: veloctiy/ee_desired/robot_left ");
    }
    while (!nh_.getParam(nh_.getNamespace() + "/veloctiy/ee_desired/robot_right", topicDesiredTwist[RIGHT])) {
      ROS_INFO("Waitinng for param: veloctiy/ee_desired/robot_right ");
    }
    while (!nh_.getParam(nh_.getNamespace() + "/veloctiy/linear_vel_norm/robot_left", topicNormLinVel[LEFT])) {
      ROS_INFO("Waitinng for param: veloctiy/ee_desired/robot_left ");
    }
    while (!nh_.getParam(nh_.getNamespace() + "/veloctiy/linear_vel_norm/robot_right", topicNormLinVel[RIGHT])) {
      ROS_INFO("Waitinng for param: veloctiy/ee_desired/robot_right ");
    }

    pubDesiredTwist_[LEFT] = nh_.advertise<geometry_msgs::Twist>(topicDesiredTwist[LEFT], 1);
    pubDesiredTwist_[RIGHT] = nh_.advertise<geometry_msgs::Twist>(topicDesiredTwist[RIGHT], 1);
    pubNormLinVel_[LEFT] = nh_.advertise<std_msgs::Float64>(topicNormLinVel[LEFT], 1);
    pubNormLinVel_[RIGHT] = nh_.advertise<std_msgs::Float64>(topicNormLinVel[RIGHT], 1);

    // Attractor
    std::string topicAttractor[NB_ROBOTS], topicDistAttractorEE[NB_ROBOTS];
    while (!nh_.getParam(nh_.getNamespace() + "/attractor/pos/robot_left", topicAttractor[LEFT])) {
      ROS_INFO("Waitinng for param: attractor/pos/robot_left ");
    }
    while (!nh_.getParam(nh_.getNamespace() + "/attractor/pos/robot_right", topicAttractor[RIGHT])) {
      ROS_INFO("Waitinng for param: attractor/pos/robot_right ");
    }
    while (!nh_.getParam(nh_.getNamespace() + "/attractor/error/robot_left", topicDistAttractorEE[LEFT])) {
      ROS_INFO("Waitinng for param: attractor/error/robot_left ");
    }
    while (!nh_.getParam(nh_.getNamespace() + "/attractor/error/robot_right", topicDistAttractorEE[RIGHT])) {
      ROS_INFO("Waitinng for param: attractor/error/robot_right ");
    }
    pubAttractor_[LEFT] = nh_.advertise<geometry_msgs::Pose>(topicAttractor[LEFT], 1);
    pubAttractor_[RIGHT] = nh_.advertise<geometry_msgs::Pose>(topicAttractor[RIGHT], 1);
    pubDistAttractorEE_[LEFT] = nh_.advertise<std_msgs::Float64>(topicDistAttractorEE[LEFT], 1);
    pubDistAttractorEE_[RIGHT] = nh_.advertise<std_msgs::Float64>(topicDistAttractorEE[RIGHT], 1);

    return true;
  }

  void publishCommands(Eigen::Vector3f axisAngleDes[], Eigen::Vector3f vDes[], Eigen::Vector4f qd[]) {
    geometry_msgs::Pose vel_quat[NB_ROBOTS];
    std_msgs::Float64MultiArray pubVel[NB_ROBOTS];

    for (int k = 0; k < NB_ROBOTS; k++) {
      pubVel[k].data.clear();
      pubVel[k].data.push_back(axisAngleDes[k](0));// axis angle pose_x
      pubVel[k].data.push_back(axisAngleDes[k](1));// axis angle pose_y
      pubVel[k].data.push_back(axisAngleDes[k](2));// axis angle pose_z
      pubVel[k].data.push_back(vDes[k](0));        // linear velocity v_x
      pubVel[k].data.push_back(vDes[k](1));        // linear velocity v_y
      pubVel[k].data.push_back(vDes[k](2));        // linear velocity v_z

      vel_quat[k].position.x = vDes[k](0); // desired velocity x
      vel_quat[k].position.y = vDes[k](1); // desired velocity y
      vel_quat[k].position.z = vDes[k](2); // desired velocity z
      vel_quat[k].orientation.w = qd[k](0);// desired pose
      vel_quat[k].orientation.x = qd[k](1);
      vel_quat[k].orientation.y = qd[k](2);
      vel_quat[k].orientation.z = qd[k](3);
    }

    pubTSCommands_[LEFT].publish(pubVel[LEFT]);
    pubTSCommands_[RIGHT].publish(pubVel[RIGHT]);
    pubDesiredVelQuat_[LEFT].publish(vel_quat[LEFT]);
    pubDesiredVelQuat_[RIGHT].publish(vel_quat[RIGHT]);
  }

  void publishData(Eigen::Vector3f vDes[],
                   Eigen::Vector3f omegaDes[],
                   Eigen::Vector4f qd[],
                   Vector6f filteredWrench[],
                   Eigen::Matrix4f whgpSpecific[],
                   Vector6f velEESpecific[],
                   Vector6f appliedWrench[],
                   Eigen::Vector3f normalVectSurfObj[],
                   float err[],
                   float nuWr0) {

    for (int k = 0; k < NB_ROBOTS; k++) {
      // Publish desired twist
      geometry_msgs::Twist msgDesiredTwist;
      msgDesiredTwist.linear.x = vDes[k](0);
      msgDesiredTwist.linear.y = vDes[k](1);
      msgDesiredTwist.linear.z = vDes[k](2);
      // Convert desired end effector frame angular velocity to world frame
      msgDesiredTwist.angular.x = omegaDes[k](0);
      msgDesiredTwist.angular.y = omegaDes[k](1);
      msgDesiredTwist.angular.z = omegaDes[k](2);
      pubDesiredTwist_[k].publish(msgDesiredTwist);

      // Publish desired orientation
      geometry_msgs::Quaternion msgDesiredOrientation;
      msgDesiredOrientation.w = qd[k](0);
      msgDesiredOrientation.x = qd[k](1);
      msgDesiredOrientation.y = qd[k](2);
      msgDesiredOrientation.z = qd[k](3);
      pubDesiredOrientation_[k].publish(msgDesiredOrientation);

      // Filtered wrench
      geometry_msgs::WrenchStamped msgFilteredWrench;
      msgFilteredWrench.header.frame_id = "world";
      msgFilteredWrench.header.stamp = ros::Time::now();
      msgFilteredWrench.wrench.force.x = filteredWrench[k](0);
      msgFilteredWrench.wrench.force.y = filteredWrench[k](1);
      msgFilteredWrench.wrench.force.z = filteredWrench[k](2);
      msgFilteredWrench.wrench.torque.x = filteredWrench[k](3);
      msgFilteredWrench.wrench.torque.y = filteredWrench[k](4);
      msgFilteredWrench.wrench.torque.z = filteredWrench[k](5);
      pubFilteredWrench_[k].publish(msgFilteredWrench);

      // Distance EE - attractor
      std_msgs::Float64 msg;
      msg.data = err[k];
      pubDistAttractorEE_[k].publish(msg);

      // Attractor info
      Eigen::Matrix4f whgpSpecificK = whgpSpecific[k];
      geometry_msgs::Pose msgPose;
      msgPose.position.x = whgpSpecificK(0, 3);
      msgPose.position.y = whgpSpecificK(1, 3);
      msgPose.position.z = whgpSpecificK(2, 3);
      Eigen::Matrix3f Rgr = whgpSpecificK.block(0, 0, 3, 3);
      Eigen::Quaternionf qgr(Rgr);
      msgPose.orientation.x = qgr.x();
      msgPose.orientation.y = qgr.y();
      msgPose.orientation.z = qgr.z();
      msgPose.orientation.w = qgr.w();
      pubAttractor_[k].publish(msgPose);

      // Norm of desired velocity
      std_msgs::Float64 msgVel;
      msgVel.data = velEESpecific[k].head(3).norm();
      pubNormLinVel_[k].publish(msgVel);

      // Applied wrench
      geometry_msgs::Wrench msgAppliedWrench;
      msgAppliedWrench.force.x = -nuWr0 * appliedWrench[k](0);
      msgAppliedWrench.force.y = -nuWr0 * appliedWrench[k](1);
      msgAppliedWrench.force.z = -nuWr0 * appliedWrench[k](2);
      msgAppliedWrench.torque.x = -nuWr0 * appliedWrench[k](3);
      msgAppliedWrench.torque.y = -nuWr0 * appliedWrench[k](4);
      msgAppliedWrench.torque.z = -nuWr0 * appliedWrench[k](5);
      pubAppliedWrench_[k].publish(msgAppliedWrench);

      // Contact normal and applied moment
      geometry_msgs::Wrench msgFnormMoment;
      msgFnormMoment.force.x = normalVectSurfObj[k](0);
      msgFnormMoment.force.y = normalVectSurfObj[k](1);
      msgFnormMoment.force.z = normalVectSurfObj[k](2);
      msgFnormMoment.torque.x = -appliedWrench[k](3);
      msgFnormMoment.torque.y = -appliedWrench[k](4);
      msgFnormMoment.torque.z = -appliedWrench[k](5);
      pubAppliedFNormMoment_[k].publish(msgFnormMoment);
    }
  }

  bool initDampingTopicCtrl() {

    // Get Passive DS params from iiwa_ros
    std::string paramDampingTopicCustomCtrlLeft;
    std::string paramDampingTopicCustomCtrlRight;
    std::string paramDampingTopicTorqueCtrlLeft;
    std::string paramDampingTopicTorqueCtrlRight;

    while (!nh_.getParam("dual_system/passiveDS/dampingTopic/CustomController/left", paramDampingTopicCustomCtrlLeft)) {
      ROS_INFO("Waitinng for param : CustomController/left");
    }
    while (
        !nh_.getParam("dual_system/passiveDS/dampingTopic/CustomController/right", paramDampingTopicCustomCtrlRight)) {
      ROS_INFO("Waitinng for param : CustomController/right");
    }
    while (!nh_.getParam("dual_system/passiveDS/dampingTopic/TorqueController/left", paramDampingTopicTorqueCtrlLeft)) {
      ROS_INFO("Waitinng for param : TorqueController/left");
    }
    while (
        !nh_.getParam("dual_system/passiveDS/dampingTopic/TorqueController/right", paramDampingTopicTorqueCtrlRight)) {
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

  bool initConveyorBelt() {
    // Publisher
    std::string topicConveyorBeltMode, topicConveyorBeltSpeed;
    while (!nh_.getParam("conveyor_belt/desired_mode", topicConveyorBeltMode)) {
      ROS_INFO("Waitinng for param: conveyor_belt/desired_mode ");
    }
    while (!nh_.getParam("conveyor_belt/desired_speed", topicConveyorBeltSpeed)) {
      ROS_INFO("Waitinng for param: conveyor_belt/desired_speed ");
    }
    pubConveyorBeltMode_ = nh_.advertise<std_msgs::Int32>(topicConveyorBeltMode, 1);
    pubConveyorBeltSpeed_ = nh_.advertise<std_msgs::Int32>(topicConveyorBeltSpeed, 1);

    // Params
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

    modeConveyorBelt_ = 0;
    isDisturbTarget_ = false;

    return true;
  }

  // ---- Callback functions

  void objectPoseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    Eigen::Vector3f xom, tXoXom;

    if (!isSimulation_) {
      tXoXom << 0.0f, 0.0f, -objectDimVect_[2] / 2.0f;
    } else {
      tXoXom << 0.0f, 0.0f, 0.0f;
    }
    xom << msg->position.x, msg->position.y, msg->position.z;
    objectOrientation_ << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
    Eigen::Vector4f qo;
    qo << 1.0f, 0.0f, 0.0f, 0.0f;
    Eigen::Matrix3f wRo = Utils<float>::quaternionToRotationMatrix(qo);
    objectPose_ = xom + wRo * tXoXom;
  }

  void targetPoseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    targetPose_ << msg->position.x, msg->position.y, msg->position.z;
    targetOrientation_ << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
  }

  void updateBasePoseCallback(const geometry_msgs::Pose::ConstPtr& msg, int k) {
    robotBasePos_[k] = Eigen::Vector3f(msg->position.x, msg->position.y, msg->position.z);
    robotBaseOrientation_[k] =
        Eigen::Vector4f(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  }

  void updateEEPoseCallback(const geometry_msgs::Pose::ConstPtr& msg, int k) {
    // Update end effecotr pose (position+orientation)
    eePose_[k] = Eigen::Vector3f(msg->position.x, msg->position.y, msg->position.z);
    eeOrientation_[k] = Eigen::Vector4f(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  }

  void updateEETwistCallback(const geometry_msgs::Twist::ConstPtr& msg, int k) {
    eeVelLin_[k] = Eigen::Vector3f(msg->linear.x, msg->linear.y, msg->linear.z);
    eeVelAng_[k] = Eigen::Vector3f(msg->angular.x, msg->angular.y, msg->angular.z);
  }

  void updateRobotWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg, int k) {
    robotWrench_[k](0) = msg->wrench.force.x;
    robotWrench_[k](1) = msg->wrench.force.y;
    robotWrench_[k](2) = msg->wrench.force.z;
    robotWrench_[k](3) = msg->wrench.torque.x;
    robotWrench_[k](4) = msg->wrench.torque.y;
    robotWrench_[k](5) = msg->wrench.torque.z;
  }

  void updateRobotStatesCallback(const sensor_msgs::JointState::ConstPtr& msg, int k) {

    for (int i = 0; i < 7; i++) {
      jointPosition_[k](i) = (float) msg->position[i];
      jointVelocity_[k](i) = (float) msg->velocity[i];
      jointTorques_[k](i) = (float) msg->effort[i];
    }
  }

  // ---- eigenvalue passive ds controller and its updated value
  void updatePassiveDSDamping() {
    std::vector<float> paramValues;

    ros::param::getCached(dsDampingTopicParams_[LEFT], paramValues);
    ros::param::getCached(dsDampingTopicParams_[RIGHT], paramValues);

    firstEigenPassiveDamping_[LEFT] = paramValues[0];
    if (firstEigenPassiveDamping_[LEFT] < FLT_EPSILON) { firstEigenPassiveDamping_[LEFT] = 150.0f; }

    firstEigenPassiveDamping_[RIGHT] = paramValues[0];
    if (firstEigenPassiveDamping_[RIGHT] < FLT_EPSILON) { firstEigenPassiveDamping_[RIGHT] = 150.0f; }
  }

  // ---- Conveyor belt control
  ConveyorBeltState getConveyorBeltStatus() {
    ConveyorBeltState conveyorBeltState;
    conveyorBeltState.ctrlModeConveyorBelt = ctrlModeConveyorBelt_;
    conveyorBeltState.modeConveyorBelt = modeConveyorBelt_;

    conveyorBeltState.nominalSpeedConveyorBelt = nominalSpeedConveyorBelt_;
    conveyorBeltState.magniturePertConveyorBelt = magniturePertConveyorBelt_;
    conveyorBeltState.isDisturbTarget = isDisturbTarget_;

    return conveyorBeltState;
  }

  void updateConveyorBeltStatus(ConveyorBeltState conveyorBeltState) {
    nominalSpeedConveyorBelt_ = conveyorBeltState.nominalSpeedConveyorBelt;
    magniturePertConveyorBelt_ = conveyorBeltState.magniturePertConveyorBelt;
    isDisturbTarget_ = conveyorBeltState.isDisturbTarget;

    if (ctrlModeConveyorBelt_ && modeConveyorBelt_ != conveyorBeltState.modeConveyorBelt) {
      modeConveyorBelt_ = conveyorBeltState.modeConveyorBelt;
      sendConveyorBeltMode();
    }
  }

  void sendConveyorBeltMode() {
    std_msgs::Int32 modeMessage;
    modeMessage.data = modeConveyorBelt_;
    pubConveyorBeltMode_.publish(modeMessage);
  }

  void sendConveyorBeltSpeed(int cycleCount) {
    // Compute conveyor belt speed command
    float omegaPert = 2.f * M_PI / 1;
    float deltaOmegaPert = 0.1f * (2.f * (float) std::rand() / RAND_MAX - 1.0f) * omegaPert;

    if (ctrlModeConveyorBelt_) {
      desSpeedConveyorBelt_ = (int) (nominalSpeedConveyorBelt_
                                     + (int) isDisturbTarget_ * magniturePertConveyorBelt_
                                         * sin((omegaPert + deltaOmegaPert) * (1 / frequency_) * cycleCount));

      // Send speed command to the conveyor belt
      std_msgs::Int32 speedMsgConveyorBelt;
      speedMsgConveyorBelt.data = desSpeedConveyorBelt_;
      if (ctrlModeConveyorBelt_ && (fmod(cycleCount, 30) == 0)) { pubConveyorBeltSpeed_.publish(speedMsgConveyorBelt); }
    }
  }

  float* getFirstEigenPassiveDamping() { return firstEigenPassiveDamping_; }
  Eigen::Vector3f getObjectPose() { return objectPose_; }
  Eigen::Vector3f getTargetPose() { return targetPose_; }
  Eigen::Vector3f* getEePose() { return eePose_; }
  Eigen::Vector3f* getEeVelLin() { return eeVelLin_; }
  Eigen::Vector3f* getEeVelAng() { return eeVelAng_; }
  Eigen::Vector3f* getRobotBasePos() { return robotBasePos_; }
  Eigen::Vector4f getObjectOrientation() { return objectOrientation_; }
  Eigen::Vector4f getTargetOrientation() { return targetOrientation_; }
  Eigen::Vector4f* getEeOrientation() { return eeOrientation_; }
  Eigen::Vector4f* getRobotBaseOrientation() { return robotBaseOrientation_; }
  Eigen::Matrix<float, 6, 1>* getRobotWrench() { return robotWrench_; }
  Vector7f* getJointPosition() { return jointPosition_; }
  Vector7f* getJointVelocity() { return jointVelocity_; }
  Vector7f* getJointTorques() { return jointTorques_; }
};