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

#include <deque>
#include <vector>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "sg_filter.h"

#include "dual_arm_control_iam/tools/Utils.hpp"

#define NB_ROBOTS 2// Number of robots
#define NB_FT_SENSOR_SAMPLES                                                                                           \
  50// Number of force torque sensors' samples used for initial calibration (compute the offsets)
#define MOVING_FORCE_WINDOW_SIZE 10// Window's size used to average the force data and detect peristent contact

typedef Eigen::Matrix<float, 7, 1> Vector7f;

class RobotVariable {

private:
  int nbJoints_[NB_ROBOTS];
  float normalForceAverage_[NB_ROBOTS];// Average normal force measured through the force windows [N]
  float normalForce_[NB_ROBOTS];       // Normal force to the surface [N]
  Eigen::Vector3f gravity_;
  float toolMass_[NB_ROBOTS];                           // Tool mass [kg]
  float toolOffsetFromEE_[NB_ROBOTS];                   // Tool offset along z axis of end effector [m]
  Eigen::Vector3f toolComPositionFromSensor_[NB_ROBOTS];// Tool CoM offset along z axis of end effector [m]
  int wrenchCount_[NB_ROBOTS];                          // Counter used to pre-process the force data
  std::deque<float>
      normalForceWindow_[NB_ROBOTS];// Moving window saving the robots measured normal force to the object's surface [N]
  bool wrenchBiasOK_[NB_ROBOTS];    // Check if computation of force/torque sensor bias is OK

  Eigen::Vector3f x_[NB_ROBOTS];
  Eigen::Vector4f q_[NB_ROBOTS];
  Eigen::Matrix3f wRb_[NB_ROBOTS];// Orientation matrix (3x3)
  Eigen::Vector3f v_[NB_ROBOTS];
  Eigen::Vector3f w_[NB_ROBOTS];
  Eigen::Vector4f qd_[NB_ROBOTS];
  Eigen::Vector3f axisAngleDes_[NB_ROBOTS];// desired axis angle
  Eigen::Vector3f vDes_[NB_ROBOTS];
  Eigen::Vector3f omegaDes_[NB_ROBOTS];
  Vector6f vEE_[NB_ROBOTS];
  Vector6f vDesEE_[NB_ROBOTS];// desired velocity twist

  Matrix6f twistEEToolCenterPoint_
      [NB_ROBOTS];// Velocity Twist transformation between the robot EE and the tool center point (tcp)
  Vector6f vEEObstacleAvoidance_[NB_ROBOTS];// self collision (obstacle) avoidance

  Eigen::Vector3f fxc_[NB_ROBOTS];// Desired conservative parts of the nominal DS [m/s] (3x1)

  Vector6f wrench_[NB_ROBOTS];        // Wrench [N and Nm] (6x1)
  Vector6f wrenchBias_[NB_ROBOTS];    // Wrench bias [N and Nm] (6x1)
  Vector6f filteredWrench_[NB_ROBOTS];// Filtered wrench [N and Nm] (6x1)

  Eigen::Matrix4f wHRb_[NB_ROBOTS];        // Homogenenous transform of robots base frame (4x4)
  Eigen::Matrix4f rbHEEStandby_[NB_ROBOTS];// Homogenenous transform of EE standby poses relatve to robots base (4x4)
  Eigen::Vector3f xrbStandby_[NB_ROBOTS];  // quaternion orientation of EE standby poses relatve to robots base (3x1)
  Eigen::Vector4f qrbStandby_[NB_ROBOTS];  // quaternion orientation of EE standby poses relatve to robots base (4x1)
  Eigen::Matrix4f wHEE_[NB_ROBOTS];        // Homogenenous transform of the End-effectors poses (4x4)
  Eigen::Matrix4f wHEEStandby_[NB_ROBOTS]; // Homogenenous transform of Standby pose of the End-effectors (4x4)

  std::unique_ptr<SGF::SavitzkyGolayFilter> sgfDdqFilteredLeft_;
  std::unique_ptr<SGF::SavitzkyGolayFilter> sgfDdqFilteredRight_;

  Vector7f jointsPositions_[NB_ROBOTS];
  Vector7f jointsVelocities_[NB_ROBOTS];
  Vector7f jointsAccelerations_[NB_ROBOTS];
  Vector7f jointsTorques_[NB_ROBOTS];

public:
  RobotVariable() {}
  ~RobotVariable(){};

  void init(int sgfQ[], float dt, Eigen::Vector3f gravity) {

    gravity_ = gravity;

    for (int k = 0; k < NB_ROBOTS; k++) {
      nbJoints_[k] = 7;
      x_[k].setConstant(0.0f);
      q_[k].setConstant(0.0f);
      wRb_[k].setIdentity();
      wHEE_[k].setConstant(0.0f);
      wHEEStandby_[k].setConstant(0.0f);
      wHRb_[k].setIdentity();
      rbHEEStandby_[k].setIdentity();
      xrbStandby_[k].setZero();
      qrbStandby_[k].setZero();

      // Desired values
      vDesEE_[k].setZero();
      vEE_[k].setZero();
      twistEEToolCenterPoint_[k].setIdentity();
      v_[k].setZero();
      w_[k].setZero();
      vDes_[k].setZero();
      omegaDes_[k].setZero();
      qd_[k].setZero();
      axisAngleDes_[k].setZero();

      // Forces control variables
      fxc_[k].setZero();
      filteredWrench_[k].setZero();
      wrench_[k].setZero();
      wrenchBias_[k].setZero();
      normalForceAverage_[k] = 0.0f;
      wrenchCount_[k] = 0;
      normalForce_[k] = 0.0f;

      // Joint variables
      jointsPositions_[k].setZero();
      jointsVelocities_[k].setZero();
      jointsAccelerations_[k].setZero();
      jointsTorques_[k].setZero();

      vEEObstacleAvoidance_[k].setZero();
    }

    // Filtered variable (SG)
    sgfDdqFilteredLeft_ = std::make_unique<SGF::SavitzkyGolayFilter>(sgfQ[0],//dim
                                                                     sgfQ[1],//order
                                                                     sgfQ[2],//window length
                                                                     dt);
    sgfDdqFilteredRight_ = std::make_unique<SGF::SavitzkyGolayFilter>(sgfQ[0],//dim
                                                                      sgfQ[1],//order
                                                                      sgfQ[2],//window length
                                                                      dt);
  }

  void getStandbyHmgTransformInBase() {
    wHEEStandby_[0] = Utils<float>::pose2HomoMx(xrbStandby_[0], qrbStandby_[0]);
    wHEEStandby_[1] = Utils<float>::pose2HomoMx(xrbStandby_[1], qrbStandby_[1]);
  }

  void getStandbyHmgTransformInWorld() {
    wHEEStandby_[0] = wHRb_[0] * Utils<float>::pose2HomoMx(xrbStandby_[0], qrbStandby_[0]);
    wHEEStandby_[1] = wHRb_[1] * Utils<float>::pose2HomoMx(xrbStandby_[1], qrbStandby_[1]);
  }

  void getEndEffectorHmgTransform() {
    wHEE_[0] = Utils<float>::pose2HomoMx(x_[0], q_[0]);// WITH EE pose wrt. the world
    wHEE_[1] = Utils<float>::pose2HomoMx(x_[1], q_[1]);// WITH EE pose wrt. the world
  }

  void getDesiredLinTaskVelocity(float applyVelo, float nuWr0) {
    vDes_[0] = applyVelo * vDes_[0] + nuWr0 * fxc_[0];
    vDes_[1] = applyVelo * vDes_[1] + nuWr0 * fxc_[1];
  }

  void getRobotBaseFrameInWorld(Eigen::Vector3f xB, Eigen::Vector4f q, int k) {
    wHRb_[k].block(0, 3, 3, 1) = xB;
    wHRb_[k].block(0, 0, 3, 3) = Utils<float>::quaternionToRotationMatrix(q);
  }

  void updateEndEffectorPosesInWorld(Eigen::Vector3f xB, Eigen::Vector4f q, int k) {
    // update positions and Orientations of the EEs
    x_[k] = xB;
    q_[k] = q;

    // update EE positions with tool offset
    wRb_[k] = Utils<float>::quaternionToRotationMatrix(q_[k]);
    x_[k] = x_[k] + toolOffsetFromEE_[k] * wRb_[k].col(2);

    // update velocity Twist transformation from EE to tcp
    Eigen::Vector3f tcp = toolOffsetFromEE_[k] * wRb_[k].col(2);
    Eigen::Matrix3f skew_Mx_tcp;
    skew_Mx_tcp << 0.0f, -tcp(2), tcp(1), tcp(2), 0.0f, -tcp(0), -tcp(1), tcp(0), 0.0f;
    twistEEToolCenterPoint_[k].block(0, 3, 3, 3) = skew_Mx_tcp;
  }

  void updateEndEffectorVelocity(Eigen::Vector3f vE, Eigen::Vector3f wE, int k) {
    v_[k] = vE;
    w_[k] = wE;
    vEE_[k].head(3) = v_[k];
    vEE_[k].tail(3) = w_[k];
    vEE_[k] = twistEEToolCenterPoint_[k] * vEE_[k];
  }

  void updateEndEffectorWrench(Eigen::Matrix<float, 6, 1> raw,
                               Eigen::Vector3f normalObj[],
                               float filteredForceGain,
                               bool wrenchBiasOK[],
                               int k) {

    if (!wrenchBiasOK[k]) {
      Eigen::Vector3f loadForce = wRb_[k].transpose() * toolMass_[k] * gravity_;
      wrenchBias_[k].segment(0, 3) -= loadForce;
      wrenchBias_[k].segment(3, 3) -= toolComPositionFromSensor_[k].cross(loadForce);
      wrenchBias_[k] += raw;
      wrenchCount_[k]++;

      if (wrenchCount_[k] == NB_FT_SENSOR_SAMPLES) {
        wrenchBias_[k] /= NB_FT_SENSOR_SAMPLES;
        wrenchBiasOK[k] = true;
      }
    }

    if (wrenchBiasOK[k]) {
      wrench_[k] = raw - wrenchBias_[k];
      Eigen::Vector3f loadForce = wRb_[k].transpose() * toolMass_[k] * gravity_;
      wrench_[k].segment(0, 3) -= loadForce;
      wrench_[k].segment(3, 3) -= toolComPositionFromSensor_[k].cross(loadForce);
      wrench_[k].head(3) = wRb_[k] * wrench_[k].head(3);
      wrench_[k].tail(3) = wRb_[k] * wrench_[k].tail(3);
      filteredWrench_[k] = filteredForceGain * filteredWrench_[k] + (1.0f - filteredForceGain) * wrench_[k];
      //
      normalForce_[k] = fabs((filteredWrench_[k].segment(0, 3)).dot(normalObj[k]));
    }
  }

  void getEstimatedJointAccelerations(int k) {

    SGF::Vec temp_acc(nbJoints_[k]);
    if (k == 0) {
      sgfDdqFilteredLeft_->AddData(jointsVelocities_[k]);
      sgfDdqFilteredLeft_->GetOutput(1, temp_acc);
    } else {
      sgfDdqFilteredRight_->AddData(jointsVelocities_[k]);
      sgfDdqFilteredRight_->GetOutput(1, temp_acc);
    }
    jointsAccelerations_[k] = temp_acc.cast<float>();
  }

  void getEstimatedAverageNormalForce() {

    for (int k = 0; k < NB_ROBOTS; k++) {
      if (normalForceWindow_[k].size() < MOVING_FORCE_WINDOW_SIZE) {
        normalForceWindow_[k].push_back(normalForce_[k]);
        normalForceAverage_[k] = 0.0f;
      } else {
        normalForceWindow_[k].pop_front();
        normalForceWindow_[k].push_back(normalForce_[k]);
        normalForceAverage_[k] = 0.0f;
        for (int m = 0; m < MOVING_FORCE_WINDOW_SIZE; m++) { normalForceAverage_[k] += normalForceWindow_[k][m]; }
        normalForceAverage_[k] /= MOVING_FORCE_WINDOW_SIZE;
      }
    }
  }

  void setInitParameters(float toolMass_param[],
                         float toolOffsetFromEE_param[],
                         Eigen::Vector3f toolComPositionFromSensor_param[],
                         Eigen::Vector3f xrbStandby_param[],
                         Eigen::Vector4f qrbStandby_param[]) {

    memcpy(toolMass_, &toolMass_param[0], NB_ROBOTS * sizeof *toolMass_param);
    memcpy(toolOffsetFromEE_, &toolOffsetFromEE_param[0], NB_ROBOTS * sizeof *toolOffsetFromEE_param);
    memcpy(toolComPositionFromSensor_,
           &toolComPositionFromSensor_param[0],
           NB_ROBOTS * sizeof *toolComPositionFromSensor_param);
    memcpy(xrbStandby_, &xrbStandby_param[0], NB_ROBOTS * sizeof *xrbStandby_param);
    memcpy(qrbStandby_, &qrbStandby_param[0], NB_ROBOTS * sizeof *qrbStandby_param);

    // get stanby transformation of the EEs wrt. the dual-robot Base frame
    this->getStandbyHmgTransformInBase();
  }
  int getNbJoints(int robotID) { return nbJoints_[robotID]; }
  float getNormalForceAverage(int robotID) { return normalForceAverage_[robotID]; }
  float getNormalForce(int robotID) { return normalForce_[robotID]; }
  Eigen::Vector3f getX(int robotID) { return x_[robotID]; }
  Eigen::Vector4f getQ(int robotID) { return q_[robotID]; }
  Eigen::Vector4f getQdSpecific(int robotID) { return qd_[robotID]; }
  Eigen::Vector4f* getQd() { return qd_; }
  Eigen::Vector3f getAxisAngleDes(int robotID) { return axisAngleDes_[robotID]; }
  Eigen::Vector3f getVDes(int robotID) { return vDes_[robotID]; }
  Eigen::Vector3f getOmegaDes(int robotID) { return omegaDes_[robotID]; }
  Vector6f getVelEESpecific(int robotID) { return vEE_[robotID]; }
  Vector6f* getVelEE() { return vEE_; }
  Vector6f getVelDesEESpecific(int robotID) { return vDesEE_[robotID]; }
  Vector6f* getVelDesEE() { return vDesEE_; }
  Matrix6f getTwistEEToolCenterPoint(int robotID) { return twistEEToolCenterPoint_[robotID]; }
  Vector6f getVEEObstacleAvoidance(int robotID) { return vEEObstacleAvoidance_[robotID]; }
  Eigen::Vector3f getFXC(int robotID) { return fxc_[robotID]; }
  Vector6f getFilteredWrench(int robotID) { return filteredWrench_[robotID]; }
  Eigen::Matrix4f* getWHEE() { return wHEE_; }
  Eigen::Matrix4f getWHEESpecific(int robotID) { return wHEE_[robotID]; }
  Eigen::Matrix4f* getWHEEStandby() { return wHEEStandby_; }
  Eigen::Matrix4f getWHEEStandbySpecific(int robotID) { return wHEEStandby_[robotID]; }
  Vector7f getJointsPositions(int robotID) { return jointsPositions_[robotID]; }
  Vector7f getJointsVelocities(int robotID) { return jointsVelocities_[robotID]; }
  Vector7f getJointsAccelerations(int robotID) { return jointsAccelerations_[robotID]; }
  Vector7f getJointsTorques(int robotID) { return jointsTorques_[robotID]; }

  void setJointsTorques(Vector7f newValue, int robotID) { jointsTorques_[robotID] = newValue; }
  void setJointsVelocities(Vector7f newValue, int robotID) { jointsVelocities_[robotID] = newValue; }
  void setJointsPositions(Vector7f newValue, int robotID) { jointsPositions_[robotID] = newValue; }
  void setAxisAngleDes(Eigen::Vector3f newValue, int robotID) { axisAngleDes_[robotID] = newValue; }
  void setVDes(Eigen::Vector3f newValue, int robotID) { vDes_[robotID] = newValue; }
  void setOmegaDes(Eigen::Vector3f newValue, int robotID) { omegaDes_[robotID] = newValue; }
  void setFXC(Eigen::Vector3f newValue, int robotID) { fxc_[robotID] = newValue; }
  void setQd(Eigen::Vector4f newValue[]) {
    qd_[0] = newValue[0];
    qd_[1] = newValue[1];
  }
  void setVelDesEE(Vector6f newValue[]) {
    vDesEE_[0] = newValue[0];
    vDesEE_[1] = newValue[1];
  }
  void setVEEObstacleAvoidance(Vector6f newValue[]) {
    vEEObstacleAvoidance_[0] = newValue[0];
    vEEObstacleAvoidance_[1] = newValue[1];
  }
};
