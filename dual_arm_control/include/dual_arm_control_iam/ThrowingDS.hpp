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

#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdio.h>
#include <vector>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include "dual_arm_control_iam/tools/Utils.hpp"

typedef Eigen::Matrix<float, 6, 1> Vector6f;

struct tossDsParam {
  float modulRegion[3]; // modulation parameters: distances [0] = radial, [1] = normal, [2] = tangent
  Eigen::Matrix3f Kp[3];// Stiffness gains for position [0] : reaching  [1]: tossing [2]: retraction
  Eigen::Matrix3f Dp[3];// Damping gains   for position [0] : reaching  [1]: tossing [2]: retraction
  Eigen::Matrix3f Ko[3];// Stiffness gains for orienation [0] : reaching  [1]: tossing [2]: retraction
  Eigen::Matrix3f Do[3];// Damping gains   for orienation [0] : reaching  [1]: tossing [2]: retraction
  bool is2ndOrder;      // boolean for the type of DS  True: second order, false: first order
};

class ThrowingDS {
private:
  // nominal DS
  Eigen::Matrix3f
      Kp_[3];// Stifness gains for position of reaching task    [0], modulated motion task, [1] go to retract task [2]
  Eigen::Matrix3f
      Dp_[3];// Damping gains  for position of reaching task    [0], modulated motion task, [1] go to retract task [2]
  Eigen::Matrix3f
      Ko_[3];// Stifness gains for orientation of reaching task [0], modulated motion task, [1] go to retract task [2]
  Eigen::Matrix3f
      Do_[3];// Damping gains  for orientation of reaching task [0], modulated motion task, [1] go to retract task [2]

  float rho_;
  float rangeNorm_;
  float rangeTang_;

  float swProxim_;
  float swNorm_;
  float swTang_;

  bool is2ndOrder_;

  Eigen::Matrix4f wHde_;
  Eigen::Matrix4f wHre_;
  Eigen::Matrix4f wHpo_;

  Eigen::Matrix3f basisQp_;
  Eigen::Matrix3f basisQo_;

  bool releaseFlag_;

  Eigen::Vector3f vToss_;
  Eigen::Vector3f wToss_;

  tossDsParam dsParam_;

  float activationProximity_;
  float activationNormal_;
  float activationTangent_;
  float activationRetract_;

  float coupling_;
  float refVtoss_;
  float activationToss_;
  float vMax_;
  float wMax_;
  bool stopAndToss_ = false;
  Eigen::Vector3f Xt_;

public:
  /** 
   * TASK ID: 0=Reaching, 1=Tossing, 2=Going to retract
  */
  enum TASK_ID { REACH = 0, TOSS = 1, RETRACT = 2 };

  /** 
   * MOTION_ID: 0=Translation, 1=Rotation
  */
  enum MOTION_ID { TRANSLATION = 0, ROTATION = 1 };

  ThrowingDS();
  ~ThrowingDS();

  bool init(float modulRegion[],
            Eigen::Matrix3f Kp[],
            Eigen::Matrix3f Dp[],
            Eigen::Matrix3f Ko[],
            Eigen::Matrix3f Do[],
            bool is2ndOrder);

  bool init(tossDsParam dsParam,
            Eigen::Vector3f releasePos,
            Eigen::Vector4f releaseOrient,
            Eigen::Vector3f releaseLinVel,
            Eigen::Vector3f releaseAngVel,
            Eigen::Vector3f restPos,
            Eigen::Vector4f restOrient);

  /**
		 * @brief parameters for the throwing motion generation (2nd order)
		 * @param wHce homogeneous transformation of current end effector pose
		 * @param vEE current velocity twist of the end-effector
		 * @param wHde homogeneous transformation of desired end effector pose
		 * @param wHre homogeneous transformation of rest end effector pose right after throwing
		 * @param Vdtoss desired throwing velocity of the end-effector (3x1)
		 * @param releaseFlag  a boolean flag to trigger the release of the object
		 */
  Vector6f generateThrowingMotion(Eigen::Matrix4f wHce,
                                  Vector6f vEE,
                                  Eigen::Matrix4f wHde,
                                  Eigen::Matrix4f wHre,
                                  Eigen::Matrix3f basisQ,
                                  Eigen::Vector3f Vdtoss,
                                  bool& releaseFlag);

  Eigen::Vector3f computeModulatedMotion(float activation,
                                         Eigen::Matrix3f basisQ,
                                         Eigen::Vector3f AreachEE,
                                         Eigen::Vector3f AmodulEENorm,
                                         Eigen::Vector3f AmodulEETang);
  Eigen::Vector3f computeAngularMotion(float coupling,
                                       Eigen::Matrix4f wHc,
                                       Eigen::Vector3f Omega,
                                       Eigen::Matrix4f wHd,
                                       Eigen::Matrix3f Ko,
                                       Eigen::Matrix3f Do,
                                       bool is2ndOrder);
  Eigen::MatrixXf createOrthonormalMatrixFromVector(Eigen::VectorXf inVec);

  Vector6f
  apply(Eigen::Vector3f curPos, Eigen::Vector4f curOrient, Eigen::Vector3f curLinVel, Eigen::Vector3f curAngVel);

  bool setTossLinearVelocity(Eigen::Vector3f newLinVel);
  bool setTossPose(Eigen::Vector3f newReleasePos, Eigen::Vector4f newReleaseOrient);
  bool setPickupObjectPose(Eigen::Vector3f pickupPos, Eigen::Vector4f pickupOrient);
  bool resetReleaseFlag();

  tossDsParam getDsParam();
  float getActivationProximity();
  float getActivationNormal();
  float getActivationTangent();
  float getActivationToss();

  void setRefVtoss(float newValue);
};
