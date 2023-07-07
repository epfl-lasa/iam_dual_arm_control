/*
 * Copyright (C) 2018 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Authors: Mahdi Khoramshahi and Nadia Figueroa
 * email:   {mahdi.khoramshahi,nadia.figueroafernandez}@epfl.ch
 * website: lasa.epfl.ch
 *
 * This work was supported by the European Communitys Horizon 2020 Research and Innovation 
 * programme ICT-23-2014, grant agreement 644727-Cogimon and 643950-SecondHands.
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

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

struct tossingTaskVariables {

  Eigen::Vector3f releasePosition;
  Eigen::Vector4f releaseOrientation;
  Eigen::Vector3f releaseLinearVelocity;
  Eigen::Vector3f releaseAngularVelocity;
  Eigen::Vector3f restPosition;
  Eigen::Vector4f restOrientation;
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
  // TASK ID: Reaching, tossing, going to retract
  enum TASK_ID { REACH = 0, TOSS = 1, RETRACT = 2 };
  // MOTION ID
  enum MOTION_ID { TRANSLATION = 0, ROTATION = 1 };

  ThrowingDS();
  ~ThrowingDS();

  bool init(float modulRegion[],
            Eigen::Matrix3f Kp[],
            Eigen::Matrix3f Dp[],
            Eigen::Matrix3f Ko[],
            Eigen::Matrix3f Do[],
            bool is2ndOrder);

  bool init(tossDsParam ds_param,
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
                                         Eigen::Vector3f Areach_ee,
                                         Eigen::Vector3f Amodul_ee_norm,
                                         Eigen::Vector3f Amodul_ee_tang);
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
