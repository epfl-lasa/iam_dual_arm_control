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
#include <pthread.h>
#include <vector>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include "dual_arm_control_iam/ThrowingDS.hpp"
#include "dual_arm_control_iam/tools/Utils.hpp"

#define NB_ROBOTS 2// Number of robots

typedef Eigen::Matrix<float, 7, 1> Vector7f;
typedef Eigen::Matrix<float, 6, 1> Vector6f;
typedef Eigen::Matrix<float, 6, 6> Matrix6f;

class DualArmFreeMotionController {

private:
  Vector6f errorAbs_, errorRel_, errorObj_, velAbs_, velRel_, velObj_;

  Eigen::Matrix3f gainPosAbs_, gainOriAbs_, gainPosRel_, gainOriRel_;

  Eigen::Vector4f qdPrev_[NB_ROBOTS];
  Matrix6f tbi_;

  Vector6f twistVo_;
  Eigen::Matrix4f wHvo_, wHVgp_[NB_ROBOTS];

  float vMax_, wMax_;

  float dt_, reachableP_, gotToObject_;

  float activationProximity_, activationNormal_, activationTangent_, activationRetract_, activationRelease_,
      activationNormalDo_;

  bool releaseFlag_;

  float rho_, rangeNorm_, rangeTang_;

  float swProxim_, swNorm_, swTang_;

  Eigen::Matrix4f wHEEStandby_[NB_ROBOTS];

  float swEEObsAv_, minDistEE_, safeRadius_;
  Eigen::Vector3f omegaObjectD_, integralVeeD_[NB_ROBOTS];

  float alphaObs_[NB_ROBOTS];
  Vector6f vDesO_;

  float desVelReach_, refVelReach_[NB_ROBOTS];

  bool modulatedReaching_ = true;
  bool isNormImpactVel_ = false;

  float heightViaPoint_ = 0.25f;

  Eigen::Vector3f objectDim_;
  float activationAperture_;

public:
  // Robot ID: left or right
  enum ROBOT { LEFT = 0, RIGHT = 1 };

  DualArmFreeMotionController();
  ~DualArmFreeMotionController();

  bool init(Eigen::Matrix4f wHEEStandby[], Matrix6f gainAbs, Matrix6f gainRel);

  void computeAsyncMotion(Eigen::Matrix4f wHee[],
                          Eigen::Matrix4f wHgp[],
                          Eigen::Matrix4f wHo,
                          Vector6f vDesEE[],
                          Eigen::Vector4f qd[],
                          bool isOrient3d);
  void computeDesiredOrientation(float weight,
                                 Eigen::Matrix4f wHee[],
                                 Eigen::Matrix4f wHgp[],
                                 Eigen::Matrix4f wHo,
                                 Eigen::Vector4f qd[],
                                 bool isOrient3d);

  void computeConstrainedMotion(Eigen::Matrix4f wHee[],
                                Eigen::Matrix4f wHgp[],
                                Eigen::Matrix4f wHo,
                                Vector6f vDesEE[],
                                Eigen::Vector4f qd[],
                                bool isOrient3d);
  void computeReleaseAndRetractMotion(Eigen::Matrix4f wHee[],
                                      Eigen::Matrix4f wHgp[],
                                      Eigen::Matrix4f wHo,
                                      Vector6f vDesEE[],
                                      Eigen::Vector4f qd[],
                                      bool isOrient3d);

  void computeCoordinatedMotion2(Eigen::Matrix4f wHee[],
                                 Eigen::Matrix4f wHgp[],
                                 Eigen::Matrix4f wHo,
                                 Vector6f vDesEE[],
                                 Eigen::Vector4f qd[],
                                 bool isOrient3d);

  Eigen::Vector3f computeModulatedMotion(float activation,
                                         Eigen::Matrix3f basisQ,
                                         Eigen::Vector3f activationReachEE,
                                         Eigen::Vector3f activationEENorm,
                                         Eigen::Vector3f activationEETang);

  Vector6f computeModulatedMotionDual(float activation,
                                      Eigen::Matrix3f basisQ[],
                                      Vector6f dsEENominal,
                                      Vector6f activationEENorm,
                                      Vector6f activationEETang);

  void dualArmMotion(Eigen::Matrix4f wHee[],
                     Vector6f Vee[],
                     Eigen::Matrix4f wHgp[],
                     Eigen::Matrix4f wHo,
                     Eigen::Matrix4f wHDo,
                     Vector6f vdO,
                     Eigen::Matrix3f basisQ[],
                     Eigen::Vector3f VdImp[],
                     bool isOrient3d,
                     int taskType,
                     Vector6f vDesEE[],
                     Eigen::Vector4f qd[],
                     bool& releaseFlag);

  Eigen::Vector3f getAbsoluteTangentError(Eigen::Matrix4f wHo, Eigen::Matrix4f wHee[], Eigen::Matrix4f wHgp[]);

  void generatePlacingMotion(Eigen::Matrix4f wHee[],
                             Eigen::Matrix4f wHgp[],
                             Eigen::Matrix4f wHo,
                             Eigen::Matrix4f wHDo,
                             float viaHeight,
                             Vector6f vDesEE[],
                             Eigen::Vector4f qd[],
                             bool isOrient3d);

  Vector6f
  generatePlacingMotion2(Eigen::Matrix4f wHo, Eigen::Matrix4f wHDo, float viaHeight, Vector6f Vo, bool isPlaceTossing);

  void computeEEAvoidanceVelocity(Eigen::Matrix4f wHee[], Vector6f (&vEEOA)[NB_ROBOTS]);

  void setVirtualObjectFrame(Eigen::Matrix4f wHVo);

  void getCoordinatedTranslation(Eigen::Vector3f xEE[],
                                 Eigen::Vector3f xGP[],
                                 Eigen::Vector3f xStd[],
                                 Eigen::Matrix3f wRo,
                                 Eigen::Vector3f (&vdEE)[NB_ROBOTS]);

  Eigen::Vector2f predictRobotTranslation(Eigen::Matrix4f wHee[],
                                          Eigen::Matrix4f wHgp[],
                                          Eigen::Matrix4f wHEEStandby[],
                                          Eigen::Matrix4f wHo,
                                          Eigen::Vector3f xRelease,
                                          float vtoss,
                                          float tolerancDistToContact,
                                          float dt,
                                          float speedScaling);

  Eigen::MatrixXf getBimanualGraspMx(const Eigen::Matrix4f& wHo, Eigen::Matrix4f wHgp[]);
  Vector6f computeDesiredTaskTwist(const Eigen::Matrix4f& wHc, const Eigen::Matrix4f& wHd);

  Vector6f getDesObjectMotion();

  void setDt(float dt);
  void setReachableP(float reachableP);
  void setWHEEStandby(Eigen::Matrix4f wHEEStandby, int robotID);
  void setDesVelReach(float desVelReach);
  void setRefVelReach(float refVelReach, int robotID);
  void setModulatedReaching(bool modulatedReaching);
  void setIsNormImpactVel(bool isNormImpactVel);
  void setHeightViaPoint(float heightViaPoint);
  void setObjectDim(Eigen::Vector3f objectDim);
  void setActivationAperture(float activationAperture);

  float getActivationProximity();
  float getActivationNormal();
  float getActivationTangent();
  float getActivationRetract();
  float getActivationRelease();
};
