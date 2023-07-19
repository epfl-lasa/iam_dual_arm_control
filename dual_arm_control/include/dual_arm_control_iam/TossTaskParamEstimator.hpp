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
#include <iostream>
#include <sstream>
#include <vector>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/SVD"

#include "dual_arm_control_iam/tools/FirstOrderFilter.hpp"
#include "dual_arm_control_iam/tools/PdfGMR.hpp"

typedef Eigen::Matrix<float, 6, 1> Vector6f;
typedef Eigen::Matrix<float, 7, 1> Vector7f;

class TossTaskParamEstimator {
private:
  Eigen::Vector3f rel3DPosLandingRelease_;// 3d Relative position between landing and release position

  float phiThrowHorizontal_;  // angle of horizontal direction (XY) of the throwing plane
  float thetaElevationRelPos_;// elevation angle of of relative position atan(z,r)

  Eigen::Vector3f xRelease_;
  Eigen::Vector4f qRelease_;
  Eigen::Vector3f vRelease_;
  Eigen::Vector3f wRelease_;

  float g = 9.81f;

  Eigen::VectorXf priorGMMToss_;
  Eigen::MatrixXf meanGMMToss_;
  Eigen::MatrixXf covMxGMMToss_;

  PdfGMR myGMR_;
  FirstOrderFilter myRK4_;

public:
  /** 
   * ProjectileType
   * PHYS_IDEAL 			: physical model of ideal (point mass) projectile with no aerodynamic drag
   * PHYS_WITH_DRAG 	: physical model of projectile (ball) with Newton (aerodynamic) drag
   * LEARNED 					: learned model of projectile (ball) with Newton (aerodynamic) drag
  */
  enum ProjectileType { PHYS_IDEAL = 0, PHYS_WITH_DRAG = 1, LEARNED = 2 };

  TossTaskParamEstimator();
  ~TossTaskParamEstimator();

  void init(std::string fileGMM[],
            Eigen::Vector3f xRelease,
            Eigen::Vector4f qRelease,
            Eigen::Vector3f vRelease,
            Eigen::Vector3f wRelease);

  // Conversion of cartesian to planar coordinates
  void cartesianToPlanar(Eigen::Vector3f pos3DIn,
                         Eigen::Vector2f& posBar,
                         float& phiThrowHorizontal,
                         float& thetaElevationRelPos);

  // Compute min release velocity for the ideal projectile (point mass) no aerodynamic drag
  void getMinReleaseSpeed(Eigen::Vector2f posD, float& vReleaseI, float& tFlightI);
  void estimate2dThrowingParam(ProjectileType type,
                               Eigen::Vector2f posD,
                               float& angReleaseI,
                               float& vReleaseI,
                               float& tFlightI);

  bool estimateTossingParam(ProjectileType type, Eigen::Vector3f posLanding, Eigen::Vector3f posRelease);

  Eigen::Vector3f cartesian2spherical(Eigen::Vector3f posD);
  Eigen::Vector3f cartesianToPlanar(Eigen::Vector3f posD);
  Eigen::Vector4f dsProjectile2D(Eigen::Vector4f rz, float g, float mu);
  void projectileMotion2d(float T,
                          float g,
                          float mu,
                          Eigen::Vector2f posI,
                          Eigen::Vector2f posD,
                          float v0I,
                          float thetaI,
                          float& flytime,
                          Eigen::Vector2f& xLand2D);//
  Eigen::Vector2f
  estimateTargetSimplePathLengthAverageSpeed(Eigen::Vector3f x, Eigen::Vector3f xD, Eigen::Vector3f aVtarget);
  Eigen::Vector3f estimateTargetStateToGo(Eigen::Vector3f dxTarget,
                                          Eigen::Vector3f xIntercept,
                                          Eigen::Vector2f lpVaPredBot,
                                          Eigen::Vector2f lpVaPredTgt,
                                          float flytimeObj);

  Eigen::Vector3f getReleasePosition();
  Eigen::Vector4f getReleaseOrientation();
  Eigen::Vector3f getReleaseLinearVelocity();
  Eigen::Vector3f getReleaseAngularVelocity();
};
