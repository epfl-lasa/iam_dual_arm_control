#pragma once

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/SVD"

#include "iam_dual_arm_control/tools/FirstOrderFilter.hpp"
#include "iam_dual_arm_control/tools/PdfGMR.hpp"

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
  // PHYS_IDEAL 			: physical model of ideal (point mass) projectile with no aerodynamic drag
  // PHYS_WITH_DRAG 	: physical model of projectile (ball) with Newton (aerodynamic) drag
  // LEARNED 					: learned model of projectile (ball) with Newton (aerodynamic) drag
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
  Eigen::Vector2f estimate_robot_Lpath_avgVel(Eigen::Vector3f X, Eigen::Vector3f Xd);
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
