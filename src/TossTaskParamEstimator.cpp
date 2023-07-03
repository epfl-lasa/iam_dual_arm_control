
#include "iam_dual_arm_control/TossTaskParamEstimator.h"

//
TossTaskParamEstimator::TossTaskParamEstimator() {}

TossTaskParamEstimator::~TossTaskParamEstimator() {}

void TossTaskParamEstimator::init(std::string fileGMM[],
                                  Eigen::Vector3f xRelease,
                                  Eigen::Vector4f qRelease,
                                  Eigen::Vector3f vRelease,
                                  Eigen::Vector3f wRelease) {

  xRelease_ = xRelease;
  qRelease_ = qRelease;
  vRelease_ = vRelease;
  wRelease_ = wRelease;

  myGMR_.init(fileGMM);
}

void TossTaskParamEstimator::cartesianToPlanar(Eigen::Vector3f pos3DIn,
                                               Eigen::Vector2f& posBar,
                                               float& phiThrowHorizontal,
                                               float& thetaElevationRelPos) {
  //Coordinates transformations
  //Convert cartesian into planar data using cylindrical coordinates
  posBar(0) = pos3DIn.head(2).norm();
  posBar(1) = pos3DIn(2);

  phiThrowHorizontal = std::atan2(pos3DIn(1), pos3DIn(0));

  // Direction of desired positions
  thetaElevationRelPos = std::atan2(posBar(1), posBar(0));
}

Eigen::Vector3f TossTaskParamEstimator::cartesianToPlanar(Eigen::Vector3f posD) {

  Eigen::Vector3f out = Eigen::VectorXf::Zero(3);
  out(0) = posD.head(2).norm();         // r
  out(1) = posD(2);                     // z
  out(2) = std::atan2(posD(1), posD(0));// phi
  return out;
}

Eigen::Vector3f TossTaskParamEstimator::cartesian2spherical(Eigen::Vector3f posD) {

  Eigen::Vector3f out = Eigen::VectorXf::Zero(3);
  out(0) = posD.norm();                             // r
  out(1) = std::atan2(posD.head(2).norm(), posD(2));// theta
  out(2) = std::atan2(posD(1), posD(0));            // phi

  return out;
}

void TossTaskParamEstimator::getMinReleaseSpeed(Eigen::Vector2f posD, float& vReleaseI, float& tFlightI) {

  float thetaElevationRelPos = std::atan2(posD(1), posD(0));

  // Function to estimate the minimum release velocity based of an known release angle
  float thetaI = std::atan(posD(1) / posD(0) + std::sqrt((posD(1) / posD(0)) * (posD(1) / posD(0)) + 1.f));

  if (thetaI > M_PI / 2.f) {
    thetaI = (90.f - 5.f) / 180.f * M_PI;
  } else if (thetaI < thetaElevationRelPos) {
    thetaI = thetaElevationRelPos + 5.f / 180.f * M_PI;
  }

  float deltaR = posD(0);
  float deltaZ = posD(1);

  float num = (this->g * deltaR * deltaR * (1.f + std::tan(thetaI) * std::tan(thetaI)));
  float den = (2.f * (deltaR * std::tan(thetaI) - deltaZ));

  // Get the initial velocity in (m/s)
  vReleaseI = fabs(std::sqrt(num / den));

  tFlightI =
      fabs((1. / this->g)
           * (vReleaseI * std::sin(thetaI) + std::sqrt(std::sin(thetaI) * std::sin(thetaI) + 2.f * this->g * deltaZ)));
}

void TossTaskParamEstimator::estimate2dThrowingParam(ProjectileType type,
                                                     Eigen::Vector2f posD,
                                                     float& angReleaseI,
                                                     float& vReleaseI,
                                                     float& tFlightI) {

  if (type == PHYS_IDEAL) {
    this->getMinReleaseSpeed(posD, vReleaseI, tFlightI);
  } else if (type == PHYS_WITH_DRAG) {
    this->getMinReleaseSpeed(posD, vReleaseI,
                             tFlightI);// TO DO replace with appropriate function TODO?
  } else {                             //(LEARNED)

    float vRelease;
    this->getMinReleaseSpeed(posD, vRelease, tFlightI);// TO DO replace with appropriate function, TODO?

    Eigen::MatrixXf eCovInput;
    Eigen::MatrixXf eCovOutput;
    Eigen::VectorXf thetaVelRelease;

    myGMR_.computeGMROutputs(posD, thetaVelRelease, eCovInput, eCovOutput);

    angReleaseI = thetaVelRelease(0);
    vReleaseI = thetaVelRelease(1);
  }
}

bool TossTaskParamEstimator::estimateTossingParam(ProjectileType type,
                                                  Eigen::Vector3f posLanding,
                                                  Eigen::Vector3f posRelease) {

  Eigen::Vector3f pos3DIn = posLanding - posRelease;

  Eigen::Vector2f posBar;
  float angReleaseI;
  float vReleaseI;
  float tFlightI;
  float thetaElevationRelPos;
  float phiThrowHorizontal;

  // in: pos3DIn --> out : posBar, phiThrowHorizontal, thetaElevationRelPos
  this->cartesianToPlanar(pos3DIn, posBar, phiThrowHorizontal, thetaElevationRelPos);

  // --> out: angReleaseI, vReleaseI, tFlightI
  this->estimate2dThrowingParam(type, posBar, angReleaseI, vReleaseI, tFlightI);

  this->xRelease_ = posRelease;
  this->vRelease_(0) = vReleaseI * cos(angReleaseI) * cos(phiThrowHorizontal);
  this->vRelease_(1) = vReleaseI * cos(angReleaseI) * sin(phiThrowHorizontal);
  this->vRelease_(2) = vReleaseI * sin(angReleaseI);
}

Eigen::Vector4f TossTaskParamEstimator::dsProjectile2D(Eigen::Vector4f rz, float g, float mu) {

  // Dynamics of a projectile with newton air drag
  Eigen::Vector4f dxdt = Eigen::VectorXf::Zero(4);

  float vRZ = rz.tail(2).norm();
  dxdt << rz(2), rz(3), -mu * rz(2) * vRZ, -mu * rz(3) * vRZ - g;

  return dxdt;
}

void TossTaskParamEstimator::projectileMotion2d(float T,
                                                float g,
                                                float mu,
                                                Eigen::Vector2f posI,
                                                Eigen::Vector2f posD,
                                                float v0I,
                                                float thetaI,
                                                float& flytime,
                                                Eigen::Vector2f& xLand2D) {
  float dt = T;// Intergration time
  float tol = 1e-5;
  Eigen::Vector4f rz0 = {posI(0), posI(1), v0I * cos(thetaI), v0I * sin(thetaI)};
  Eigen::Vector4f rz = rz0;
  Eigen::Vector2f rzRot = rz0.head(2);

  float thetaElevationRelPosD = std::atan2(posD(1), posD(0));
  Eigen::Matrix2f Rt;
  Rt << cos(thetaElevationRelPosD), sin(thetaElevationRelPosD), -sin(thetaElevationRelPosD), cos(thetaElevationRelPosD);

  // Projectile dynamics
  Eigen::Vector4f dxdt = dsProjectile2D(rz, g, mu);

  int iter = 0;
  bool isStopping = true;

  myRK4_.InitializeFilter(T, 1.f, 0.f, rz);
  flytime = 0.0f;

  while (isStopping) {
    if ((T > 0.001f) && (rz.tail(2).norm() > 5.f)) {
      dt = 0.001;
      myRK4_.setSampleTime(dt);
    } else {
      dt = T;
      myRK4_.setSampleTime(dt);
    }

    // Integrate
    rz = myRK4_.getRK4Integral(dxdt);
    rzRot = Rt * rz.head(2);

    dxdt = dsProjectile2D(rz, g, mu);
    flytime += dt;
    iter += 1;

    if (thetaElevationRelPosD < 0.f) {
      isStopping = ((rz(1) + tol >= posD(1)) && (rzRot(0) + tol != rz0(0)) && iter <= 2000);
    } else {
      isStopping = ((rzRot(1) + tol >= 0) && (rzRot(0) + tol != rz0(0)) && iter <= 2000);
    }
  }

  xLand2D = rz.head(2);
  flytime = flytime;
}

Eigen::Vector2f TossTaskParamEstimator::estimateTargetSimplePathLengthAverageSpeed(Eigen::Vector3f x,
                                                                                   Eigen::Vector3f xD,
                                                                                   Eigen::Vector3f aVtarget) {

  float vAvg = aVtarget.norm();
  float LpXdX = (xD - x).norm();

  // Path length and averge velocity
  Eigen::Vector2f lpDxAvg = {LpXdX, vAvg};
  return lpDxAvg;
}

Eigen::Vector3f TossTaskParamEstimator::estimateTargetStateToGo(Eigen::Vector3f dxTarget,
                                                                Eigen::Vector3f xIntercept,
                                                                Eigen::Vector2f lpVaPredBot,
                                                                Eigen::Vector2f lpVaPredTgt,
                                                                float flytimeObj) {
  float lptgt2XIntercept = 0.0f;

  if (lpVaPredBot(1) >= 1e-4) {
    lptgt2XIntercept = lpVaPredTgt(1) * (lpVaPredBot(0) / lpVaPredBot(1) + flytimeObj);
  } else {
    lptgt2XIntercept = 0.0f;
  }

  // Relative to xIntercept
  Eigen::Vector3f Xtarget2go = xIntercept - dxTarget.normalized() * lptgt2XIntercept;

  return Xtarget2go;
}

Eigen::Vector3f TossTaskParamEstimator::getReleasePosition() { return xRelease_; }
Eigen::Vector4f TossTaskParamEstimator::getReleaseOrientation() { return qRelease_; }
Eigen::Vector3f TossTaskParamEstimator::getReleaseLinearVelocity() { return vRelease_; }
Eigen::Vector3f TossTaskParamEstimator::getReleaseAngularVelocity() { return wRelease_; }
