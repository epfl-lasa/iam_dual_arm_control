

#include "iam_dual_arm_control/ThrowingDS.h"

ThrowingDS::ThrowingDS() {
  for (int i = 0; i < 3; i++) {

    Kp_[i] = 5.0 * Eigen::MatrixXf::Identity(3, 3);
    Dp_[i] = 2.0 * sqrt(Kp_[i](0, 0)) * Eigen::MatrixXf::Identity(3, 3);

    Ko_[i] = 2.5 * Eigen::MatrixXf::Identity(3, 3);
    Do_[i] = 2.0 * sqrt(Ko_[i](0, 0)) * Eigen::MatrixXf::Identity(3, 3);
  }

  rho_ = 0.12;
  rangeNorm_ = 0.05;
  rangeTang_ = 0.02;

  swProxim_ = 300.0;
  swNorm_ = 300.0;
  swTang_ = 150.0;

  activationProximity_ = 0.0;
  activationNormal_ = 0.0;
  activationTangent_ = 0.0;
  activationRetract_ = 0.0;
  coupling_ = 0.0;

  wHde_ = Eigen::MatrixXf::Identity(4, 4);
  wHre_ = Eigen::MatrixXf::Identity(4, 4);
  wHpo_ = Eigen::MatrixXf::Identity(4, 4);
  vToss_ = Eigen::VectorXf::Zero(3);
  wToss_ = Eigen::VectorXf::Zero(3);
  basisQp_ = Eigen::MatrixXf::Identity(3, 3);
  basisQo_ = Eigen::MatrixXf::Identity(3, 3);

  Xt_ = Eigen::VectorXf::Zero(3);

  releaseFlag_ = false;
  refVtoss_ = 0.00f;
  activationToss_ = 0.0f;

  vMax_ = 1.15f;
  wMax_ = 4.0f;

  // ==============================================================================================================================
  dsParam_.is2ndOrder = false;
  // Modulation parameters
  dsParam_.modulRegion[0] = 0.35;
  dsParam_.modulRegion[1] = 0.04;
  dsParam_.modulRegion[2] = 0.02;

  // ============================================================================================
  // Gains
  // ============================================================================================
  float settlingTime = 1.0f;//[Settling time in second]
  float gains = pow((4.0f / settlingTime), 2.f);

  // Stiffness
  // ==========

  // Position
  dsParam_.Kp[0] << -gains, 0.00f, 0.00f, 0.00f, -gains, 0.00f, 0.00f, 0.00f, -gains;// Reach
  dsParam_.Kp[1] << -gains, 0.00f, 0.00f, 0.00f, -gains, 0.00f, 0.00f, 0.00f, -gains;// Toss
  dsParam_.Kp[2] << -gains, 0.00f, 0.00f, 0.00f, -gains, 0.00f, 0.00f, 0.00f, -gains;// retract

  // Orientation
  dsParam_.Ko[0] << -1.0f * gains, 0.00f, 0.00f, 0.00f, -1.0f * gains, 0.00f, 0.00f, 0.00f, -1.0f * gains;// Reach
  dsParam_.Ko[1] << -1.0f * gains, 0.00f, 0.00f, 0.00f, -1.0f * gains, 0.00f, 0.00f, 0.00f, -1.0f * gains;// Toss
  dsParam_.Ko[2] << -1.0f * gains, 0.00f, 0.00f, 0.00f, -1.0f * gains, 0.00f, 0.00f, 0.00f, -1.0f * gains;// retract

  if (dsParam_.is2ndOrder = false) {

    // Stiffness : choosen to yield the same settling time as the second order
    // =========

    // Position
    dsParam_.Kp[0] << -1.0f * (-1.0f * dsParam_.Kp[0]).cwiseSqrt();// Reach
    dsParam_.Kp[1] << -1.0f * (-1.0f * dsParam_.Kp[1]).cwiseSqrt();// Toss
    dsParam_.Kp[2] << -1.0f * (-1.0f * dsParam_.Kp[2]).cwiseSqrt();// retract

    // orientation
    dsParam_.Ko[0] << -1.0f * (-1.0f * dsParam_.Ko[0]).cwiseSqrt();// Reach
    dsParam_.Ko[1] << -1.0f * (-1.0f * dsParam_.Ko[1]).cwiseSqrt();// Toss
    dsParam_.Ko[2] << -1.0f * (-1.0f * dsParam_.Ko[2]).cwiseSqrt();// retract
  }

  // Damping
  // ==========
  // chosen to yield critically damped motion
  dsParam_.Dp[0] = -2.0f * (-1.0f * dsParam_.Kp[0]).cwiseSqrt();
  dsParam_.Dp[1] = -2.0f * (-1.0f * dsParam_.Kp[1]).cwiseSqrt();
  dsParam_.Dp[2] = -2.0f * (-1.0f * dsParam_.Kp[2]).cwiseSqrt();

  dsParam_.Do[0] = -2.0f * (-1.0f * dsParam_.Ko[0]).cwiseSqrt();
  dsParam_.Do[1] = -2.0f * (-1.0f * dsParam_.Ko[1]).cwiseSqrt();
  dsParam_.Do[2] = -2.0f * (-1.0f * dsParam_.Ko[2]).cwiseSqrt();

  // ==============================================================================================================================
}
ThrowingDS::~ThrowingDS() {}

bool ThrowingDS::init(float modulRegion[],
                      Eigen::Matrix3f Kp[],
                      Eigen::Matrix3f Dp[],
                      Eigen::Matrix3f Ko[],
                      Eigen::Matrix3f Do[],
                      bool is2ndOrder) {

  // Initialize the gains
  memcpy(Kp_, &Kp[0], 3 * sizeof *Kp);
  memcpy(Dp_, &Dp[0], 3 * sizeof *Dp);
  memcpy(Ko_, &Ko[0], 3 * sizeof *Ko);
  memcpy(Do_, &Do[0], 3 * sizeof *Do);

  is2ndOrder_ = is2ndOrder;
  rho_ = modulRegion[0];
  rangeNorm_ = modulRegion[1];
  rangeTang_ = modulRegion[2];

  return true;
}

bool ThrowingDS::init(tossDsParam ds_param,
                      Eigen::Vector3f releasePos,
                      Eigen::Vector4f releaseOrient,
                      Eigen::Vector3f releaseLinVel,
                      Eigen::Vector3f releaseAngVel,
                      Eigen::Vector3f restPos,
                      Eigen::Vector4f restOrient) {
  // Initialize the gains
  memcpy(Kp_, &ds_param.Kp[0], 3 * sizeof *ds_param.Kp);
  memcpy(Dp_, &ds_param.Dp[0], 3 * sizeof *ds_param.Dp);
  memcpy(Ko_, &ds_param.Ko[0], 3 * sizeof *ds_param.Ko);
  memcpy(Do_, &ds_param.Do[0], 3 * sizeof *ds_param.Do);

  is2ndOrder_ = ds_param.is2ndOrder;
  rho_ = ds_param.modulRegion[0];
  rangeNorm_ = ds_param.modulRegion[1];
  rangeTang_ = ds_param.modulRegion[2];

  vToss_ = releaseLinVel;
  wToss_ = releaseAngVel;

  wHde_ = Utils<float>::pose2HomoMx(releasePos, releaseOrient);
  wHre_ = Utils<float>::pose2HomoMx(restPos, restOrient);

  basisQp_ = this->createOrthonormalMatrixFromVector(vToss_);
  basisQo_ = this->createOrthonormalMatrixFromVector(wToss_);
  stopAndToss_ = false;

  Xt_ = releasePos;

  return true;
}

Vector6f ThrowingDS::apply(Eigen::Vector3f curPos,
                           Eigen::Vector4f curOrient,
                           Eigen::Vector3f curLinVel,
                           Eigen::Vector3f curAngVel) {

  Eigen::Matrix4f wHce = Utils<float>::pose2HomoMx(curPos, curOrient);
  Vector6f vEE = Eigen::VectorXf::Zero(6);
  vEE.head(3) = curLinVel;
  vEE.tail(3) = curAngVel;
  bool releaseFlag = false;

  Vector6f vDesObj = this->generateThrowingMotion(wHce, vEE, wHde_, wHre_, basisQp_, vToss_, releaseFlag);
  vDesObj = Utils<float>::SaturationTwist(vMax_, wMax_, vDesObj);
  float alp = 0.10f;
  refVtoss_ = (1.0f - alp) * refVtoss_ + alp * vToss_.norm();
  vDesObj.head(3) = vDesObj.head(3) / (vDesObj.head(3).norm() + 1e-10) * refVtoss_;
  return vDesObj;
}

Vector6f ThrowingDS::generateThrowingMotion(Eigen::Matrix4f wHce,
                                            Vector6f vEE,
                                            Eigen::Matrix4f wHde,
                                            Eigen::Matrix4f wHre,
                                            Eigen::Matrix3f basisQ,
                                            Eigen::Vector3f Vdtoss,
                                            bool& releaseFlag) {
  // States and desired states
  Eigen::Vector3f x = wHce.block<3, 1>(0, 3);
  Eigen::Vector3f xDot = vEE.head(3);
  Eigen::Vector3f omega = vEE.tail(3);
  Eigen::Vector3f xDes = wHde.block<3, 1>(0, 3);
  Eigen::Vector3f xRetr = wHre.block<3, 1>(0, 3);
  Eigen::Vector3f xB = xDes + basisQ * Eigen::Vector3f(-0.4 * this->rho_, 0.0, 0.0);// 0.4//0.5
  Eigen::Vector3f xE = xDes + basisQ * Eigen::Vector3f(0.2 * this->rho_, 0.0, 0.0); // 0.2
  Eigen::Vector3f xC = xDes + basisQ * Eigen::Vector3f(-0.1 * this->rho_, 0.0, 0.0);// 0.1 // wHpo_
  Eigen::Vector3f xPick = wHpo_.block<3, 1>(0, 3);
  Eigen::Matrix3f se1 = Eigen::MatrixXf::Zero(3, 3);
  se1(0, 0) = 1.0f;
  Eigen::Vector3f xTi = xDes + basisQ * se1 * basisQ.transpose() * (xPick - xDes);
  float den = basisQ.col(0).transpose() * Eigen::Vector3f(1.0, 0.0, 0.0);
  float num = (xPick - xTi).transpose() * Eigen::Vector3f(1.0, 0.0, 0.0);
  float d = num / (den + 1e-15);
  Eigen::Vector3f Xpe = xTi + basisQ.col(0) * d;

  float beta = 0.50f;
  Xt_ = xDes;
  if (stopAndToss_) {
    Xt_ = (1.0f - beta) * Xpe + beta * xB;
    Xt_(2) = 0.95f * xDes(2);
  } else {
    beta = 1.0f;
    Xt_ = (1.0f - beta) * Xpe + beta * xB;
  }

  //=======================================================================
  // Modulation term
  //=======================================================================
  Eigen::Vector3f Xqb = basisQ.transpose() * (x - xB);
  Eigen::Vector3f Xqe = basisQ.transpose() * (x - xE);

  float dist2reach = (x - xC).norm();
  float dist2line = Xqb.tail(2).norm();
  float dist2end = Xqe.head(1).norm();

  // scalar function of 3D distance  to initial (pre-modulation) position of attractor
  activationProximity_ = 0.5 * (std::tanh(this->swProxim_ * (0.5 * this->rho_ - dist2reach)) + 1.0);
  // scalar function of distance to the line of direction Vtoss and passing through the release position
  activationNormal_ = activationProximity_ * 0.5 * (std::tanh(this->swNorm_ * (this->rangeNorm_ - dist2line)) + 1.0);
  // scalar function of distance to the stopping position of the throwing task
  activationTangent_ = activationProximity_ * 0.5 * (std::tanh(this->swTang_ * (this->rangeTang_ - dist2end)) + 1.0);
  coupling_ = exp(-0.5 * dist2line / (2.0 * rangeNorm_ * rangeNorm_));
  coupling_ = 1.0;

  if (activationTangent_ >= 0.95f) { activationRetract_ = 1.0f; }

  // Release if the norm is within 1 cm
  if ((x - xDes).norm() <= 2.5e-2) { releaseFlag_ = true; }

  releaseFlag = releaseFlag_;

  float activation = activationProximity_;
  activationRetract_ = 0.0f;

  float tolRad = (x - Xt_).norm();
  float activationNormalT = 0.5f * (std::tanh(1.2f * this->swNorm_ * (this->rangeNorm_ - tolRad)) + 1.0f);

  if (activationNormalT >= 0.90f) { activationToss_ = 1.0f; }

  float swToss = (activationNormalT + activationToss_);
  if ((activationNormalT + activationToss_) >= 1.0f) { swToss = 1.0f; }

  // State-dependent gain matrix
  // ----------------------------
  Vector6f outMotion = Eigen::VectorXf::Zero(6, 1);

  if (this->is2ndOrder_) {
    Eigen::Vector3f Xstar = (1.0f - activationNormal_) * xB
        + activationNormal_ * (x + Vdtoss - (Eigen::MatrixXf::Identity(3, 3) - Kp_[TOSS].inverse() * Dp_[TOSS]) * xDot);

    // DS for approaching the tossing position
    Eigen::Vector3f activationReachEE = Dp_[REACH] * xDot + Kp_[REACH] * (x - xB);
    // Modulated DS that aligned  the EE with the desired velocity
    Eigen::Vector3f activationEENorm = Dp_[TOSS] * xDot + Kp_[TOSS] * (x - xB);
    Eigen::Vector3f activationEETang = Dp_[TOSS] * xDot + Kp_[TOSS] * (x - Xstar);
    // DS for retracting after the tossing position
    Eigen::Vector3f activationRetractEE = Dp_[RETRACT] * xDot + Kp_[RETRACT] * (x - xRetr);

    // Get the modulated motion (out_motion: Acceleration)
    outMotion.head(3) = (1.0 - activationRetract_)
            * this->computeModulatedMotion(activation, basisQ, activationReachEE, activationEENorm, activationEETang)
        + activationRetract_ * activationRetractEE;
    // Get angular motion
    outMotion.tail(3) = (1.0 - activationRetract_)
            * this->computeAngularMotion(coupling_, wHce, omega, wHde, Ko_[REACH], Do_[REACH], this->is2ndOrder_)
        + activationRetract_
            * this->computeAngularMotion(coupling_, wHce, omega, wHre, Ko_[RETRACT], Do_[RETRACT], this->is2ndOrder_);
  } else {
    Eigen::Vector3f Xstar = (1.0f - activationNormal_) * xB + activationNormal_ * (x - Kp_[TOSS].inverse() * Vdtoss);
    Eigen::Vector3f Xtoss = xDes + Vdtoss.normalized() * 0.05f;
    // DS for approaching the tossing position
    Eigen::Vector3f activationReachEE = Kp_[REACH] * (x - xB);
    // Modulated DS that aligned  the EE with the desired velocity
    Eigen::Vector3f activationEENorm = 1.5f * Kp_[TOSS] * (x - xB);
    Eigen::Vector3f activationEETang = Kp_[TOSS] * (x - Xstar);
    // DS for retracting after the tossing position
    Eigen::Vector3f activationRetractEE = Kp_[RETRACT] * (x - xRetr);

    // TODO printing needed?
    std::cout << "[ThrowingDS]:  -------------XXXXXXXXXXXXXXXXXXXXX ------ activationEETang   : \t"
              << activationEETang.transpose() << std::endl;

    // Get the modulated motion (out_motion: Velocity)
    outMotion.head(3) = (1.0 - activationRetract_)
            * this->computeModulatedMotion(activation, basisQ, activationReachEE, activationEENorm, activationEETang)
        + activationRetract_ * activationRetractEE;
    // Get angular motion
    outMotion.tail(3) = (1.0 - activationRetract_)
            * this->computeAngularMotion(coupling_, wHce, omega, wHde, Ko_[REACH], Do_[REACH], this->is2ndOrder_)
        + activationRetract_
            * this->computeAngularMotion(coupling_, wHce, omega, wHre, Ko_[RETRACT], Do_[RETRACT], this->is2ndOrder_);
  }

  return outMotion;
}

Eigen::Vector3f ThrowingDS::computeModulatedMotion(float activation,
                                                   Eigen::Matrix3f basisQ,
                                                   Eigen::Vector3f activationReachEE,
                                                   Eigen::Vector3f activationEENorm,
                                                   Eigen::Vector3f activationEETang) {
  Eigen::MatrixXf denTemp = activationEENorm.transpose() * activationEENorm;
  Eigen::RowVector3f betaJ = 1.0 / (denTemp(0, 0) + 1e-10) * (activationEENorm.transpose() * basisQ);

  Eigen::Matrix3f lambda = Eigen::MatrixXf::Zero(3, 3);
  lambda.block<1, 1>(0, 0) = activation * (basisQ.col(0).transpose() * activationEETang * betaJ(0))
      + (1.0 - activation) * Eigen::MatrixXf::Identity(1, 1);
  lambda.block<1, 1>(0, 1) = activation * (basisQ.col(0).transpose() * activationEETang * betaJ(1));
  lambda.block<1, 1>(0, 2) = activation * (basisQ.col(0).transpose() * activationEETang * betaJ(2));

  lambda.block<1, 1>(1, 0) = activation * (basisQ.col(1).transpose() * activationEENorm * betaJ(0));
  lambda.block<1, 1>(1, 1) = activation * (basisQ.col(1).transpose() * activationEENorm * betaJ(1))
      + (1.0 - activation) * Eigen::MatrixXf::Identity(1, 1);
  lambda.block<1, 1>(1, 2) = activation * (basisQ.col(1).transpose() * activationEENorm * betaJ(2));

  lambda.block<1, 1>(2, 0) = activation * (basisQ.col(2).transpose() * activationEENorm * betaJ(0));
  lambda.block<1, 1>(2, 1) = activation * (basisQ.col(2).transpose() * activationEENorm * betaJ(1));
  lambda.block<1, 1>(2, 2) = activation * (basisQ.col(2).transpose() * activationEENorm * betaJ(2))
      + (1.0 - activation) * Eigen::MatrixXf::Identity(1, 1);

  // Computing the modulated second order DS (translation)
  float comb = 0.3f;
  return (1.f - comb) * basisQ * lambda * basisQ.transpose() * activationEENorm + comb * activationReachEE;
}

Eigen::Vector3f ThrowingDS::computeAngularMotion(float coupling,
                                                 Eigen::Matrix4f wHc,
                                                 Eigen::Vector3f omega,
                                                 Eigen::Matrix4f wHd,
                                                 Eigen::Matrix3f Ko,
                                                 Eigen::Matrix3f Do,
                                                 bool is2ndOrder) {

  // Rotation motion
  //----------------
  // Current orientation of the end effector
  Eigen::Matrix3f wRc = wHc.block<3, 3>(0, 0);
  // Desired orientation of the end effector
  Eigen::Matrix3f wRd = wHd.block<3, 3>(0, 0);
  // Coupling orientation task to position through spherical interpolation of the orientation
  Eigen::Matrix3f wRdt = Utils<float>::getCombinedRotationMatrix(coupling, wRc, wRd);
  // Relative transformation between desired and current frame
  Eigen::Matrix3f dRct = wRdt.transpose() * wRc;
  // 3D Orientation Jacobian
  Eigen::Matrix3f jacMuTheta = Utils<float>::getMuThetaJacobian(dRct) * wRc.transpose();

  if (is2ndOrder) {
    // Approaximation of the acceleration (neglecting  -jacMuTheta_dot * omega)
    return jacMuTheta.inverse() * (Do * jacMuTheta * omega + Ko * Utils<float>::getOrientationErrorCur2Des(dRct));
  } else {
    return (Ko * Utils<float>::getOrientationErrorCur2Des(dRct));
  }
}

Eigen::MatrixXf ThrowingDS::createOrthonormalMatrixFromVector(Eigen::VectorXf inVec) {
  int n = inVec.rows();
  Eigen::MatrixXf basis = Eigen::MatrixXf::Random(n, n);

  if (inVec.norm() <= 1e-10) {
    basis.col(0) = Eigen::VectorXf::Zero(n);
    basis(0, 0) = 1.0f;
  } else {
    basis.col(0) = 1.f / inVec.norm() * inVec;
  }

  assert(basis.rows() == basis.cols());
  uint dim = basis.rows();
  basis.col(0).normalize();

  for (uint i = 1; i < dim; i++) {
    for (uint j = 0; j < i; j++) basis.col(i) -= basis.col(j).dot(basis.col(i)) * basis.col(j);
    basis.col(i).normalize();
  }

  if (basis.rows() == 3) {
    Eigen::Vector3f u = basis.col(0);
    Eigen::Vector3f v = basis.col(1);
    Eigen::Vector3f w = u.cross(v);
    basis.col(2) = w;
  }

  return basis;
}

bool ThrowingDS::setTossLinearVelocity(Eigen::Vector3f newLinVel) {
  vToss_ = newLinVel;
  basisQp_ = this->createOrthonormalMatrixFromVector(vToss_);
  return true;
}

bool ThrowingDS::setTossPose(Eigen::Vector3f newReleasePos, Eigen::Vector4f newReleaseOrient) {
  wHde_ = Utils<float>::pose2HomoMx(newReleasePos, newReleaseOrient);
  return true;
}

bool ThrowingDS::setPickupObjectPose(Eigen::Vector3f pickupPos, Eigen::Vector4f pickupOrient) {
  wHpo_ = Utils<float>::pose2HomoMx(pickupPos, pickupOrient);
  return true;
}

bool ThrowingDS::resetReleaseFlag() {
  releaseFlag_ = false;
  activationRetract_ = 0.0f;
  activationToss_ = 0.0f;
  return true;
}

tossDsParam ThrowingDS::getDsParam() { return dsParam_; }
float ThrowingDS::getActivationProximity() { return activationProximity_; }
float ThrowingDS::getActivationNormal() { return activationNormal_; }
float ThrowingDS::getActivationTangent() { return activationTangent_; };
float ThrowingDS::getActivationToss() { return activationToss_; };

float ThrowingDS::setRefVtoss(float newValue) { refVtoss_ = newValue; }
