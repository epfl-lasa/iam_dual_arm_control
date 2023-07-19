
#include "iam_dual_arm_control/DualArmFreeMotionController.hpp"

DualArmFreeMotionController::DualArmFreeMotionController() {
  errorAbs_.setZero();
  errorRel_.setZero();
  errorObj_.setZero();
  velAbs_.setZero();
  velRel_.setZero();
  velObj_.setZero();
  gainPosAbs_.setZero();
  gainOriAbs_.setZero();
  gainPosRel_.setZero();
  gainOriRel_.setZero();
  omegaObjectD_.setZero();
  vDesO_.setZero();

  tbi_ = Eigen::MatrixXf::Identity(6, 6);
  tbi_.topLeftCorner(3, 3) = 0.5f * Eigen::MatrixXf::Identity(3, 3);
  tbi_.topRightCorner(3, 3) = 0.5f * Eigen::MatrixXf::Identity(3, 3);
  tbi_.bottomLeftCorner(3, 3) = -Eigen::MatrixXf::Identity(3, 3);
  tbi_.bottomRightCorner(3, 3) = Eigen::MatrixXf::Identity(3, 3);

  activationProximity_ = 0.0f;
  activationProximity_ = 0.0f;
  activationNormal_ = 0.0f;
  activationTangent_ = 0.0f;
  activationRetract_ = 0.0f;
  activationRelease_ = 0.0f;
  releaseFlag_ = false;

  rho_ = 0.15;
  rangeNorm_ = 0.05;
  rangeTang_ = 0.015;

  swProxim_ = 100.0;
  swNorm_ = 150.0;
  swTang_ = 200.0;

  activationNormalDo_ = 0.0f;
  desVelReach_ = 1.0f;
  refVelReach_[LEFT] = 0.0f;
  refVelReach_[RIGHT] = 0.0f;
  modulatedReaching_ = true;
  isNormImpactVel_ = false;
  heightViaPoint_ = 0.25f;

  swEEObsAv_ = 100.0f;
  minDistEE_ = 0.09f;

  Eigen::Vector2f P1 = Eigen::Vector2f(0.f, 0.085f);
  Eigen::Vector2f P2 = Eigen::Vector2f(0.f, -0.085f);
  Eigen::Vector2f P3 = Eigen::Vector2f(0.04f, 0.f);

  Eigen::Matrix3f Mdelta, Mx0, My0;
  Mdelta << P1(0), P1(1), 1, P2(0), P2(1), 1, P3(0), P3(1), 1;
  Mx0 << pow(P1(0), 2) + pow(P1(1), 2), P1(1), 1, pow(P2(0), 2) + pow(P2(1), 2), P2(1), 1,
      pow(P3(0), 2) + pow(P3(1), 2), P3(1), 1;
  My0 << pow(P1(0), 2) + pow(P1(1), 2), P1(0), 1, pow(P2(0), 2) + pow(P2(1), 2), P2(0), 1,
      pow(P3(0), 2) + pow(P3(1), 2), P3(0), 1;

  float delta = 2 * Mdelta.determinant();
  Eigen::Vector2f centerSphereXZ = Eigen::Vector2f(1 / delta * Mx0.determinant(), -1 / delta * My0.determinant());
  safeRadius_ = centerSphereXZ.norm();
  integralVeeD_[LEFT].setZero();
  integralVeeD_[RIGHT].setZero();

  twistVo_.setZero();
  wHvo_.setIdentity();
  wHVgp_[0].setIdentity();
  wHVgp_[1].setIdentity();
  dt_ = 0.005;
  objectDim_ << 0.20f, 0.20f, 0.20f;
  gotToObject_ = 0.0f;
  alphaObs_[LEFT] = 0.0f;
  alphaObs_[RIGHT] = 0.0f;
  activationAperture_ = 1.0f;
}

DualArmFreeMotionController::~DualArmFreeMotionController() {}

// publishing of the reference trajectories
bool DualArmFreeMotionController::init(Eigen::Matrix4f wHEEStandby[], Matrix6f gainAbs, Matrix6f gainRel) {

  memcpy(wHEEStandby_, &wHEEStandby[0], NB_ROBOTS * sizeof *wHEEStandby);

  reachableP_ = 1.0f;

  vMax_ = 1.1f;
  wMax_ = 3.0f;

  gainPosAbs_ = gainAbs.topLeftCorner(3, 3);
  gainOriAbs_ = gainAbs.bottomRightCorner(3, 3);
  gainPosRel_ = gainRel.topLeftCorner(3, 3);
  gainOriRel_ = gainRel.bottomRightCorner(3, 3);

  qdPrev_[LEFT] << 1.0f, 0.0f, 0.0f, 0.0f;
  qdPrev_[RIGHT] << 1.0f, 0.0f, 0.0f, 0.0f;
}

void DualArmFreeMotionController::computeConstrainedMotion(Eigen::Matrix4f wHee[],
                                                           Eigen::Matrix4f wHgp[],
                                                           Eigen::Matrix4f wHo,
                                                           Vector6f vDesEE[],
                                                           Eigen::Vector4f qd[],
                                                           bool isOrient3d) {
  // Computation of desired orientation
  this->computeDesiredOrientation(1.0f, wHee, wHgp, wHo, qd, isOrient3d);
  Eigen::Matrix4f wHDgpL = wHgp[LEFT];
  wHDgpL.block<3, 3>(0, 0) = Utils<float>::quaternionToRotationMatrix(qd[LEFT]);
  Eigen::Matrix4f wHdpgR = wHgp[RIGHT];
  wHdpgR.block<3, 3>(0, 0) = Utils<float>::quaternionToRotationMatrix(qd[RIGHT]);

  // Get the task transformations
  Eigen::Matrix4f wHAr, lrHRR;      // absolute and relative EE poses
  Eigen::Matrix4f wHAp, lpHRp;      // absolute and relative object's grasp points
  Eigen::Matrix4f wHArStb, lrHRRStb;// absolute and relative EE standby poses
  Eigen::Matrix4f lpHRpPgrasp;      // relative pregrasp EE pose

  // Bimanual coordinated task-space transforms
  Utils<float>::getBimanualTransforms(wHee[LEFT], wHee[RIGHT], wHAr, lrHRR);// EE
  Utils<float>::getBimanualTransforms(wHDgpL, wHdpgR, wHAp, lpHRp);         // object's grasp points
  Utils<float>::getBimanualTransforms(this->wHEEStandby_[LEFT],
                                      this->wHEEStandby_[RIGHT],
                                      wHArStb,
                                      lrHRRStb);// standby arms

  lpHRpPgrasp = lpHRp;
  lpHRpPgrasp(1, 3) *= 1.4;

  // =======================================
  // Absolute velocity of the End-effectors
  // =======================================
  Eigen::Vector3f dPAbs = reachableP_ * wHAp.block<3, 1>(0, 3) + (1.0f - reachableP_) * wHArStb.block<3, 1>(0, 3);
  errorAbs_.head(3) = wHAr.block<3, 1>(0, 3) - dPAbs;

  // Coupling the orientation with the position error
  float cplAbs = 1.0f;

  Eigen::Matrix3f dRAbs = reachableP_ * wHAp.block<3, 3>(0, 0) + (1.0f - reachableP_) * wHArStb.block<3, 3>(0, 0);
  Eigen::Matrix4f wHArT = wHAr;
  wHArT.block<3, 3>(0, 0) = Utils<float>::getCombinedRotationMatrix(cplAbs, wHAr.block<3, 3>(0, 0), dRAbs);//desired

  // relative transformation
  Eigen::Matrix4f dHCAbs = wHArT.inverse() * wHAr;

  // orientation error
  errorAbs_.tail(3) = Utils<float>::getPoseErrorCur2Des(dHCAbs).tail(3);

  // 3D Orientation Jacobian
  Eigen::Matrix3f jacMuThetaAbs =
      Utils<float>::getMuThetaJacobian(dHCAbs.block<3, 3>(0, 0)) * wHAr.block<3, 3>(0, 0).transpose();

  // computing the velocity
  // ~~~~~~~~~~~~~~~~~~~~~~~
  velAbs_.head(3) = -4.0f * gainPosAbs_ * errorAbs_.head(3);
  velAbs_.tail(3) = -1.0f * jacMuThetaAbs.inverse() * gainOriAbs_ * errorAbs_.tail(3);

  // =====================================
  // Relative velocity of the hands
  // =====================================
  float coordAbs = 1.0f;

  // Coupling the orientation with the position error
  Eigen::Matrix3f dRRel =
      reachableP_ * (coordAbs * lpHRp.block<3, 3>(0, 0) + (1.0f - coordAbs) * lpHRpPgrasp.block<3, 3>(0, 0))
      + (1.0f - reachableP_) * lrHRRStb.block<3, 3>(0, 0);

  Eigen::Matrix4f lrHRRT = lrHRR;
  lrHRRT.block<3, 3>(0, 0) = Utils<float>::getCombinedRotationMatrix(coordAbs, lrHRR.block<3, 3>(0, 0), dRRel);//desired

  // relative transformation
  lrHRRT.block<3, 3>(0, 0) = Utils<float>::quaternionToRotationMatrix(qd[LEFT]).transpose()
      * Utils<float>::quaternionToRotationMatrix(qd[RIGHT]);
  Eigen::Matrix4f dHCRel = lrHRRT.inverse() * lrHRR;// expressed in the left hand frame

  // orientation error
  errorRel_.tail(3) = Utils<float>::getPoseErrorCur2Des(dHCRel).tail(3);

  // 3D Orientation Jacobian
  Eigen::Matrix3f jacMuThetaRel = Utils<float>::getMuThetaJacobian(dHCRel.block<3, 3>(0, 0))
      * wHee[RIGHT].block<3, 3>(0, 0).transpose();// wrt. the world

  float cplRel = 1.0f;

  // position error accounting for the reachability of the target
  Eigen::Vector3f dPRel =
      reachableP_ * (cplRel * lpHRp.block<3, 1>(0, 3) + (1.0f - cplRel) * lpHRpPgrasp.block<3, 1>(0, 3))
      + (1.0f - reachableP_) * lrHRRStb.block<3, 1>(0, 3);// TBC
  errorRel_.head(3) = lrHRR.block<3, 1>(0, 3) - dPRel;

  // computing the velocity
  velRel_.head(3) = -4.0f * gainPosRel_ * errorRel_.head(3);
  velRel_.tail(3) = -4.0f * jacMuThetaRel.inverse() * gainOriRel_ * errorRel_.tail(3);

  // ========================================
  // Computation of individual EE motion
  // ========================================
  // velocity
  float aBi = 0.5f;
  float bBi = 1.0f;
  Utils<float>::getBimanualTwistDistribution(aBi, bBi, velAbs_, velRel_, vDesEE[LEFT], vDesEE[RIGHT]);
}

void DualArmFreeMotionController::computeAsyncMotion(Eigen::Matrix4f wHee[],
                                                     Eigen::Matrix4f wHgp[],
                                                     Eigen::Matrix4f wHo,
                                                     Vector6f vDesEE[],
                                                     Eigen::Vector4f qd[],
                                                     bool isOrient3d) {

  for (int k = 0; k < NB_ROBOTS; k++) {
    Vector6f errorEE;
    errorEE.setZero();
    Eigen::Vector3f dPEE =
        reachableP_ * wHgp[k].block<3, 1>(0, 3) + (1.0f - reachableP_) * this->wHEEStandby_[k].block<3, 1>(0, 3);
    Eigen::Matrix3f dREE =
        reachableP_ * wHgp[k].block<3, 3>(0, 0) + (1.0f - reachableP_) * this->wHEEStandby_[k].block<3, 3>(0, 0);
    Eigen::Matrix4f wHEeT = wHee[k];
    wHEeT.block<3, 3>(0, 0) = Utils<float>::getCombinedRotationMatrix(1.0f, wHee[k].block<3, 3>(0, 0), dREE);//desired

    // relative transformation between desired and current frame
    Eigen::Matrix4f dHCEe = wHEeT.inverse() * wHee[k];
    errorEE.head(3) = wHee[k].block<3, 1>(0, 3) - dPEE;
    errorEE.tail(3) = Utils<float>::getPoseErrorCur2Des(dHCEe).tail(3);

    // 3D Orientation Jacobian
    Eigen::Matrix3f jacMuThetaEE =
        Utils<float>::getMuThetaJacobian(dHCEe.block<3, 3>(0, 0)) * wHee[k].block<3, 3>(0, 0).transpose();

    // ---------------------------------
    // computing of desired ee velocity
    // ---------------------------------
    vDesEE[k].head(3) = -2.0f * gainPosAbs_ * errorEE.head(3);
    vDesEE[k].tail(3) = -3.0f * jacMuThetaEE.inverse() * gainOriAbs_ * errorEE.tail(3);
    vDesEE[k] = Utils<float>::saturationTwist(vMax_, wMax_, vDesEE[k]);
  }

  // Computation of desired orientation
  this->computeDesiredOrientation(0.5f, wHee, wHgp, wHo, qd, isOrient3d);
}

void DualArmFreeMotionController::computeDesiredOrientation(float weight,
                                                            Eigen::Matrix4f wHee[],
                                                            Eigen::Matrix4f wHgp[],
                                                            Eigen::Matrix4f wHo,
                                                            Eigen::Vector4f qd[],
                                                            bool isOrient3d) {

  if (isOrient3d) {
    for (int k = 0; k < NB_ROBOTS; k++) {
      qd[k] = Utils<float>::getSlerpInterpolation(weight, wHee[k].block(0, 0, 3, 3), wHgp[k].block(0, 0, 3, 3));
    }
  } else {
    for (int k = 0; k < NB_ROBOTS; k++) {
      Eigen::Vector3f ref;
      ref = wHgp[k].block<3, 1>(0, 2);
      ref.normalize();

      // Compute rotation error between current orientation and plane orientation using Rodrigues' law
      Eigen::Vector3f u;

      u = (wHee[k].block<3, 3>(0, 0).col(2)).cross(ref);
      float s = u.norm();
      u /= s;
      float c = (wHee[k].block<3, 3>(0, 0).col(2)).transpose() * ref;

      Eigen::Matrix3f K;
      K << Utils<float>::getSkewSymmetricMatrix(u);

      Eigen::Matrix3f Re;
      if (fabs(s) < FLT_EPSILON) {
        Re = Eigen::Matrix3f::Identity();
      } else {
        Re = Eigen::Matrix3f::Identity() + s * K + (1 - c) * K * K;
      }

      // Convert rotation error into axis angle representation
      Eigen::Vector3f omega;
      float angle;
      Eigen::Vector4f qtemp = Utils<float>::rotationMatrixToQuaternion(Re);
      Eigen::Vector4f q = Utils<float>::rotationMatrixToQuaternion(wHee[k].block<3, 3>(0, 0));
      Utils<float>::quaternionToAxisAngle(qtemp, omega, angle);

      // Compute final quaternion on plane
      Eigen::Vector4f qf = Utils<float>::quaternionProduct(qtemp, q);
      // Eigen::Vector4f qf = Utils<float>::quaternionProduct(q,qtemp);

      // Perform quaternion slerp interpolation to progressively orient the end effector while approaching the object surface
      qd[k] = Utils<float>::slerpQuaternion(q, qf, 1.0f - std::tanh(2.0f * errorRel_.head(3).norm()));

      if (qd[k].dot(qdPrev_[k]) < 0.0f) { qd[k] *= -1.0f; }

      qdPrev_[k] = qd[k];
    }
  }
}

void DualArmFreeMotionController::computeReleaseAndRetractMotion(Eigen::Matrix4f wHee[],
                                                                 Eigen::Matrix4f wHgp[],
                                                                 Eigen::Matrix4f wHo,
                                                                 Vector6f vDesEE[],
                                                                 Eigen::Vector4f qd[],
                                                                 bool isOrient3d) {
  // Computation of desired orientation
  this->computeDesiredOrientation(1.0f, wHee, wHEEStandby_, wHo, qd, isOrient3d);
  Eigen::Matrix4f wHDgpL = wHgp[LEFT];
  wHDgpL.block<3, 3>(0, 0) = Utils<float>::quaternionToRotationMatrix(qd[LEFT]);
  Eigen::Matrix4f wHdpgR = wHgp[RIGHT];
  wHdpgR.block<3, 3>(0, 0) = Utils<float>::quaternionToRotationMatrix(qd[RIGHT]);

  // get the task transformations
  Eigen::Matrix4f wHAr, lrHRR;      // absolute and relative EE poses
  Eigen::Matrix4f wHAp, lpHRp;      // absolute and relative object's grasp points
  Eigen::Matrix4f wHArStb, lrHRRStb;// absolute and relative EE standby poses

  // Bimanual coordinated task-space transforms
  Utils<float>::getBimanualTransforms(wHee[LEFT], wHee[RIGHT], wHAr, lrHRR);// EE

  Utils<float>::getBimanualTransforms(wHDgpL, wHdpgR, wHAp, lpHRp);// object's grasp points
  Utils<float>::getBimanualTransforms(this->wHEEStandby_[LEFT],
                                      this->wHEEStandby_[RIGHT],
                                      wHArStb,
                                      lrHRRStb);// standby arms

  // =====================================
  // Relative velocity of the hands
  // =====================================
  // Coupling the orientation with the position error
  Eigen::Matrix3f dRRel = lrHRRStb.block<3, 3>(0, 0);

  Eigen::Matrix4f lrHRRT = lrHRR;
  lrHRRT.block<3, 3>(0, 0) = Utils<float>::getCombinedRotationMatrix(1.0f, lrHRR.block<3, 3>(0, 0), dRRel);//desired

  // relative transformation
  lrHRRT.block<3, 3>(0, 0) = Utils<float>::quaternionToRotationMatrix(qd[LEFT]).transpose()
      * Utils<float>::quaternionToRotationMatrix(qd[RIGHT]);
  Eigen::Matrix4f dHCRel = lrHRRT.inverse() * lrHRR;// expressed in the left hand frame

  // orientation error
  errorRel_.tail(3) = Utils<float>::getPoseErrorCur2Des(dHCRel).tail(3);
  // 3D Orientation Jacobian
  Eigen::Matrix3f jacMuThetaRel = Utils<float>::getMuThetaJacobian(dHCRel.block<3, 3>(0, 0))
      * wHee[RIGHT].block<3, 3>(0, 0).transpose();// wrt. the world

  // position error accounting for the reachability of the target
  Eigen::Vector3f dPRel = lrHRRStb.block<3, 1>(0, 3);// TBC
  errorRel_.head(3) = lrHRR.block<3, 1>(0, 3) - dPRel;

  // computing the velocity
  velRel_.head(3) = -4.0f * gainPosRel_ * errorRel_.head(3);
  velRel_.tail(3) = -jacMuThetaRel.inverse() * gainOriRel_ * errorRel_.tail(3);

  // =======================================
  // Absolute velocity of the End-effectors
  // =======================================
  float cplRel = Utils<float>::computeCouplingFactor(errorRel_.head(3), 50.0f, 0.01f, 1.0f, true);// 50.0f, 0.05f, 2.8f

  Eigen::Vector3f dPAbs = (1.f - cplRel) * wHAr.block<3, 1>(0, 3) + cplRel * wHArStb.block<3, 1>(0, 3);
  errorAbs_.head(3) = wHAr.block<3, 1>(0, 3) - dPAbs;

  // Coupling the orientation with the position error
  float cplAbs = Utils<float>::computeCouplingFactor(errorAbs_.head(3), 50.0f, 0.02f, 1.0f, false);// 0.2f
  cplAbs = 1.0f;

  Eigen::Matrix3f dRAbs = (1.f - cplAbs) * wHAr.block<3, 3>(0, 0) + cplAbs * wHArStb.block<3, 3>(0, 0);
  Eigen::Matrix4f wHArT = wHAr;
  wHArT.block<3, 3>(0, 0) = Utils<float>::getCombinedRotationMatrix(cplAbs, wHAr.block<3, 3>(0, 0), dRAbs);//desired
  // relative transformation
  Eigen::Matrix4f dHCAbs = wHArT.inverse() * wHAr;
  // orientation error
  errorAbs_.tail(3) = Utils<float>::getPoseErrorCur2Des(dHCAbs).tail(3);
  // 3D Orientation Jacobian
  Eigen::Matrix3f jacMuThetaAbs =
      Utils<float>::getMuThetaJacobian(dHCAbs.block<3, 3>(0, 0)) * wHAr.block<3, 3>(0, 0).transpose();

  // computing the velocity
  // ~~~~~~~~~~~~~~~~~~~~~~~
  velAbs_.head(3) = -gainPosAbs_ * errorAbs_.head(3);
  velAbs_.tail(3) = -jacMuThetaAbs.inverse() * gainOriAbs_ * errorAbs_.tail(3);

  // ========================================
  // Computation of individual EE motion
  // ========================================
  // velocity
  float aBi = 0.5f;
  float bBi = 1.0f;
  Utils<float>::getBimanualTwistDistribution(aBi, bBi, velAbs_, velRel_, vDesEE[LEFT], vDesEE[RIGHT]);

  for (int k = 0; k < NB_ROBOTS; k++) {
    Eigen::Matrix3f dREE = Utils<float>::quaternionToRotationMatrix(qd[k]);
    Eigen::Matrix4f wHEeT = wHee[k];
    wHEeT.block<3, 3>(0, 0) = dREE;//desired
    Eigen::Matrix4f dHCEe = wHEeT.inverse() * wHee[k];
    Eigen::Vector3f errorEEo = Utils<float>::getPoseErrorCur2Des(dHCEe).tail(3);
    Eigen::Matrix3f jacMuThetaEE =
        Utils<float>::getMuThetaJacobian(dHCEe.block<3, 3>(0, 0)) * wHee[k].block<3, 3>(0, 0).transpose();
    vDesEE[k].tail(3) = -4.0f * jacMuThetaEE.inverse() * gainOriAbs_ * errorEEo;
  }

  vDesEE[LEFT] = Utils<float>::saturationTwist(vMax_, wMax_, vDesEE[LEFT]);
  vDesEE[RIGHT] = Utils<float>::saturationTwist(vMax_, wMax_, vDesEE[RIGHT]);
}

void DualArmFreeMotionController::generatePlacingMotion(Eigen::Matrix4f wHee[],
                                                        Eigen::Matrix4f wHgp[],
                                                        Eigen::Matrix4f wHo,
                                                        Eigen::Matrix4f wHDo,
                                                        float viaHeight,
                                                        Vector6f vDesEE[],
                                                        Eigen::Vector4f qd[],
                                                        bool isOrient3d) {
  // Computation of desired orientation
  this->computeDesiredOrientation(1.0f, wHee, wHgp, wHo, qd, isOrient3d);
  Eigen::Matrix4f wHDgpL = wHgp[LEFT];
  wHDgpL.block<3, 3>(0, 0) = Utils<float>::quaternionToRotationMatrix(qd[LEFT]);
  Eigen::Matrix4f wHdpgR = wHgp[RIGHT];
  wHdpgR.block<3, 3>(0, 0) = Utils<float>::quaternionToRotationMatrix(qd[RIGHT]);

  Eigen::Matrix4f wHOZ, wHDoZ;// current and desired object pose but with height of via plane
  Eigen::Matrix4f wHAp, lpHRp;// absolute and relative object's grasp points
  Eigen::Matrix4f wHAr, lrHRR;// absolute and relative EE poses

  wHOZ = wHo;
  wHDoZ = wHDo;
  wHOZ(2, 3) = wHDo(2, 3) + viaHeight;
  wHDoZ(2, 3) = wHDo(2, 3) + viaHeight;

  // Bimanual transformation
  Utils<float>::getBimanualTransforms(wHDgpL, wHdpgR, wHAp, lpHRp);         // object's grasp points
  Utils<float>::getBimanualTransforms(wHee[LEFT], wHee[RIGHT], wHAr, lrHRR);// EE

  Eigen::Vector3f errorZ = Eigen::Vector3f(0.f, 0.f, wHo(2, 3) - wHOZ(2, 3));
  Eigen::Vector3f errorXY = Eigen::Vector3f(wHo(0, 3) - wHDoZ(0, 3), wHo(1, 3) - wHDoZ(1, 3), 0.0f);

  float cplOZ = 1.0f - std::tanh(8.0f * errorZ.norm());
  float cplDOXY = Utils<float>::computeCouplingFactor(errorXY, 50.0f, 0.10f, 1.0f, false);

  // ================================================================
  // Desired Object motion : Absolute velocity of the End-effectors
  // ================================================================
  float satCplZ = ((cplOZ + cplDOXY) <= 1.f) ? (cplOZ + cplDOXY) : 1.f;
  float coordPos =
      Utils<float>::computeCouplingFactor(errorXY,
                                          50.0f,
                                          0.02f,
                                          1.0f,
                                          true);//  Coupling the orientation function of planar position error
  Eigen::Matrix4f wHOT = wHo;
  wHOT.block<3, 3>(0, 0) =
      Utils<float>::getCombinedRotationMatrix(coordPos, wHo.block<3, 3>(0, 0), wHDo.block<3, 3>(0, 0));//desired

  // Relative pose of the current object pose relative to the desired one
  Eigen::Matrix4f dHCObj = wHOT.inverse() * wHo;// relative transformation
  Eigen::Vector3f dPosObj = satCplZ * (cplDOXY * wHDo.block<3, 1>(0, 3) + (1.f - cplDOXY) * wHDoZ.block<3, 1>(0, 3))
      + (1.f - satCplZ) * wHOZ.block<3, 1>(0, 3);
  Eigen::Matrix3f jacMuThetaObj = Utils<float>::getMuThetaJacobian(dHCObj.block<3, 3>(0, 0))
      * wHo.block<3, 3>(0, 0).transpose();// 3D Orientation Jacobian

  // pose error
  errorObj_.head(3) = wHo.block<3, 1>(0, 3) - dPosObj;                  // position error
  errorObj_.tail(3) = Utils<float>::getPoseErrorCur2Des(dHCObj).tail(3);// orientation error

  // computing the velocity
  // ~~~~~~~~~~~~~~~~~~~~~~~
  velObj_.head(3) = -gainPosAbs_ * errorObj_.head(3);
  velObj_.tail(3) = -jacMuThetaObj.inverse() * gainOriAbs_ * errorObj_.tail(3);

  // ================================================================
  // Compute relative hand velocity to maintain the grasp
  // ================================================================
  // relative velocity
  Eigen::Matrix3f dRRel = lpHRp.block<3, 3>(0, 0);
  Eigen::Matrix4f lrHRRT = lrHRR;

  // Computation of desired orientation
  lrHRRT.block<3, 3>(0, 0) = Utils<float>::getCombinedRotationMatrix(1.0f, lrHRR.block<3, 3>(0, 0), dRRel);//desired

  Eigen::Matrix4f dHCRel = lrHRRT.inverse() * lrHRR;// expressed in the left hand frame
  Eigen::Matrix3f jacMuThetaRel = Utils<float>::getMuThetaJacobian(dHCRel.block<3, 3>(0, 0))
      * wHee[RIGHT].block<3, 3>(0, 0).transpose();// 3D Orientation Jacobian wrt. the world
  Eigen::Vector3f dPosRel = lpHRp.block<3, 1>(0, 3);

  errorRel_.head(3) = lrHRR.block<3, 1>(0, 3) - dPosRel;
  errorRel_.tail(3) = Utils<float>::getPoseErrorCur2Des(dHCRel).tail(3);// orientation error

  // computing the velocity
  velRel_.head(3) = -4.0f * gainPosRel_ * errorRel_.head(3);
  velRel_.tail(3) = -4.0f * jacMuThetaRel.inverse() * gainOriRel_ * errorRel_.tail(3);

  // ================================================================
  // compute the grasp matrix to distribute the motion
  // ================================================================
  Eigen::Matrix<float, 6, 12> GraspMxObjEE;
  GraspMxObjEE.block<6, 6>(0, 0).setIdentity();
  GraspMxObjEE.block<6, 6>(0, 6).setIdentity();

  Eigen::Matrix3f skewMx[NB_ROBOTS];
  for (int i = 0; i < NB_ROBOTS; i++) {
    Eigen::Vector3f t = wHee[i].block<3, 1>(0, 3) - wHo.block<3, 1>(0, 3);
    skewMx[i] << 0.0f, -t(2), t(1), t(2), 0.0f, -t(0), -t(1), t(0), 0.0f;
  }
  GraspMxObjEE.block<3, 3>(3, 0) = -skewMx[LEFT]; // left EE
  GraspMxObjEE.block<3, 3>(3, 6) = -skewMx[RIGHT];// right EE

  // ===================================================================
  // Computation of individual EE motion
  // ===================================================================
  vDesEE[LEFT] = GraspMxObjEE.block<6, 6>(0, 0).transpose() * velObj_ - 0.5f * velRel_;
  vDesEE[RIGHT] = GraspMxObjEE.block<6, 6>(0, 6).transpose() * velObj_ + 0.5f * velRel_;

  // applying velocity
  // ========================================
  vDesEE[LEFT] = Utils<float>::saturationTwist(vMax_, wMax_, vDesEE[LEFT]);
  vDesEE[RIGHT] = Utils<float>::saturationTwist(vMax_, wMax_, vDesEE[RIGHT]);

  std::cout << "[dual_arm_control]: CCCCCCCCCCC cplOZ: \t" << cplOZ << std::endl;
  std::cout << "[dual_arm_control]: CCCCCCCCCCC cplDOXY: \t" << cplDOXY << std::endl;
}

void DualArmFreeMotionController::computeCoordinatedMotion2(Eigen::Matrix4f wHee[],
                                                            Eigen::Matrix4f wHgp[],
                                                            Eigen::Matrix4f wHo,
                                                            Vector6f vDesEE[],
                                                            Eigen::Vector4f qd[],
                                                            bool isOrient3d) {

  float coordAbs = Utils<float>::computeCouplingFactor(errorAbs_.head(3), 50.0f, 0.02f, 1.0f, true);

  // Computation of desired orientation
  this->computeDesiredOrientation(coordAbs, wHee, wHgp, wHo, qd, isOrient3d);
  Eigen::Matrix4f wHDgpL = wHgp[LEFT];
  wHDgpL.block<3, 3>(0, 0) = Utils<float>::quaternionToRotationMatrix(qd[LEFT]);
  Eigen::Matrix4f wHdpgR = wHgp[RIGHT];
  wHdpgR.block<3, 3>(0, 0) = Utils<float>::quaternionToRotationMatrix(qd[RIGHT]);

  // get the task transformations
  Eigen::Matrix4f wHAr, lrHRR;                // absolute and relative EE poses
  Eigen::Matrix4f wHAp, lpHRp;                // absolute and relative object's grasp points
  Eigen::Matrix4f wHArStb, lrHRRStb;          // absolute and relative EE standby poses
  Eigen::Matrix4f lpHRpPgrasp;                // relative pregrasp EE pose
  Eigen::Matrix4f wHApMomentum, lpHRpMomentum;// absolute and relative rectracting EE pose to give some momentum

  // Bimanual coordinated task-space transforms
  Utils<float>::getBimanualTransforms(wHee[LEFT], wHee[RIGHT], wHAr, lrHRR);// EE
  Utils<float>::getBimanualTransforms(wHDgpL, wHdpgR, wHAp, lpHRp);         // object's grasp points
  Utils<float>::getBimanualTransforms(this->wHEEStandby_[LEFT],
                                      this->wHEEStandby_[RIGHT],
                                      wHArStb,
                                      lrHRRStb);// standby arms

  lpHRpPgrasp = lpHRp;
  lpHRpPgrasp(1, 3) = lpHRp(1, 3) / fabs(lpHRp(1, 3)) * (fabs(lpHRp(1, 3)) + 0.30f);

  float combCoef = 0.3f;
  lpHRpMomentum = lpHRp;
  lpHRpMomentum.block(0, 3, 3, 1) =
      combCoef * lpHRpPgrasp.block(0, 3, 3, 1) + (1. - combCoef) * lrHRRStb.block(0, 3, 3, 1);

  wHApMomentum = wHAp;
  wHApMomentum.block(0, 3, 3, 1) = combCoef * wHAp.block(0, 3, 3, 1) + (1. - combCoef) * wHArStb.block(0, 3, 3, 1);

  // =======================================
  // Absolute velocity of the End-effectors
  // =======================================
  Eigen::Vector3f dPAbs = activationAperture_ * reachableP_ * wHAp.block<3, 1>(0, 3)
      + (1.0f - activationAperture_ * reachableP_)
          * (activationAperture_ * wHArStb.block<3, 1>(0, 3)
             + (1. - activationAperture_) * wHApMomentum.block<3, 1>(0, 3));
  errorAbs_.head(3) = wHAr.block<3, 1>(0, 3) - dPAbs;

  // Coupling the orientation with the position error
  float cplAbs = Utils<float>::computeCouplingFactor(errorAbs_.head(3), 50.0f, 0.02f, 1.0f, false);

  Eigen::Matrix3f dRAbs = reachableP_ * wHAp.block<3, 3>(0, 0) + (1.0f - reachableP_) * wHArStb.block<3, 3>(0, 0);
  Eigen::Matrix4f wHArT = wHAr;
  wHArT.block<3, 3>(0, 0) = Utils<float>::getCombinedRotationMatrix(cplAbs, wHAr.block<3, 3>(0, 0), dRAbs);//desired

  // relative transformation
  Eigen::Matrix4f dHCAbs = wHArT.inverse() * wHAr;

  // orientation error
  errorAbs_.tail(3) = Utils<float>::getPoseErrorCur2Des(dHCAbs).tail(3);

  // 3D Orientation Jacobian
  Eigen::Matrix3f jacMuThetaAbs =
      Utils<float>::getMuThetaJacobian(dHCAbs.block<3, 3>(0, 0)) * wHAr.block<3, 3>(0, 0).transpose();

  // computing the velocity
  // ~~~~~~~~~~~~~~~~~~~~~~~
  velAbs_.head(3) = -3.0f * gainPosAbs_ * errorAbs_.head(3);// -3.0
  velAbs_.tail(3) = -3.0f * jacMuThetaAbs.inverse() * gainOriAbs_ * errorAbs_.tail(3);

  // =====================================
  // Relative velocity of the hands
  // =====================================
  // Coupling the orientation with the position error
  Eigen::Matrix3f dRRel =
      reachableP_ * (coordAbs * lpHRp.block<3, 3>(0, 0) + (1.0f - coordAbs) * lpHRpPgrasp.block<3, 3>(0, 0))
      + (1.0f - reachableP_) * lrHRRStb.block<3, 3>(0, 0);

  Eigen::Matrix4f lrHRRT = lrHRR;
  lrHRRT.block<3, 3>(0, 0) = Utils<float>::getCombinedRotationMatrix(coordAbs, lrHRR.block<3, 3>(0, 0), dRRel);//desired

  // relative transformation
  lrHRRT.block<3, 3>(0, 0) = Utils<float>::quaternionToRotationMatrix(qd[LEFT]).transpose()
      * Utils<float>::quaternionToRotationMatrix(qd[RIGHT]);
  Eigen::Matrix4f dHCRel = lrHRRT.inverse() * lrHRR;// expressed in the left hand frame

  // orientation error
  errorRel_.tail(3) = Utils<float>::getPoseErrorCur2Des(dHCRel).tail(3);
  // 3D Orientation Jacobian
  Eigen::Matrix3f jacMuThetaRel = Utils<float>::getMuThetaJacobian(dHCRel.block<3, 3>(0, 0))
      * wHee[RIGHT].block<3, 3>(0, 0).transpose();// wrt. the world

  float cplRel =
      Utils<float>::computeCouplingFactor(errorAbs_.head(3), 50.0f, 0.08f, 1.0f, true);// 50.0f, 0.05f, 2.8f  0.5

  Eigen::Vector3f oErrorPosAbs = wHo.block<3, 3>(0, 0).transpose() * errorAbs_.head(3);
  Eigen::Vector3f oErrorPosAbsParal = Eigen::Vector3f(oErrorPosAbs(0), 0.0f, oErrorPosAbs(2));
  float cpAp = activationAperture_
      * Utils<float>::computeCouplingFactor(oErrorPosAbsParal,
                                            50.0f,
                                            0.06f,
                                            1.0f,
                                            true);// 50.0f, 0.12f, 1.0f  (0.04f) (0.02f)

  // position error accounting for the reachability of the target
  Eigen::Vector3f dPRel = cplRel
          * (cpAp * lpHRp.block<3, 1>(0, 3)
             + (1.0f - cpAp)
                 * (activationAperture_ * lpHRpPgrasp.block<3, 1>(0, 3)
                    + (1. - activationAperture_) * lpHRpMomentum.block<3, 1>(0, 3)))
      + (1.0f - cplRel) * lrHRRStb.block<3, 1>(0, 3);// TBC

  errorRel_.head(3) = lrHRR.block<3, 1>(0, 3) - dPRel;//

  // computing the velocity
  velRel_.head(3) = -5.0f * gainPosRel_ * errorRel_.head(3);                          // -3.0
  velRel_.tail(3) = -3.0f * jacMuThetaRel.inverse() * gainOriRel_ * errorRel_.tail(3);// -3.0

  // ========================================
  // Computation of individual EE motion
  // ========================================
  // velocity
  float aBi = 0.5f;
  float bBi = 1.0f;
  Utils<float>::getBimanualTwistDistribution(aBi, bBi, velAbs_, velRel_, vDesEE[LEFT], vDesEE[RIGHT]);

  Eigen::Matrix4f dHCL = wHDgpL.inverse() * wHee[LEFT];
  Eigen::Matrix4f dHCR = wHdpgR.inverse() * wHee[RIGHT];

  // orientation error
  Eigen::Vector3f errorOriLeft = Utils<float>::getPoseErrorCur2Des(dHCL).tail(3);
  Eigen::Vector3f errorOriRight = Utils<float>::getPoseErrorCur2Des(dHCR).tail(3);

  // 3D Orientation Jacobian
  Eigen::Matrix3f jacMuThetaLeft = Utils<float>::getMuThetaJacobian(dHCL.block<3, 3>(0, 0))
      * wHee[LEFT].block<3, 3>(0, 0).transpose();// wrt. the world
  Eigen::Matrix3f jacMuThetaRight = Utils<float>::getMuThetaJacobian(dHCR.block<3, 3>(0, 0))
      * wHee[RIGHT].block<3, 3>(0, 0).transpose();// wrt. the world

  vDesEE[LEFT].tail(3) = -3.0f * jacMuThetaLeft.inverse() * gainOriRel_ * errorOriLeft;
  vDesEE[RIGHT].tail(3) = -3.0f * jacMuThetaRight.inverse() * gainOriRel_ * errorOriRight;

  vDesEE[LEFT] = Utils<float>::saturationTwist(vMax_, wMax_, vDesEE[LEFT]);
  vDesEE[RIGHT] = Utils<float>::saturationTwist(vMax_, wMax_, vDesEE[RIGHT]);
}

Vector6f DualArmFreeMotionController::generatePlacingMotion2(Eigen::Matrix4f wHo,
                                                             Eigen::Matrix4f wHDo,
                                                             float viaHeight,
                                                             Vector6f Vo,
                                                             bool isPlaceTossing) {

  Eigen::Matrix4f wHOZ, wHDoZ;// current and desired object pose but with height of via plane
  Eigen::Matrix4f wHAp, lpHRp;// absolute and relative object's grasp points
  Eigen::Matrix4f wHAr, lrHRR;// absolute and relative EE poses

  wHOZ = wHo;
  if (isPlaceTossing) {
    wHOZ(2, 3) = wHDo(2, 3) + 0.0 * viaHeight;
  } else {
    wHOZ(2, 3) = wHDo(2, 3) + viaHeight;
  }
  wHDoZ = wHDo;
  wHDoZ(2, 3) = wHDo(2, 3) + viaHeight;

  Eigen::Vector3f errorZ = Eigen::Vector3f(0.f, 0.f, wHo(2, 3) - wHOZ(2, 3));
  Eigen::Vector3f errorXY = Eigen::Vector3f(wHo(0, 3) - wHDoZ(0, 3), wHo(1, 3) - wHDoZ(1, 3), 0.0f);
  float cplOZ = 0.5f * (std::tanh(1.5f * this->swNorm_ * (1.2f * this->rangeNorm_ - errorZ.norm())) + 1.0f);
  float cplDOXY = 0.5f * (std::tanh(1.5f * this->swNorm_ * (2.0f * this->rangeNorm_ - errorXY.norm())) + 1.0f);

  // ================================================================
  // Desired Object motion : Absolute velocity of the End-effectors
  // ================================================================
  float satCplZ = ((cplOZ + cplDOXY) <= 1.f) ? (cplOZ + cplDOXY) : 1.f;
  float coordPos =
      Utils<float>::computeCouplingFactor(errorXY,
                                          50.0f,
                                          0.02f,
                                          1.0f,
                                          true);//  Coupling the orientation function of planar position error
  Eigen::Matrix4f wHOT = wHo;
  wHOT.block<3, 3>(0, 0) =
      Utils<float>::getCombinedRotationMatrix(coordPos, wHo.block<3, 3>(0, 0), wHDo.block<3, 3>(0, 0));//desired
  // Relative pose of the current object pose relative to the desired one
  Eigen::Matrix4f dHCObj = wHOT.inverse() * wHo;// relative transformation
  Eigen::Vector3f dPosObj = satCplZ * (cplDOXY * wHDo.block<3, 1>(0, 3) + (1.f - cplDOXY) * wHDoZ.block<3, 1>(0, 3))
      + (1.f - satCplZ) * wHOZ.block<3, 1>(0, 3);
  Eigen::Matrix3f jacMuThetaObj = Utils<float>::getMuThetaJacobian(dHCObj.block<3, 3>(0, 0))
      * wHo.block<3, 3>(0, 0).transpose();// 3D Orientation Jacobian

  // pose error
  errorObj_.head(3) = wHo.block<3, 1>(0, 3) - dPosObj;                  // position error
  errorObj_.tail(3) = Utils<float>::getPoseErrorCur2Des(dHCObj).tail(3);// orientation error

  // computing the velocity
  // ~~~~~~~~~~~~~~~~~~~~~~~
  velObj_.head(3) = -3.0 * gainPosAbs_ * errorObj_.head(3);
  velObj_.tail(3) = -3.0 * jacMuThetaObj.inverse() * gainOriAbs_ * errorObj_.tail(3);

  return velObj_;
}

Eigen::Vector3f DualArmFreeMotionController::computeModulatedMotion(float activation,
                                                                    Eigen::Matrix3f basisQ,
                                                                    Eigen::Vector3f activationReachEE,
                                                                    Eigen::Vector3f activationEENorm,
                                                                    Eigen::Vector3f activationEETang) {
  Eigen::MatrixXf denTemp = activationReachEE.transpose() * activationReachEE;
  Eigen::RowVector3f betaJ = 1.0f / (denTemp(0, 0) + 1e-10) * (activationReachEE.transpose() * basisQ);

  Eigen::Matrix3f Lambda = Eigen::MatrixXf::Zero(3, 3);
  Lambda.block<1, 1>(0, 0) = activation * (basisQ.col(0).transpose() * activationEETang * betaJ(0))
      + (1.0 - activation) * Eigen::MatrixXf::Identity(1, 1);
  Lambda.block<1, 1>(0, 1) = activation * (basisQ.col(0).transpose() * activationEETang * betaJ(1));
  Lambda.block<1, 1>(0, 2) = activation * (basisQ.col(0).transpose() * activationEETang * betaJ(2));

  Lambda.block<1, 1>(1, 0) = activation * (basisQ.col(1).transpose() * activationEENorm * betaJ(0));
  Lambda.block<1, 1>(1, 1) = activation * (basisQ.col(1).transpose() * activationEENorm * betaJ(1))
      + (1.0 - activation) * Eigen::MatrixXf::Identity(1, 1);
  Lambda.block<1, 1>(1, 2) = activation * (basisQ.col(1).transpose() * activationEENorm * betaJ(2));

  Lambda.block<1, 1>(2, 0) = activation * (basisQ.col(2).transpose() * activationEENorm * betaJ(0));
  Lambda.block<1, 1>(2, 1) = activation * (basisQ.col(2).transpose() * activationEENorm * betaJ(1));
  Lambda.block<1, 1>(2, 2) = activation * (basisQ.col(2).transpose() * activationEENorm * betaJ(2))
      + (1.0 - activation) * Eigen::MatrixXf::Identity(1, 1);

  // computing the modulated second order DS (translation)
  return basisQ * Lambda * basisQ.transpose() * activationReachEE;
}

Vector6f DualArmFreeMotionController::computeModulatedMotionDual(float activation,
                                                                 Eigen::Matrix3f basisQ[],
                                                                 Vector6f dsEENominal,
                                                                 Vector6f activationEENorm,
                                                                 Vector6f activationEETang) {
  // computing the modulated second order DS (translation)
  Vector6f vdModulated = Eigen::VectorXf::Zero(6);
  vdModulated.head(3) = this->computeModulatedMotion(activation,
                                                     basisQ[LEFT],
                                                     dsEENominal.head(3),
                                                     activationEENorm.head(3),
                                                     activationEETang.head(3));
  vdModulated.tail(3) = this->computeModulatedMotion(activation,
                                                     basisQ[RIGHT],
                                                     dsEENominal.tail(3),
                                                     activationEENorm.tail(3),
                                                     activationEETang.tail(3));

  return vdModulated;
}

void DualArmFreeMotionController::dualArmMotion(Eigen::Matrix4f wHee[],
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
                                                bool& releaseFlag) {
  // States and desired states
  Eigen::Vector3f X[NB_ROBOTS],// position of ee
      Xdot[NB_ROBOTS],         // linear velocity of ee
      Omega[NB_ROBOTS],        // angular velocity of ee
      Xdes[NB_ROBOTS],         // desired position (final attractor)
      Xb[NB_ROBOTS],           // first transitor attractor
      Xe[NB_ROBOTS],           // final attractor
      Xc[NB_ROBOTS];           // center of modulation ellipsoid

  Eigen::Vector3f Xqb[NB_ROBOTS], Xqe[NB_ROBOTS];
  float dist2reach[NB_ROBOTS], dist2line[NB_ROBOTS], dist2end[NB_ROBOTS];

  for (int i = 0; i < NB_ROBOTS; i++) {
    X[i] = wHee[i].block<3, 1>(0, 3);
    Xdot[i] = Vee[i].head(3);
    Omega[i] = Vee[i].tail(3);
    Xdes[i] = wHgp[i].block<3, 1>(0, 3);
    Xb[i] = Xdes[i] + basisQ[i] * Eigen::Vector3f(-0.5f * this->rho_, 0.0f, 0.0f);
    Xe[i] = Xdes[i] + basisQ[i] * Eigen::Vector3f(0.2f * this->rho_, 0.0f, 0.0f);
    Xc[i] = Xdes[i] + basisQ[i] * Eigen::Vector3f(-0.2f * this->rho_, 0.0f, 0.0f);

    //=======================================================================
    // Modulation term
    //=======================================================================
    Xqb[i] = basisQ[i].transpose() * (X[i] - Xb[i]);
    Xqe[i] = basisQ[i].transpose() * (X[i] - Xe[i]);

    dist2reach[i] = (X[i] - Xc[i]).norm();
    dist2line[i] = Xqb[i].tail(2).norm();
    dist2end[i] = Xqe[i].head(1).norm();
  }

  // Modulation term

  // scalar function of 3D distance  to initial (pre-modulation) position of attractor
  activationProximity_ = 0.5f
      * (0.5f * (std::tanh(this->swProxim_ * (0.5f * this->rho_ - dist2reach[LEFT])) + 1.0f)
         + 0.5f * (std::tanh(this->swProxim_ * (0.5f * this->rho_ - dist2reach[RIGHT])) + 1.0f));

  // scalar function of distance to the line of direction VdImp  and passing through the release position
  activationNormal_ = activationProximity_ * 0.5f
      * (0.5f * (std::tanh(this->swNorm_ * (this->rangeNorm_ - dist2line[LEFT])) + 1.0f)
         + 0.5f * (std::tanh(this->swNorm_ * (this->rangeNorm_ - dist2line[RIGHT])) + 1.0f));

  // scalar function of distance to the atopping position of the throwing task
  activationTangent_ = activationProximity_ * 0.5f
      * (0.5f * (std::tanh(this->swTang_ * (this->rangeTang_ - dist2end[LEFT])) + 1.0f)
         + 0.5f * (std::tanh(this->swTang_ * (this->rangeTang_ - dist2end[RIGHT])) + 1.0f));

  if (activationTangent_ >= 0.95f) { activationRetract_ = 1.0f; }

  // release if the norm is within 1 cm
  if ((X[LEFT] - Xdes[LEFT]).norm() <= 1e-2 && (X[RIGHT] - Xdes[RIGHT]).norm() <= 1e-2) { releaseFlag_ = true; }
  releaseFlag = releaseFlag_;// reaching phase

  float activation = activationProximity_;

  // state-dependent gain matrix
  // ----------------------------
  Eigen::Matrix4f wHGpT[NB_ROBOTS];
  wHGpT[LEFT] = wHgp[LEFT];
  wHGpT[RIGHT] = wHgp[RIGHT];
  wHGpT[LEFT].block<3, 1>(0, 3) = Xb[LEFT];
  wHGpT[RIGHT].block<3, 1>(0, 3) = Xb[RIGHT];

  Vector6f vdEENorm[NB_ROBOTS];
  Eigen::Vector4f qdNorm[NB_ROBOTS];

  if (modulatedReaching_ || isNormImpactVel_) {
    this->computeCoordinatedMotion2(wHee, wHGpT, wHo, vdEENorm, qdNorm, isOrient3d);
  } else {
    this->computeCoordinatedMotion2(wHee, wHgp, wHo, vdEENorm, qdNorm, isOrient3d);
  }

  Vector6f dsEENominal = Eigen::VectorXf::Zero(6);
  dsEENominal.head(3) = vdEENorm[LEFT].head(3);
  dsEENominal.tail(3) = vdEENorm[RIGHT].head(3);

  Matrix6f a = Eigen::MatrixXf::Identity(6, 6);
  a.block<3, 3>(0, 0) = -4.0f * this->gainPosAbs_;
  a.block<3, 3>(3, 3) = -4.0f * this->gainPosRel_;
  Matrix6f aPrime = tbi_.inverse() * a * tbi_;
  Matrix6f tbiInversA = tbi_.inverse() * a;

  Vector6f xDual, xDesDual, xbDual, vdImpDual;
  xDual.head(3) = X[LEFT];
  xDual.tail(3) = X[RIGHT];
  xDesDual.head(3) = Xdes[LEFT];
  xDesDual.tail(3) = Xdes[RIGHT];
  xbDual.head(3) = Xb[LEFT];
  xbDual.tail(3) = Xb[RIGHT];
  vdImpDual.head(3) = VdImp[LEFT];
  vdImpDual.tail(3) = VdImp[RIGHT];

  Eigen::Matrix3f qToss = Eigen::MatrixXf::Identity(3, 3);
  if (vdO.head(3).norm() <= 1e-6) {
    qToss = Utils<float>::create3dOrthonormalMatrixFromVector(Eigen::Vector3f(1.0f, 0.0f, 0.0f));//
  } else {
    qToss = Utils<float>::create3dOrthonormalMatrixFromVector(vdO.head(3));//
  }

  Eigen::Vector3f Xqo = qToss.transpose() * (wHo.block<3, 1>(0, 3) - wHDo.block<3, 1>(0, 3));
  float dist2line_toss = Xqo.tail(2).norm();
  activationNormalDo_ = 0.5f * (std::tanh(1.0f * this->swNorm_ * (0.99f * this->rangeNorm_ - dist2line_toss)) + 1.0f);

  if (activationNormalDo_ >= 0.90f) { activationRelease_ = 1.0f; }

  float swNormDo = (activationNormalDo_ + activationRelease_);
  if ((activationNormalDo_ + activationRelease_) >= 1.0f) { swNormDo = 1.0f; }
  swNormDo = 1.0f;

  Vector6f xStarDual = xDual;
  Vector6f activationEENorm = Eigen::VectorXf::Zero(6);// Modulated DS that aligned the EE with the desired velocity
  Vector6f activationEETang = Eigen::VectorXf::Zero(6);

  Eigen::MatrixXf graspMxObj = this->getBimanualGraspMx(wHo, wHgp);

  switch (taskType) {
    // Reaching with impact
    case 0: {
      // TODO: add
      xStarDual = (1.0 - activationNormal_) * xbDual + activationNormal_ * (xDual - aPrime.inverse() * vdImpDual);
      if (VdImp[LEFT].norm() <= 0.01f || VdImp[RIGHT].norm() <= 0.01f) {
        // TODO: add
        xStarDual = (1.0 - activationNormal_) * xbDual + activationNormal_ * xDesDual;
      }

      // Modulated DS that aligned the EE with the desired velocity
      activationEENorm = aPrime * (xDual - xbDual);
      activationEETang = aPrime * (xDual - xStarDual);

      if (!(modulatedReaching_ || isNormImpactVel_)) { activation = 0.0f; }
      vDesO_.setZero();
    } break;
    // Point to point motion of the object
    case 1: {
      a.block<3, 3>(0, 0) = -4.0f * this->gainPosAbs_;
      a.block<3, 3>(3, 3) = -12.0f * this->gainPosRel_;

      Vector6f xBi = Eigen::VectorXf::Zero(6);
      xBi.head(3) = 0.5f * (X[LEFT] + X[RIGHT]);
      xBi.tail(3) = 0.95f * (X[RIGHT] - X[LEFT]);
      xStarDual = tbi_.inverse() * xBi;

      vDesO_ = this->computeDesiredTaskTwist(wHo, wHDo);
      Vector6f desTwistObj = vDesO_;
      desTwistObj.tail(3) = 1.0f * desTwistObj.tail(3);

      Vector6f desTwistLeft = graspMxObj.leftCols(6).transpose() * desTwistObj;
      Vector6f desTwistRight = graspMxObj.rightCols(6).transpose() * desTwistObj;
      Vector6f xDotBi = Eigen::VectorXf::Zero(6);
      xDotBi.head(3) = 0.5 * (desTwistLeft.head(3) + desTwistRight.head(3));
      xDotBi.tail(3) = Eigen::VectorXf::Zero(3);

      Vector6f vTaskBi = (a * tbi_ * (xDual - xStarDual) + 1.0f * swNormDo * xDotBi);

      // Apply velocity limits
      vTaskBi = Utils<float>::saturationTwist(vdO.head(3).norm(), wMax_, vTaskBi);
      activationEENorm = tbi_.inverse() * vTaskBi;
      activationEETang = tbi_.inverse() * vTaskBi;
      activation = 1.0f;

      vdEENorm[LEFT].tail(3) = 1.0 * vdEENorm[LEFT].tail(3) + 1.0f * vDesO_.tail(3);
      vdEENorm[RIGHT].tail(3) = 1.0 * vdEENorm[RIGHT].tail(3) + 1.0f * vDesO_.tail(3);
    } break;
      // Velocity based motion of the object
    case 2: {
      integralVeeD_[LEFT].setZero();
      integralVeeD_[RIGHT].setZero();

      Vector6f xBi = Eigen::VectorXf::Zero(6);
      xBi.head(3) = 0.5f * (X[LEFT] + X[RIGHT]);
      xBi.tail(3) = 0.95f * (X[RIGHT] - X[LEFT]);
      xStarDual = tbi_.inverse() * xBi;

      //Velocity based motion of the object
      Vector6f xDotBi = Eigen::VectorXf::Zero(6);
      Eigen::Vector3f xRel = X[RIGHT] - X[LEFT];
      Eigen::Vector3f wO = 0.0f * vdO.tail(3);

      xDotBi.head(3) = vdO.head(3);
      xDotBi.tail(3) = wO.cross(xRel);

      Vector6f vTaskBi = (a * tbi_ * (xDual - xStarDual) + 1.0f * swNormDo * xDotBi);
      vTaskBi.head(3) = vTaskBi.head(3).normalized() * vdO.head(3).norm();

      activationEENorm = tbi_.inverse() * vTaskBi;
      activationEETang = tbi_.inverse() * vTaskBi;
      activation = 1.0f;
    } break;
      // Point to point motion of the object
    case 3: {
      a.block<3, 3>(0, 0) = -4.0f * this->gainPosAbs_;
      a.block<3, 3>(3, 3) = -20.0f * this->gainPosRel_;

      integralVeeD_[LEFT].setZero();
      integralVeeD_[RIGHT].setZero();

      Vector6f xBi = Eigen::VectorXf::Zero(6);
      xBi.head(3) = 0.5f * (X[LEFT] + X[RIGHT]);
      xBi.tail(3) = 0.95f * (X[RIGHT] - X[LEFT]);
      xStarDual = tbi_.inverse() * xBi;

      float aFilt = 0.02;
      vDesO_.head(3) = (1 - aFilt) * vDesO_.head(3) + aFilt * vdO.head(3);
      vDesO_.head(3) = vdO.head(3);

      Vector6f desTwistLeft = graspMxObj.leftCols(6).transpose() * vDesO_;
      Vector6f desTwistRight = graspMxObj.rightCols(6).transpose() * vDesO_;

      // Velocity based motion of the object
      Vector6f xDotBi = Eigen::VectorXf::Zero(6);
      xDotBi.head(3) = 0.5 * (desTwistLeft.head(3) + desTwistRight.head(3));
      xDotBi.tail(3) = 0.0 * (desTwistRight.head(3) - desTwistLeft.head(3));

      Vector6f vTaskBi = (a * tbi_ * (xDual - xStarDual) + 1.0f * swNormDo * xDotBi);
      vTaskBi.head(3) = vTaskBi.head(3).normalized() * 1.0f * vTaskBi.head(3).norm();

      activationEENorm = tbi_.inverse() * vTaskBi;
      activationEETang = tbi_.inverse() * vTaskBi;
      activation = 1.0f;

      vdEENorm[LEFT].tail(3) = vdEENorm[LEFT].tail(3) + 1.0f * vDesO_.tail(3);
      vdEENorm[RIGHT].tail(3) = vdEENorm[RIGHT].tail(3) + 1.0f * vDesO_.tail(3);

    } break;
      // Point to point motion of the object
    case 4: {
      integralVeeD_[LEFT].setZero();
      integralVeeD_[RIGHT].setZero();

      Vector6f xBi = Eigen::VectorXf::Zero(6);
      xBi.head(3) = 0.50f * (X[LEFT] + X[RIGHT]);
      xBi.tail(3) = 0.95f * (X[RIGHT] - X[LEFT]);
      xStarDual = tbi_.inverse() * xBi;

      // Velocity based motion of the object
      Vector6f xDotBi = Eigen::VectorXf::Zero(6);
      Eigen::Vector3f xRel = X[RIGHT] - X[LEFT];
      Vector6f Vo = Eigen::VectorXf::Zero(6);
      Vector6f voPlace = this->generatePlacingMotion2(wHo, wHDo, heightViaPoint_, Vo, false);

      float aFilt = 0.05;
      vDesO_ = (1 - aFilt) * vDesO_ + aFilt * voPlace;
      voPlace = vDesO_;

      float cpObj =
          Utils<float>::computeCouplingFactor(wHo.block<3, 1>(0, 3) - wHDo.block<3, 1>(0, 3), 50.0f, 0.12f, 1.0f, true);

      voPlace.head(3) =
          voPlace.head(3).normalized() * (cpObj * voPlace.head(3).norm() + (1.0f - cpObj) * vdO.head(3).norm());

      Eigen::Vector3f wO = 0.0f * voPlace.tail(3);
      xDotBi.head(3) = voPlace.head(3);
      xDotBi.tail(3) = wO.cross(xRel);

      Vector6f vTaskBi = (a * tbi_ * (xDual - xStarDual) + xDotBi);
      vTaskBi.head(3) = vTaskBi.head(3).normalized() * vdO.head(3).norm();

      activationEENorm = tbi_.inverse() * vTaskBi;
      activationEETang = tbi_.inverse() * vTaskBi;
      activation = 1.0f;

      this->computeDesiredOrientation(1.0f, wHee, wHgp, wHo, qdNorm, false);

      vdEENorm[LEFT].tail(3) = voPlace.tail(3);
      vdEENorm[RIGHT].tail(3) = voPlace.tail(3);

      vDesO_ = voPlace;
    } break;
      // PlaceTossing (fast interrupted placing)
    case 5: {

      a.block<3, 3>(0, 0) = -4.0f * this->gainPosAbs_;
      a.block<3, 3>(3, 3) = -20.0f * this->gainPosRel_;

      integralVeeD_[LEFT].setZero();
      integralVeeD_[RIGHT].setZero();

      Vector6f xBi = Eigen::VectorXf::Zero(6);
      xBi.head(3) = 0.50f * (X[LEFT] + X[RIGHT]);
      xBi.tail(3) = 0.95f * (X[RIGHT] - X[LEFT]);
      xStarDual = tbi_.inverse() * xBi;

      // Velocity based motion of the object
      Vector6f xDotBi = Eigen::VectorXf::Zero(6);
      Eigen::Vector3f xRel = X[RIGHT] - X[LEFT];
      Vector6f Vo = Eigen::VectorXf::Zero(6);
      Vector6f voPlace = this->generatePlacingMotion2(wHo, wHDo, heightViaPoint_, Vo, true);
      float cpObj =
          Utils<float>::computeCouplingFactor(wHo.block<3, 1>(0, 3) - wHDo.block<3, 1>(0, 3), 50.0f, 0.12f, 1.0f, true);

      voPlace.head(3) = voPlace.head(3).normalized() * (vdO.head(3).norm());

      Eigen::Vector3f wO = 0.0f * voPlace.tail(3);
      xDotBi.head(3) = voPlace.head(3);
      xDotBi.tail(3) = wO.cross(xRel);

      Vector6f vTaskBi = (a * tbi_ * (xDual - xStarDual) + xDotBi);
      vTaskBi.head(3) = vTaskBi.head(3).normalized() * vdO.head(3).norm();

      activationEENorm = tbi_.inverse() * vTaskBi;
      activationEETang = tbi_.inverse() * vTaskBi;
      activation = 1.0f;

      this->computeDesiredOrientation(1.0f, wHee, wHgp, wHo, qdNorm, false);

      vdEENorm[LEFT].tail(3) = voPlace.tail(3);
      vdEENorm[RIGHT].tail(3) = voPlace.tail(3);
      vDesO_ = voPlace;
    } break;
  }

  // Get the modulated motion (outMotion: Velocity)
  Vector6f dsEEModulated = Eigen::VectorXf::Zero(6, 1);
  dsEEModulated = this->computeModulatedMotionDual(activation, basisQ, dsEENominal, activationEENorm, activationEETang);

  vDesEE[LEFT].head(3) = dsEEModulated.head(3);
  vDesEE[LEFT].tail(3) = vdEENorm[LEFT].tail(3);
  vDesEE[RIGHT].head(3) = dsEEModulated.tail(3);
  vDesEE[RIGHT].tail(3) = vdEENorm[RIGHT].tail(3);

  // Unitary velocity field
  float speedEE[2];

  Eigen::Vector3f oErrorPosAbsParal = this->getAbsoluteTangentError(wHo, wHee, wHgp);

  float cpAp = Utils<float>::computeCouplingFactor(oErrorPosAbsParal, 50.0f, 0.03f, 1.2f, true);
  float cpAp2 = 0.0f;
  float alp = 1.0f;

  if (modulatedReaching_) {
    if (true) { alp = 0.10f; }

    cpAp2 = 0.0f;
    refVelReach_[LEFT] = (1.0f - alp) * refVelReach_[LEFT]
        + alp * ((1.0f - cpAp) * dsEEModulated.head(3).norm() + cpAp * VdImp[LEFT].norm());
    refVelReach_[RIGHT] = (1.0f - alp) * refVelReach_[RIGHT]
        + alp * ((1.0f - cpAp) * dsEEModulated.tail(3).norm() + cpAp * VdImp[RIGHT].norm());
    speedEE[LEFT] = refVelReach_[LEFT];
    speedEE[RIGHT] = refVelReach_[RIGHT];

  } else if (isNormImpactVel_) {
    cpAp2 = 0.0f;
    refVelReach_[LEFT] = VdImp[LEFT].norm();
    refVelReach_[RIGHT] = VdImp[RIGHT].norm();
    speedEE[LEFT] = refVelReach_[LEFT];
    speedEE[RIGHT] = refVelReach_[RIGHT];
  } else {
    cpAp2 = Utils<float>::computeCouplingFactor(oErrorPosAbsParal, 50.0f, 0.10f, 1.5f, true);
    alp = 0.10f;
    refVelReach_[LEFT] = (1.0f - alp) * refVelReach_[LEFT]
        + alp * ((1.0f - cpAp) * dsEENominal.head(3).norm() + cpAp * VdImp[LEFT].norm());
    refVelReach_[RIGHT] = (1.0f - alp) * refVelReach_[RIGHT]
        + alp * ((1.0f - cpAp) * dsEENominal.tail(3).norm() + cpAp * VdImp[RIGHT].norm());
    speedEE[LEFT] = refVelReach_[LEFT];
    speedEE[RIGHT] = refVelReach_[RIGHT];
  }

  if (taskType == 0) {
    speedEE[LEFT] = refVelReach_[LEFT];
    speedEE[RIGHT] = refVelReach_[RIGHT];
  } else {
    speedEE[LEFT] = dsEEModulated.head(3).norm();
    speedEE[RIGHT] = dsEEModulated.tail(3).norm();
  }

  vDesEE[LEFT].head(3) = vDesEE[LEFT].head(3).normalized() * speedEE[LEFT];
  vDesEE[RIGHT].head(3) = vDesEE[RIGHT].head(3).normalized() * speedEE[RIGHT];

  qd[LEFT] = qdNorm[LEFT];
  qd[RIGHT] = qdNorm[RIGHT];
}

Eigen::Vector3f DualArmFreeMotionController::getAbsoluteTangentError(Eigen::Matrix4f wHo,
                                                                     Eigen::Matrix4f wHee[],
                                                                     Eigen::Matrix4f wHgp[]) {

  Eigen::Vector3f normalL = wHgp[0].block<3, 1>(0, 2);
  Eigen::Matrix3f oSpace = Utils<float>::orthogonalProjector(normalL);

  Eigen::Vector3f errorPAbs =
      wHo.block(0, 3, 3, 1) - 0.5f * (wHee[LEFT].block(0, 3, 3, 1) + wHee[RIGHT].block(0, 3, 3, 1));

  return oSpace * errorPAbs;
}

void DualArmFreeMotionController::computeEEAvoidanceVelocity(Eigen::Matrix4f wHee[], Vector6f (&vEEOA)[NB_ROBOTS]) {

  Eigen::Vector3f centerSphereEELeft =
      wHee[LEFT].block(0, 3, 3, 1) + (0.5f * minDistEE_ - safeRadius_) * wHee[LEFT].block(0, 0, 3, 3).col(2);
  Eigen::Vector3f centerSphereEERight =
      wHee[RIGHT].block(0, 3, 3, 1) + (0.5f * minDistEE_ - safeRadius_) * wHee[RIGHT].block(0, 0, 3, 3).col(2);

  float distEE = (centerSphereEELeft - centerSphereEERight).norm() - 2.0f * safeRadius_;
  float alphaActive = 0.5f * (std::tanh(swEEObsAv_ * (minDistEE_ - distEE)) + 1.0);

  vEEOA[LEFT].head(3) =
      -alphaActive * vMax_ * (wHee[RIGHT].block(0, 3, 3, 1) - wHee[LEFT].block(0, 3, 3, 1)).normalized();
  vEEOA[RIGHT].head(3) =
      -alphaActive * vMax_ * (wHee[LEFT].block(0, 3, 3, 1) - wHee[RIGHT].block(0, 3, 3, 1)).normalized();
}

void DualArmFreeMotionController::getCoordinatedTranslation(Eigen::Vector3f xEE[],
                                                            Eigen::Vector3f xGP[],
                                                            Eigen::Vector3f xStd[],
                                                            Eigen::Matrix3f wRo,
                                                            Eigen::Vector3f (&vdEE)[NB_ROBOTS]) {
  Eigen::Vector3f xAbsEE = 0.5f * (xEE[RIGHT] + xEE[LEFT]);
  Eigen::Vector3f xAbsGp = 0.5f * (xGP[RIGHT] + xGP[LEFT]);
  Eigen::Vector3f xAbsStb = 0.5f * (xStd[RIGHT] + xStd[LEFT]);

  Eigen::Vector3f xRelEE = (xEE[RIGHT] - xEE[LEFT]);
  Eigen::Vector3f xRelGP = (xGP[RIGHT] - xGP[LEFT]);
  Eigen::Vector3f xRelStb = (xStd[RIGHT] - xStd[LEFT]);

  Eigen::Vector3f xRelPGrasp = xRelGP;
  xRelPGrasp(1) = xRelGP(1) / fabs(xRelGP(1)) * (fabs(xRelGP(1)) + 0.30f);

  Eigen::Vector3f dPAbs = reachableP_ * xAbsGp + (1.0f - reachableP_) * xAbsStb;

  // =======================================
  // Absolute velocity of the End-effectors
  // =======================================
  Eigen::Vector3f errorAbs = xAbsEE - dPAbs;

  // computing the velocity
  // ~~~~~~~~~~~~~~~~~~~~~~~
  Eigen::Vector3f vAbs = -3.0f * gainPosAbs_ * errorAbs;

  Eigen::Vector3f oErrorPosAbs = wRo.transpose() * errorAbs;
  Eigen::Vector3f oErrorPosAbsParal = Eigen::Vector3f(oErrorPosAbs(0), 0.0f, oErrorPosAbs(2));
  float cpAp = Utils<float>::computeCouplingFactor(oErrorPosAbsParal, 50.0f, 0.06f, 1.0f, true);
  float cplRel = Utils<float>::computeCouplingFactor(errorAbs, 50.0f, 0.08f, 1.0f, true);//

  Eigen::Vector3f dPRel = cplRel * (cpAp * xRelGP + (1.0f - cpAp) * xRelPGrasp) + (1.0f - cplRel) * xRelStb;// TBC
  Eigen::Vector3f errorRel = xRelEE - dPRel;

  // =====================================
  // Relative velocity of the hands
  // =====================================

  // computing the velocity
  Eigen::Vector3f vRel = -5.0f * gainPosRel_ * errorRel;

  // ========================================
  // Computation of individual EE motion
  // ========================================
  // velocity
  float aBi = 0.5f;
  float bBi = 1.0f;

  vdEE[LEFT] = vAbs - (1 - aBi) * vRel;
  vdEE[RIGHT] = vAbs + aBi * vRel;
}

Eigen::Vector2f DualArmFreeMotionController::predictRobotTranslation(Eigen::Matrix4f wHee[],
                                                                     Eigen::Matrix4f wHgp[],
                                                                     Eigen::Matrix4f wHEEStandby[],
                                                                     Eigen::Matrix4f wHo,
                                                                     Eigen::Vector3f xRelease,
                                                                     float vtoss,
                                                                     float tolerancDistToContact,
                                                                     float dt,
                                                                     float speedScaling) {

  Eigen::Vector3f xEE[NB_ROBOTS], xGP[NB_ROBOTS], xStd[NB_ROBOTS], xObj;

  xEE[LEFT] = wHee[LEFT].block(0, 3, 3, 1);
  xEE[RIGHT] = wHee[RIGHT].block(0, 3, 3, 1);
  xGP[LEFT] = wHgp[LEFT].block(0, 3, 3, 1);
  xGP[RIGHT] = wHgp[RIGHT].block(0, 3, 3, 1);
  xStd[LEFT] = wHEEStandby[LEFT].block(0, 3, 3, 1);
  xStd[RIGHT] = wHEEStandby[RIGHT].block(0, 3, 3, 1);
  xObj = wHo.block<3, 1>(0, 3);
  Eigen::Matrix3f wRo = wHo.block<3, 3>(0, 0);
  Eigen::Vector3f vdEE[NB_ROBOTS], vdO;

  bool isTossingCommand = false;
  bool isContact = false;
  bool isReleasePositionReached = false;
  float tolDist2contact = tolerancDistToContact;
  float aBi = 0.5f;
  float bBi = 1.0f;

  int maxHorizon = 30;
  int predCount = 0;

  std::vector<Eigen::Vector3f> xNextLeft, xNextRight;
  std::vector<Eigen::Vector3f> dxNextLeft, dxNextRight;
  Eigen::Vector3f xEEInitLeft;
  Eigen::Vector3f xEEInitRight;

  xEEInitLeft = xEE[LEFT];
  xEEInitRight = xEE[RIGHT];

  // Robot path length and
  Eigen::Vector2f lpDxAvg = {1e-4, 1e-4};

  while ((!isReleasePositionReached) && (predCount < maxHorizon)) {
    // Update contact status
    // Positioning error
    Eigen::Vector3f lhEr = (xGP[LEFT] - xEE[LEFT]);
    Eigen::Vector3f rhEr = (xGP[RIGHT] - xEE[RIGHT]);

    // Distances to contacts
    if ((lhEr.norm() <= tolDist2contact) && (rhEr.norm() <= tolDist2contact)) {
      isContact = true;
    } else {
      isContact = false;
    }

    if (!isContact) {
      this->getCoordinatedTranslation(xEE, xGP, xStd, wRo, vdEE);
      vdO.setZero();
    } else {
      vdO = vtoss * (xRelease - xObj).normalized();

      Eigen::Vector3f vAbs = vdO;
      Eigen::Vector3f vRel = Eigen::Vector3f(0, 0, 0);

      vdEE[LEFT] = vAbs - (1 - aBi) * vRel;
      vdEE[RIGHT] = vAbs + aBi * vRel;
    }

    vdEE[LEFT] *= speedScaling;
    vdEE[RIGHT] *= speedScaling;
    vdO *= speedScaling;

    // Update position from velocity
    xEE[LEFT] = xEE[LEFT] + dt * vdEE[LEFT];
    xEE[RIGHT] = xEE[RIGHT] + dt * vdEE[RIGHT];
    xGP[LEFT] = xGP[LEFT] + dt * vdO;
    xGP[RIGHT] = xGP[RIGHT] + dt * vdO;
    xObj = xObj + dt * vdO;

    // Update Convergence status
    isTossingCommand = ((xObj - xRelease).norm() <= 0.05);

    if ((isTossingCommand)) { isReleasePositionReached = true; }

    // Extract the translation state
    xNextLeft.push_back(xEE[LEFT] - xEEInitLeft);
    xNextRight.push_back(xEE[RIGHT] - xEEInitRight);
    dxNextLeft.push_back(vdEE[LEFT]);
    dxNextRight.push_back(vdEE[RIGHT]);

    xEEInitLeft = xEE[LEFT];
    xEEInitRight = xEE[RIGHT];

    predCount++;
  }

  int sizeXNext = xNextLeft.size();
  float vAvg = 0.0f;
  float lpXdX = 0.0f;

  for (int i = 0; i < sizeXNext; i++) {
    vAvg += 0.5f * (dxNextLeft[i].norm() + dxNextRight[i].norm()) / sizeXNext;
    lpXdX += 0.5f * (xNextLeft[i].norm() + xNextRight[i].norm());
  }

  lpDxAvg << lpXdX, vAvg;

  return lpDxAvg;
}

Eigen::MatrixXf DualArmFreeMotionController::getBimanualGraspMx(const Eigen::Matrix4f& wHo, Eigen::Matrix4f wHgp[]) {

  Eigen::MatrixXf graspMxObj = Eigen::MatrixXf::Zero(6, 12);

  Eigen::Vector3f tog[2];
  Eigen::Matrix3f skewMxOg[2];

  for (int k = 0; k < 2; k++) {
    tog[k] = wHo.block<3, 1>(0, 3) - wHgp[k].block<3, 1>(0, 3);

    skewMxOg[k] << 0.0f, -tog[k](2), tog[k](1), tog[k](2), 0.0f, -tog[k](0), -tog[k](1), tog[k](0), 0.0f;
  }

  graspMxObj.leftCols(6) = Eigen::MatrixXf::Identity(6, 6);
  graspMxObj.rightCols(6) = Eigen::MatrixXf::Identity(6, 6);
  graspMxObj.block<3, 3>(3, 0) = skewMxOg[LEFT];
  graspMxObj.block<3, 3>(3, 6) = skewMxOg[RIGHT];

  return graspMxObj;
}

Vector6f DualArmFreeMotionController::computeDesiredTaskTwist(const Eigen::Matrix4f& wHc, const Eigen::Matrix4f& wHd) {

  Vector6f errorEE;
  errorEE.setZero();
  Vector6f desTwistEE;
  desTwistEE.setZero();

  Eigen::Matrix3f dRc = wHc.block<3, 3>(0, 0) * wHd.block<3, 3>(0, 0).transpose();
  Eigen::AngleAxisf dAxisAngleC(dRc);
  Eigen::Vector3f dAxis = dAxisAngleC.axis();

  errorEE.head(3) = wHc.block<3, 1>(0, 3) - wHd.block<3, 1>(0, 3);
  errorEE.tail(3) = dAxisAngleC.axis().normalized() * dAxisAngleC.angle();

  // ---------------------------------
  // computing of desired ee velocity
  // ---------------------------------
  float gainTheta = 0.5f * (std::tanh(30 * (0.25 - errorEE.tail(3).norm())) + 1.0);
  float gnAdapt = (1 + 2.f * gainTheta);
  desTwistEE.head(3) = -4.0f * gainPosAbs_ * errorEE.head(3);
  desTwistEE.tail(3) = -1.0f * gainOriAbs_ * gnAdapt * errorEE.tail(3);
  desTwistEE = Utils<float>::saturationTwist(vMax_, wMax_, desTwistEE);

  return desTwistEE;
}

void DualArmFreeMotionController::setVirtualObjectFrame(Eigen::Matrix4f wHVo) { wHvo_ = wHVo; }
void DualArmFreeMotionController::setDt(float dt) { dt_ = dt; }
void DualArmFreeMotionController::setReachableP(float reachableP) { reachableP_ = reachableP; }
void DualArmFreeMotionController::setWHEEStandby(Eigen::Matrix4f wHEEStandby, int robotID) {
  wHEEStandby_[robotID] = wHEEStandby;
}
void DualArmFreeMotionController::setDesVelReach(float desVelReach) { desVelReach_ = desVelReach; }
void DualArmFreeMotionController::setRefVelReach(float refVelReach, int robotID) {
  refVelReach_[robotID] = refVelReach;
}
void DualArmFreeMotionController::setModulatedReaching(bool modulatedReaching) {
  modulatedReaching_ = modulatedReaching;
}
void DualArmFreeMotionController::setIsNormImpactVel(bool isNormImpactVel) { isNormImpactVel_ = isNormImpactVel; }
void DualArmFreeMotionController::setHeightViaPoint(float heightViaPoint) { heightViaPoint_ = heightViaPoint; }
void DualArmFreeMotionController::setObjectDim(Eigen::Vector3f objectDim) { objectDim_ = objectDim; }
void DualArmFreeMotionController::setActivationAperture(float activationAperture) {
  activationAperture_ = activationAperture;
}

Vector6f DualArmFreeMotionController::getDesObjectMotion() { return vDesO_; }
float DualArmFreeMotionController::getActivationProximity() { return activationProximity_; }
float DualArmFreeMotionController::getActivationNormal() { return activationNormal_; }
float DualArmFreeMotionController::getActivationTangent() { return activationTangent_; }
float DualArmFreeMotionController::getActivationRetract() { return activationRetract_; }
float DualArmFreeMotionController::getActivationRelease() { return activationRelease_; }
