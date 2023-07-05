
#include "iam_dual_arm_control/DualArmCooperativeController.hpp"

// BWC Solver variables
bwc_Vars bwc_vars;
bwc_Params bwc_params;
bwc_Workspace bwc_work;
bwc_Settings bwc_settings;

DualArmCooperativeController::DualArmCooperativeController() {}
DualArmCooperativeController::~DualArmCooperativeController() {}

bool DualArmCooperativeController::init() {
  toleranceDistToContact_ = 0.055f;
  contactConfidence_ = 0.0f;
  minFz_ = 30.0f;
  minNF_ = 30.0f;
  maxNF_ = 45.0f;

  muEE_ = 0.9f;
  gammaEE_ = 0.9f;
  deltaXEE_ = 0.05f;
  deltaYEE_ = 0.05f;
  contactOccured_ = false;
  targetForce_ = 45.0f;

  graspMatrixEEs_.setZero();
  optimalContactWrenchEE_.setZero();
  optimalSlack_.setZero();

  withForceSaturation_ = true;

  for (int k = 0; k < NB_ROBOTS; k++) {
    distToContact_[k] = 1.0f;
    worldXstartDesiredEE_[k].setIdentity();
    wrenchCorrectionEE_[k].setZero();
    complementaryConstraintMatrix_[k].setZero();
    contactConstraintVector_[k].setZero();
    contactConstraintMatrix_[k].setZero();
    forceApplied_[k].setZero();
    ForceInEE_[k].setZero();
    normalVectToObjSurface_[k].setZero();
  }

  weigthEEsWrench_.setOnes();
  weigthEEsWrench_.segment(0, 3) *= 50.0e-2; //Forces
  weigthEEsWrench_.segment(3, 3) *= 500.0e-2;//Moments
  weigthEEsWrench_.segment(6, 3) *= 50.0e-2; //Forces
  weigthEEsWrench_.segment(9, 3) *= 500.0e-2;//Moments

  weigthEEsSlack_.setZero();
  weigthEEsSlack_ << 100.0, 100.0, 100.0, 200.0, 200.0, 200.0;

  // Initialization of the cvxgen solver for the cooperative manipulation
  bwc_set_defaults();
  bwc_setup_indexing();

  return true;
}

void DualArmCooperativeController::getGraspKineDynVariables(Eigen::Matrix4f wHo,
                                                            Eigen::Matrix4f wHee[],
                                                            Eigen::Matrix4f wHcp[]) {
  // Update the grasp matrix
  this->computeBimanualGraspMatrix(wHo, wHee, graspMatrixEEs_);

  for (int k = 0; k < NB_ROBOTS; k++) {
    worldXstartDesiredEE_[k].block<3, 3>(0, 0) = wHee[k].block<3, 3>(0, 0);
    worldXstartDesiredEE_[k].block<3, 3>(3, 3) = wHee[k].block<3, 3>(0, 0);
    normalVectToObjSurface_[k] = wHcp[k].block(0, 0, 3, 3).col(2);

    this->setMinNormalForcesEEs(minFz_, wHee[k].block<3, 3>(0, 0), wrenchCorrectionEE_[k]);
  }
}

void DualArmCooperativeController::checkContactProximity(Eigen::Matrix4f wHee[],
                                                         Eigen::Matrix4f wHcp[],
                                                         bool isForceDetected) {
  // Positioning error in hand frames
  Eigen::Vector3f leftHandError =
      wHcp[LEFT].block<3, 3>(0, 0).transpose() * (wHcp[LEFT].block<3, 1>(0, 3) - wHee[LEFT].block<3, 1>(0, 3));
  Eigen::Vector3f rightHandError =
      wHcp[RIGHT].block<3, 3>(0, 0).transpose() * (wHcp[RIGHT].block<3, 1>(0, 3) - wHee[RIGHT].block<3, 1>(0, 3));

  // If norm in x and y are less than thrxy and z less than tol
  bool tsk = false;
  if ((leftHandError.head(2).norm() <= 1.5f * toleranceDistToContact_)
      && (rightHandError.head(2).norm() <= 1.5f * toleranceDistToContact_)) {
    tsk = true;
    contactOccured_ = true;
  } else {
    tsk = false;
  }

  // Distances normal to contacts
  distToContact_[LEFT] = (leftHandError(2));
  distToContact_[RIGHT] = (rightHandError(2));

  // Update the Contact confidence indicator
  if ((fabs(distToContact_[RIGHT]) <= toleranceDistToContact_)
      && (fabs(distToContact_[LEFT]) <= toleranceDistToContact_) && isForceDetected) {
    contactConfidence_ = 1.0;
  } else {
    contactConfidence_ = 0.0;
  }
}

bool DualArmCooperativeController::computeBimanualGraspMatrix(Eigen::Matrix4f wHo,
                                                              Eigen::Matrix4f wHee[],
                                                              Eigen::Matrix<float, 6, 12>& graspMxHands) {

  graspMxHands.setZero();

  Eigen::Matrix3f skewMx[NB_ROBOTS];
  for (int i = 0; i < NB_ROBOTS; i++) {
    Eigen::Vector3f t = wHee[i].block<3, 1>(0, 3) - wHo.block<3, 1>(0, 3);
    skewMx[i] << 0.0f, -t(2), t(1), t(2), 0.0f, -t(0), -t(1), t(0), 0.0f;
  }

  // Left EE
  graspMxHands.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();
  graspMxHands.block<3, 3>(3, 0) = -skewMx[LEFT];
  graspMxHands.block<3, 3>(3, 3) = Eigen::Matrix3f::Identity();

  // Right EE
  graspMxHands.block<3, 3>(0, 6) = Eigen::Matrix3f::Identity();
  graspMxHands.block<3, 3>(3, 6) = -skewMx[RIGHT];
  graspMxHands.block<3, 3>(3, 9) = Eigen::Matrix3f::Identity();

  return true;
}

void DualArmCooperativeController::setMinNormalForcesEEs(float min, Eigen::Matrix3f wRh, Vector6f& wrenchW) {
  // Thresholding normal forces
  Vector6f Wrench_h = Eigen::VectorXf::Zero(6);
  Wrench_h(2) = min;

  wrenchW.head<3>() = wRh * Wrench_h.head<3>();
  wrenchW.tail<3>() = wRh * Wrench_h.tail<3>();
}

void DualArmCooperativeController::thresholdNormalForcesEEs(float min,
                                                            float max,
                                                            Eigen::Matrix3f wRh,
                                                            Vector6f& wrenchW) {
  // Thresholding normal forces
  Vector6f Wrench_h;
  Wrench_h.head<3>() = wRh.transpose() * wrenchW.head<3>();
  Wrench_h.tail<3>() = wRh.transpose() * wrenchW.tail<3>();

  if (Wrench_h(2) <= min) Wrench_h(2) = min;
  else if (Wrench_h(2) >= max)
    Wrench_h(2) = max;

  wrenchW.head<3>() = wRh * Wrench_h.head<3>();
  wrenchW.tail<3>() = wRh * Wrench_h.tail<3>();
}

bool DualArmCooperativeController::getComplementaryConstraints(Matrix6f worldXstartDesiredEE[],
                                                               float distToContact[],
                                                               float toleranceDistToContact) {
  // Thresholding of distance to contact
  float thresholdDistToContact[NB_ROBOTS];
  for (int k = 0; k < NB_ROBOTS; k++) {
    thresholdDistToContact[k] = distToContact[k];
    if (fabs(distToContact[k]) <= toleranceDistToContact) { thresholdDistToContact[k] = 0.0; }

    // Transformation from the world to desired lhand frame
    Matrix6f desEE_X_world = worldXstartDesiredEE[k].transpose();
    complementaryConstraintMatrix_[k].block<1, 6>(0, 0) = thresholdDistToContact[k] * desEE_X_world.block<1, 6>(2, 0);
  }
  return true;
}

void DualArmCooperativeController::computeOptimalWrench(Vector6f desiredObjectWrench) {
  float t_ctrl = ros::Time::now().toSec();

  // Get the contact constraints
  this->getContactConstraints(worldXstartDesiredEE_);

  // Get complementary constraints
  this->getComplementaryConstraints(worldXstartDesiredEE_, distToContact_, toleranceDistToContact_);

  // Load the data for the solver
  this->loadWrenchData(desiredObjectWrench);

  bwc_settings.verbose = 0;

  // Compute the optimal solution (wrench and acceleration)
  bwc_solve();

  // Extract the optimal solution vectors

  // contact forces
  for (int i = 0; i < 12; i++) { optimalContactWrenchEE_(i) = bwc_vars.Fh[i]; }

  // slack variables
  for (int i = 0; i < 6; i++) { optimalSlack_(i) = bwc_vars.w[i]; }
}

bool DualArmCooperativeController::getContactConstraints(Matrix6f worldXstarEE[]) {
  for (int k = 0; k < NB_ROBOTS; k++) {

    Matrix6f ee_X_world = worldXstarEE[k].inverse();

    // Inequality Constraint matrix
    contactConstraintMatrix_[k].block<1, 6>(0, 0) = -1.0f * ee_X_world.block<1, 6>(2, 0);
    contactConstraintMatrix_[k].block<1, 6>(1, 0) =
        -muEE_ / sqrt(2.0f) * ee_X_world.block<1, 6>(2, 0) - ee_X_world.block<1, 6>(0, 0);
    contactConstraintMatrix_[k].block<1, 6>(2, 0) =
        -muEE_ / sqrt(2.0f) * ee_X_world.block<1, 6>(2, 0) + ee_X_world.block<1, 6>(0, 0);
    contactConstraintMatrix_[k].block<1, 6>(3, 0) =
        -muEE_ / sqrt(2.0f) * ee_X_world.block<1, 6>(2, 0) - ee_X_world.block<1, 6>(1, 0);
    contactConstraintMatrix_[k].block<1, 6>(4, 0) =
        -muEE_ / sqrt(2.0f) * ee_X_world.block<1, 6>(2, 0) + ee_X_world.block<1, 6>(1, 0);
    contactConstraintMatrix_[k].block<1, 6>(5, 0) =
        -gammaEE_ * ee_X_world.block<1, 6>(2, 0) - ee_X_world.block<1, 6>(5, 0);
    contactConstraintMatrix_[k].block<1, 6>(6, 0) =
        -gammaEE_ * ee_X_world.block<1, 6>(2, 0) + ee_X_world.block<1, 6>(5, 0);
    contactConstraintMatrix_[k].block<1, 6>(7, 0) =
        -deltaXEE_ * ee_X_world.block<1, 6>(2, 0) - ee_X_world.block<1, 6>(4, 0);
    contactConstraintMatrix_[k].block<1, 6>(8, 0) =
        -deltaXEE_ * ee_X_world.block<1, 6>(2, 0) + ee_X_world.block<1, 6>(4, 0);
    contactConstraintMatrix_[k].block<1, 6>(9, 0) =
        -deltaYEE_ * ee_X_world.block<1, 6>(2, 0) - ee_X_world.block<1, 6>(3, 0);
    contactConstraintMatrix_[k].block<1, 6>(10, 0) =
        -deltaYEE_ * ee_X_world.block<1, 6>(2, 0) + ee_X_world.block<1, 6>(3, 0);

    // Inequality Constraint vector
    contactConstraintVector_[k](0) = minNF_; // minimal normal forces
    contactConstraintVector_[k](11) = maxNF_;// maximun normal forces
  }
  return true;
}

void DualArmCooperativeController::loadWrenchData(Vector6f desiredObjectWrench) {

  // Weight hands wrench
  for (int i = 0; i < 12; i++) { bwc_params.QFh[i] = weigthEEsWrench_(i); }

  // Weight hands slack
  for (int i = 0; i < 6; i++) { bwc_params.Qw[i] = weigthEEsSlack_(i); }

  // Desired contact wrench wrench
  for (int i = 0; i < 6; i++) {
    bwc_params.pFh[i] = 0.0f + wrenchCorrectionEE_[LEFT](i);
    bwc_params.pFh[6 + i] = 0.0f + wrenchCorrectionEE_[RIGHT](i);
  }

  // Contact activator
  bwc_params.beta[0] = contactConfidence_;

  // Grasp Matrix
  for (int i = 0; i < 12; i++) {
    bwc_params.Gh_4[i] = graspMatrixEEs_(3, i);
    bwc_params.Gh_5[i] = graspMatrixEEs_(4, i);
    bwc_params.Gh_6[i] = graspMatrixEEs_(5, i);
  }

  for (int i = 0; i < 6; i++) { bwc_params.b1[i] = desiredObjectWrench(i); }

  // Complementary condition
  for (int i = 0; i < 6; i++) {
    bwc_params.Cplh[i] = complementaryConstraintMatrix_[LEFT](0, i);
    bwc_params.Cprh[i] = complementaryConstraintMatrix_[RIGHT](0, i);
  }

  // Contact constaints
  for (int i = 0; i < 6; i++) {
    bwc_params.CLH_1[i] = contactConstraintMatrix_[LEFT](0, i);
    bwc_params.CLH_7[i] = contactConstraintMatrix_[LEFT](6, i);
    bwc_params.CLH_2[i] = contactConstraintMatrix_[LEFT](1, i);
    bwc_params.CLH_8[i] = contactConstraintMatrix_[LEFT](7, i);
    bwc_params.CLH_3[i] = contactConstraintMatrix_[LEFT](2, i);
    bwc_params.CLH_9[i] = contactConstraintMatrix_[LEFT](8, i);
    bwc_params.CLH_4[i] = contactConstraintMatrix_[LEFT](3, i);
    bwc_params.CLH_10[i] = contactConstraintMatrix_[LEFT](9, i);
    bwc_params.CLH_5[i] = contactConstraintMatrix_[LEFT](4, i);
    bwc_params.CLH_11[i] = contactConstraintMatrix_[LEFT](10, i);
    bwc_params.CLH_6[i] = contactConstraintMatrix_[LEFT](5, i);

    bwc_params.CRH_1[i] = contactConstraintMatrix_[RIGHT](0, i);
    bwc_params.CRH_7[i] = contactConstraintMatrix_[RIGHT](6, i);
    bwc_params.CRH_2[i] = contactConstraintMatrix_[RIGHT](1, i);
    bwc_params.CRH_8[i] = contactConstraintMatrix_[RIGHT](7, i);
    bwc_params.CRH_3[i] = contactConstraintMatrix_[RIGHT](2, i);
    bwc_params.CRH_9[i] = contactConstraintMatrix_[RIGHT](8, i);
    bwc_params.CRH_4[i] = contactConstraintMatrix_[RIGHT](3, i);
    bwc_params.CRH_10[i] = contactConstraintMatrix_[RIGHT](9, i);
    bwc_params.CRH_5[i] = contactConstraintMatrix_[RIGHT](4, i);
    bwc_params.CRH_11[i] = contactConstraintMatrix_[RIGHT](10, i);
    bwc_params.CRH_6[i] = contactConstraintMatrix_[RIGHT](5, i);
  }
}

void DualArmCooperativeController::computeControlWrench(Eigen::Matrix4f wHo,
                                                        Eigen::Matrix4f wHee[],
                                                        Eigen::Matrix4f wHcp[],
                                                        Vector6f desiredObjectWrench,
                                                        bool isForceDetected) {

  this->getGraspKineDynVariables(wHo, wHee, wHcp);
  this->checkContactProximity(wHee, wHcp, isForceDetected);
  this->computeOptimalWrench(desiredObjectWrench);

  // Extraction of left and right hands wrenches
  forceApplied_[LEFT] = optimalContactWrenchEE_.head<6>();
  forceApplied_[RIGHT] = optimalContactWrenchEE_.tail<6>();
  // Normal forces saturation and moments correction
  ForceInEE_[LEFT].head<3>() = wHee[LEFT].block<3, 3>(0, 0).transpose() * forceApplied_[LEFT].head<3>();
  ForceInEE_[LEFT].head<3>() = wHee[LEFT].block<3, 3>(0, 0).transpose() * forceApplied_[LEFT].head<3>();
  ForceInEE_[RIGHT].head<3>() = wHee[RIGHT].block<3, 3>(0, 0).transpose() * forceApplied_[RIGHT].tail<3>();
  ForceInEE_[RIGHT].head<3>() = wHee[RIGHT].block<3, 3>(0, 0).transpose() * forceApplied_[RIGHT].tail<3>();

  // Thresholding normal forces
  if (withForceSaturation_) {
    this->thresholdNormalForcesEEs(minNF_, maxNF_, wHee[LEFT].block<3, 3>(0, 0), forceApplied_[LEFT]);
    this->thresholdNormalForcesEEs(minNF_, maxNF_, wHee[RIGHT].block<3, 3>(0, 0), forceApplied_[RIGHT]);
  }
  forceApplied_[LEFT] = contactConfidence_ * forceApplied_[LEFT];
  forceApplied_[RIGHT] = contactConfidence_ * forceApplied_[RIGHT];

  Eigen::Vector3f z_axis = {0.0f, 0.0f, 1.0f};

  forceApplied_[LEFT].head(3) =
      (normalVectToObjSurface_[LEFT].transpose() * forceApplied_[LEFT].head(3)) * normalVectToObjSurface_[LEFT]
      + (z_axis.transpose() * forceApplied_[LEFT].head(3)) * z_axis;
  forceApplied_[RIGHT].head(3) =
      (normalVectToObjSurface_[RIGHT].transpose() * forceApplied_[RIGHT].head(3)) * normalVectToObjSurface_[RIGHT]
      + (z_axis.transpose() * forceApplied_[RIGHT].head(3)) * z_axis;
}

void DualArmCooperativeController::getPredefinedContactForceProfile(bool goHome,
                                                                    int contactState,
                                                                    Eigen::Matrix4f wHo,
                                                                    Eigen::Matrix4f wHee[],
                                                                    Eigen::Matrix4f wHcp[],
                                                                    bool isForceDetected) {

  this->getGraspKineDynVariables(wHo, wHee, wHcp);
  this->checkContactProximity(wHee, wHcp, isForceDetected);

  forceApplied_[LEFT].setZero();
  forceApplied_[RIGHT].setZero();

  if (goHome) {
    forceApplied_[LEFT].setZero();
    forceApplied_[RIGHT].setZero();
  } else {
    if (contactState == CONTACT) {
      forceApplied_[LEFT].head(3) = targetForce_ * normalVectToObjSurface_[LEFT];
      forceApplied_[RIGHT].head(3) = targetForce_ * normalVectToObjSurface_[RIGHT];
    } else if (contactState == CLOSE_TO_CONTACT) {
      forceApplied_[LEFT].head(3) = 3.0f * normalVectToObjSurface_[LEFT];
      forceApplied_[RIGHT].head(3) = 3.0f * normalVectToObjSurface_[RIGHT];
    } else {
      forceApplied_[LEFT].setZero();
      forceApplied_[RIGHT].setZero();
    }
  }
}

void DualArmCooperativeController::getPredefinedContactForceProfile(bool goHome,
                                                                    int contactState,
                                                                    Eigen::Matrix4f wHo,
                                                                    Eigen::Matrix4f wHee[],
                                                                    Eigen::Matrix4f wHcp[],
                                                                    float objectMass,
                                                                    bool isForceDetected) {

  this->getGraspKineDynVariables(wHo, wHee, wHcp);
  this->checkContactProximity(wHee, wHcp, isForceDetected);

  forceApplied_[LEFT].setZero();
  forceApplied_[RIGHT].setZero();

  if (goHome) {
    forceApplied_[LEFT].setZero();
    forceApplied_[RIGHT].setZero();
  } else {
    if (contactState == CONTACT) {
      forceApplied_[LEFT].head(3) =
          targetForce_ * normalVectToObjSurface_[LEFT] + 0.5f * objectMass * Eigen::Vector3f(0.0f, 0.0f, 9.81f);
      forceApplied_[RIGHT].head(3) =
          targetForce_ * normalVectToObjSurface_[RIGHT] + 0.5f * objectMass * Eigen::Vector3f(0.0f, 0.0f, 9.81f);
    } else if (contactState == CLOSE_TO_CONTACT) {
      forceApplied_[LEFT].head(3) = 3.0f * normalVectToObjSurface_[LEFT];
      forceApplied_[RIGHT].head(3) = 3.0f * normalVectToObjSurface_[RIGHT];
    } else {
      forceApplied_[LEFT].setZero();
      forceApplied_[RIGHT].setZero();
    }
  }
}

void DualArmCooperativeController::getAppliedWrenches(bool goHome,
                                                      int contactState,
                                                      Eigen::Matrix4f wHo,
                                                      Eigen::Matrix4f wHee[],
                                                      Eigen::Matrix4f wHcp[],
                                                      Vector6f desiredObjectWrench,
                                                      float objectMass,
                                                      bool qpWrenchGeneration,
                                                      bool isForceDetected) {
  if (qpWrenchGeneration) {
    this->computeControlWrench(wHo, wHee, wHcp, desiredObjectWrench, isForceDetected);
  } else {
    // Using Predefined normal forcve
    this->getPredefinedContactForceProfile(goHome, contactState, wHo, wHee, wHcp, objectMass, isForceDetected);
  }
}

float DualArmCooperativeController::getContactConfidence() { return contactConfidence_; }

Vector6f DualArmCooperativeController::getForceApplied(int robotID) { return forceApplied_[robotID]; }
