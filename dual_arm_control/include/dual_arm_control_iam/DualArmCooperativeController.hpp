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

extern "C" {
#include "bwc_solver.h"
}

#include "dual_arm_control_iam/tools/Utils.hpp"

#define NB_ROBOTS 2// Number of robots

typedef Eigen::Matrix<float, 6, 1> Vector6f;
typedef Eigen::Matrix<float, 6, 6> Matrix6f;

class DualArmCooperativeController {

private:
  float toleranceDistToContact_;
  float distToContact_[NB_ROBOTS];
  float minFz_;
  float muEE_;
  float gammaEE_;
  float deltaXEE_;
  float deltaYEE_;
  float minNF_;
  float maxNF_;
  float targetForce_;

  bool withForceSaturation_;
  bool contactOccured_;

  // EE
  Eigen::Matrix<float, 6, 12> graspMatrixEEs_;
  Matrix6f worldXstartDesiredEE_[NB_ROBOTS];
  Vector6f wrenchCorrectionEE_[NB_ROBOTS];
  Eigen::Matrix<float, 1, 6> complementaryConstraintMatrix_[NB_ROBOTS];
  Eigen::Matrix<float, 12, 1> contactConstraintVector_[NB_ROBOTS];
  Eigen::Matrix<float, 11, 6> contactConstraintMatrix_[NB_ROBOTS];
  Vector6f optimalSlack_;

  Eigen::Matrix<double, 12, 1> weigthEEsWrench_;
  Eigen::Matrix<double, 6, 1> weigthEEsSlack_;

  Eigen::Vector3f normalVectToObjSurface_[NB_ROBOTS];// Normal vector to surface object for each robot (3x1)

  float contactConfidence_;

  Eigen::Matrix<float, 12, 1> optimalContactWrenchEE_;

  Vector6f forceApplied_[NB_ROBOTS];
  Vector6f ForceInEE_[NB_ROBOTS];

public:
  /** 
   * Robot ID: left or right 
  */
  enum ROBOT { LEFT = 0, RIGHT = 1 };

  /**
   * Contact state:
   * CONTACT: Both robots are in contact with the object
   * CLOSE_TO_CONTACT: Both robots are close to make contact with the object
   * NO_CONTACT: Both robots are not in contact with the object
  */
  enum ContactState { CONTACT = 0, CLOSE_TO_CONTACT = 1, NO_CONTACT = 2 };

  /**
   * Exection mode:
   * REACHING_GRASPING_ONLY: The two robots reach and grasp the object
   * REACHING_GRASPING_MANIPULATING: The two robots reach, grasp and move the object to a predefined position
  */
  enum Mode { REACHING_GRASPING = 0, REACHING_GRASPING_MANIPULATING = 1 };

  DualArmCooperativeController();
  ~DualArmCooperativeController();

  bool init();
  void checkContactProximity(Eigen::Matrix4f wHee[], Eigen::Matrix4f wHcp[], bool isForceDetected);

  void getGraspKineDynVariables(Eigen::Matrix4f wHo, Eigen::Matrix4f wHee[], Eigen::Matrix4f wHcp[]);
  bool
  computeBimanualGraspMatrix(Eigen::Matrix4f wHo, Eigen::Matrix4f wHee[], Eigen::Matrix<float, 6, 12>& graspMxHands);
  void setMinNormalForcesEEs(float min, Eigen::Matrix3f wRh, Vector6f& wrenchW);
  void thresholdNormalForcesEEs(float min, float max, Eigen::Matrix3f wRh, Vector6f& wrenchW);
  void computeOptimalWrench(Vector6f desiredObjectWrench);
  bool
  getComplementaryConstraints(Matrix6f worldXstartDesiredEE[], float distToContact[], float toleranceDistToContact);
  bool getContactConstraints(Matrix6f worldXstarEE[]);
  void loadWrenchData(Vector6f desiredObjectWrench);
  void computeControlWrench(Eigen::Matrix4f wHo,
                            Eigen::Matrix4f wHee[],
                            Eigen::Matrix4f wHcp[],
                            Vector6f desiredObjectWrench,
                            bool isForceDetected);
  void getPredefinedContactForceProfile(bool goHome,
                                        int contactState,
                                        Eigen::Matrix4f wHo,
                                        Eigen::Matrix4f wHee[],
                                        Eigen::Matrix4f wHcp[],
                                        bool isForceDetected);
  void getAppliedWrenches(bool goHome,
                          int contactState,
                          Eigen::Matrix4f wHo,
                          Eigen::Matrix4f wHee[],
                          Eigen::Matrix4f wHcp[],
                          Vector6f desiredObjectWrench,
                          float objectMass,
                          bool qpWrenchGeneration,
                          bool isForceDetected);
  void getPredefinedContactForceProfile(bool goHome,
                                        int contactState,
                                        Eigen::Matrix4f wHo,
                                        Eigen::Matrix4f wHee[],
                                        Eigen::Matrix4f wHcp[],
                                        float objectMass,
                                        bool isForceDetected);
  float getContactConfidence();

  bool setContactConfidence();

  Vector6f getForceApplied(int robotID);
};
