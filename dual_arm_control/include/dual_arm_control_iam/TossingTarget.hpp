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

#include <deque>
#include <vector>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "sg_filter.h"

#include "dual_arm_control_iam/tools/KalmanFilter3DVelFromPosEstimator.hpp"
#include "dual_arm_control_iam/tools/Utils.hpp"

#define NB_ROBOTS 2                // Number of robots
#define MOVING_FORCE_WINDOW_SIZE 10// Window's size used to average the force data and detect peristent contact
#define NB_OBJECTS 3               // Number of objects

class TossingTarget {
private:
  // target (tossing)
  Eigen::Vector3f xt_;
  Eigen::Vector4f qt_;
  Eigen::Vector3f vt_;

  Eigen::Vector3f xdLanding_;
  Eigen::Vector3f xIntercept_;// intercept point of the moving object
  Eigen::Vector3f xtStateToGo_;

  std::unique_ptr<SGF::SavitzkyGolayFilter> xtFiltered_;// target
  KalmanFilter3DVelFromPosEstimator xtKalmanFiltered_;

public:
  TossingTarget(){};
  ~TossingTarget(){};

  void init(int sgfPos[], float dt) {//(int dim, int order, int winL, float dt) {
    // Target
    xt_.setZero();
    qt_.setZero();
    vt_.setZero();
    qt_ << 1.0f, 0.0f, 0.0f, 0.0f;
    xtStateToGo_.setZero();

    xtFiltered_ = std::make_unique<SGF::SavitzkyGolayFilter>(sgfPos[0], sgfPos[1], sgfPos[2], dt);
    xtKalmanFiltered_.init(dt, Eigen::Vector2f(0.004, 0.1), 0.004, xt_);
    xtKalmanFiltered_.update(xt_);
  }

  void computeFilteredState() {
    // Filtered target position
    SGF::Vec temp(3);
    xtFiltered_->AddData(xt_);
    xtFiltered_->GetOutput(0, temp);
    xt_ = temp;
    xtFiltered_->GetOutput(1, temp);
    vt_ = temp;
    xtKalmanFiltered_.update(vt_);
    vt_ = xtKalmanFiltered_.getEstimatePosition();
  }

  Eigen::Vector3f getXt() { return xt_; }
  Eigen::Vector4f getQt() { return qt_; }
  Eigen::Vector3f getVt() { return vt_; }
  Eigen::Vector3f getXdLanding() { return xdLanding_; }
  Eigen::Vector3f getXIntercept() { return xIntercept_; }
  Eigen::Vector3f getXtStateToGo() { return xtStateToGo_; }

  void setXt(Eigen::Vector3f newValue) { xt_ = newValue; }
  void setQt(Eigen::Vector4f newValue) { qt_ = newValue; }
  void setXdLanding(Eigen::Vector3f newValue) { xdLanding_ = newValue; }
  void setXIntercept(Eigen::Vector3f newValue) { xIntercept_ = newValue; }
  void setXtStateToGo(Eigen::Vector3f newValue) { xtStateToGo_ = newValue; }
};
