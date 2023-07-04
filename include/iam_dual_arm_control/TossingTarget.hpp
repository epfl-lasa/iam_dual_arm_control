#pragma once

#include <deque>
#include <vector>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "sg_filter.h"

#include "iam_dual_arm_control/tools/KalmanFilter3DVelFromPosEstimator.hpp"
#include "iam_dual_arm_control/tools/Utils.hpp"

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

  void init(int dim, int order, int win_l, float dt) {
    // Target
    xt_.setZero();
    qt_.setZero();
    vt_.setZero();
    qt_ << 1.0f, 0.0f, 0.0f, 0.0f;
    xtStateToGo_.setZero();

    xtFiltered_ = std::make_unique<SGF::SavitzkyGolayFilter>(dim, order, win_l, dt);
    xtKalmanFiltered_.init(dt, Eigen::Vector2f(0.004, 0.1), 0.004, xt_);
    xtKalmanFiltered_.update(xt_);
  }

  void getFilteredState() {
    // Filtered target position
    SGF::Vec temp(3);
    xtFiltered_->AddData(xt_);
    xtFiltered_->GetOutput(0, temp);
    xt_ = temp;
    xtFiltered_->GetOutput(1, temp);
    vt_ = temp;
    xtKalmanFiltered_.update(vt_);
    vt_ = xtKalmanFiltered_.get_estimate_position();
  }

  Eigen::Vector3f getXt() { return xt_; }
  Eigen::Vector4f getQt() { return qt_; }
  Eigen::Vector3f getVt() { return vt_; }
  Eigen::Vector3f getXdLanding() { return xdLanding_; }
  Eigen::Vector3f getXIntercept() { return xIntercept_; }
  Eigen::Vector3f getXtStateToGo() { return xtStateToGo_; }

  Eigen::Vector3f setXt(Eigen::Vector3f newValue) { xt_ = newValue; }
  Eigen::Vector4f setQt(Eigen::Vector4f newValue) { qt_ = newValue; }
  Eigen::Vector3f setXdLanding(Eigen::Vector3f newValue) { xdLanding_ = newValue; }
  Eigen::Vector3f setXIntercept(Eigen::Vector3f newValue) { xIntercept_ = newValue; }
  Eigen::Vector3f setXtStateToGo(Eigen::Vector3f newValue) { xtStateToGo_ = newValue; }
};
