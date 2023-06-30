/** Class ObjectToGrasp

*/

#pragma once

#ifndef TOSSING_TARGET_H
#define TOSSING_TARGET_H

#include "iam_dual_arm_control/tools/Utils.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "sg_filter.h"
#include <deque>
#include <vector>

#include "iam_dual_arm_control/tools/KalmanFilter3DVelFromPosEstimator.hpp"

#define NB_ROBOTS 2                // Number of robots
#define MOVING_FORCE_WINDOW_SIZE 10// Window's size used to average the force data and detect peristent contact
#define NB_OBJECTS 3               // Number of objects

class tossing_target {

public:
  // target (tossing)
  Eigen::Vector3f _xt;
  Eigen::Vector4f _qt;
  Eigen::Vector3f _vt;
  Eigen::Vector3f _wt;

  Eigen::Vector3f _xd_landing;
  // Eigen::Vector3f _x_pickup;
  Eigen::Vector3f _x_intercept;// intercept point of the moving object
  Eigen::Vector3f _xt_state2go;

  std::unique_ptr<SGF::SavitzkyGolayFilter> _xt_filtered;// target
  KalmanFilter3DVelFromPosEstimator _xt_KF_filtered;     //

  tossing_target(){};
  ~tossing_target(){};

  void init_target(int dim, int order, int win_l, float dt) {
    // target
    _xt.setZero();
    _qt.setZero();
    _vt.setZero();
    _wt.setZero();
    _qt << 1.0f, 0.0f, 0.0f, 0.0f;
    // _x_pickup.setZero();
    _xt_state2go.setZero();
    //
    _xt_filtered =
        std::make_unique<SGF::SavitzkyGolayFilter>(dim, order, win_l, dt);//(3,3,10,dt); dim, order, win_l, dt
    _xt_KF_filtered.init(dt, Eigen::Vector2f(0.004, 0.1), 0.004, _xt);
    _xt_KF_filtered.update(_xt);
  }

  void get_filtered_state() {
    // filtered target position
    SGF::Vec temp(3);
    _xt_filtered->AddData(_xt);
    _xt_filtered->GetOutput(0, temp);
    _xt = temp;
    _xt_filtered->GetOutput(1, temp);
    _vt = temp;
    _xt_KF_filtered.update(_vt);
    _vt = _xt_KF_filtered.get_estimate_position();
  }
};

#endif// TOSSING_TARGET_H