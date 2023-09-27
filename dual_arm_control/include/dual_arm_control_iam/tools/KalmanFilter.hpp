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

#include "Eigen/Eigen"

/**
  * @brief Create a Kalman filter with the specified matrices. 
  *   A - System dynamics matrix
  *   C - Output matrix
  *   Q - Process noise covariance
  *   R - Measurement noise covariance
  *   P - Estimate error covariance
  */
class KalmanFilter {
private:
  // Matrices for computation
  Eigen::MatrixXf A_, C_, Q_, R_, P_, K_, P0_;

  // System dimensions
  int m_, n_;

  // Initial and current time
  float t0_, t_;

  // Discrete time step
  float dt_;

  // Is the filter initialized?
  bool isInitialized_;

  // n-size identity
  Eigen::MatrixXf I_;

  // Estimated states
  Eigen::VectorXf xHat_, xHatNew_;

public:
  KalmanFilter(float dt,
               const Eigen::MatrixXf& A,
               const Eigen::MatrixXf& C,
               const Eigen::MatrixXf& Q,
               const Eigen::MatrixXf& R,
               const Eigen::MatrixXf& P) :
      A_(A),
      C_(C), Q_(Q), R_(R), P0_(P), m_(C_.rows()), n_(A_.rows()), dt_(dt), isInitialized_(false), I_(n_, n_), xHat_(n_),
      xHatNew_(n_) {
    I_.setIdentity();
  }

  /**
  * @brief Create a blank estimator.
  */
  KalmanFilter() {}

  /**
  * @brief Initialize the filter with initial states as zero.
  */
  void init() {
    xHat_.setZero();
    P_ = P0_;
    t0_ = 0;
    t_ = t0_;
    isInitialized_ = true;
  }

  /**
  * @brief Initialize the filter with a guess for initial states.
  */
  void init(float t0, const Eigen::VectorXf& x0) {
    xHat_ = x0;
    P_ = P0_;
    this->t0_ = t0;
    t_ = t0_;
    isInitialized_ = true;
  }
  /**
  * @brief Update the estimated state based on measured values. The
  * time step is assumed to remain constant.
  */
  void update(const Eigen::VectorXf& y) {

    if (!isInitialized_) throw std::runtime_error("Filter is not initialized!");

    xHatNew_ = A_ * xHat_;
    P_ = A_ * P_ * A_.transpose() + Q_;
    K_ = P_ * C_.transpose() * (C_ * P_ * C_.transpose() + R_).inverse();
    xHatNew_ += K_ * (y - C_ * xHatNew_);
    P_ = (I_ - K_ * C_) * P_;
    xHat_ = xHatNew_;

    t_ += dt_;
  }
  /**
  * @brief Update the estimated state based on measured values,
  * using the given time step and dynamics matrix.
  */
  void update(const Eigen::VectorXf& y, float dt, const Eigen::MatrixXf A) {

    this->A_ = A;
    this->dt_ = dt;
    update(y);
  }
  /**
  * @brief Return the current state and time.
  */
  Eigen::VectorXf state() { return xHat_; };
  float time() { return t_; };

  void setState(const Eigen::VectorXf& xHat) { xHat_ = xHat; }
};
