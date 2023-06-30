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
  Eigen::MatrixXf A, C, Q, R, P, K, P0;

  // System dimensions
  int m, n;

  // Initial and current time
  float t0, t;

  // Discrete time step
  float dt;

  // Is the filter initialized?
  bool initialized;

  // n-size identity
  Eigen::MatrixXf I;

  // Estimated states
  Eigen::VectorXf x_hat, x_hat_new;

public:
  KalmanFilter(float dt,
               const Eigen::MatrixXf& A,
               const Eigen::MatrixXf& C,
               const Eigen::MatrixXf& Q,
               const Eigen::MatrixXf& R,
               const Eigen::MatrixXf& P) :
      A(A),
      C(C), Q(Q), R(R), P0(P), m(C.rows()), n(A.rows()), dt(dt), initialized(false), I(n, n), x_hat(n), x_hat_new(n) {
    I.setIdentity();
  }

  /**
  * @brief Create a blank estimator.
  */
  KalmanFilter() {}

  /**
  * @brief Initialize the filter with initial states as zero.
  */
  void init() {
    x_hat.setZero();
    P = P0;
    t0 = 0;
    t = t0;
    initialized = true;
  }

  /**
  * @brief Initialize the filter with a guess for initial states.
  */
  void init(float t0, const Eigen::VectorXf& x0) {
    x_hat = x0;
    P = P0;
    this->t0 = t0;
    t = t0;
    initialized = true;
  }
  /**
  * @brief Update the estimated state based on measured values. The
  * time step is assumed to remain constant.
  */
  void update(const Eigen::VectorXf& y) {

    if (!initialized) throw std::runtime_error("Filter is not initialized!");

    x_hat_new = A * x_hat;
    P = A * P * A.transpose() + Q;
    K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
    x_hat_new += K * (y - C * x_hat_new);
    P = (I - K * C) * P;
    x_hat = x_hat_new;

    t += dt;
  }
  /**
  * @brief Update the estimated state based on measured values,
  * using the given time step and dynamics matrix.
  */
  void update(const Eigen::VectorXf& y, float dt, const Eigen::MatrixXf A) {

    this->A = A;
    this->dt = dt;
    update(y);
  }
  /**
  * @brief Return the current state and time.
  */
  Eigen::VectorXf state() { return x_hat; };
  float time() { return t; };

  void setState(const Eigen::VectorXf& x_hat_1) { x_hat = x_hat_1; }
};
