
#pragma once

#include "Eigen/Eigen"
#include "iam_dual_arm_control/tools/KalmanFilter.hpp"

#include <fstream>

class KalmanFilter3DVelFromPosEstimator {
public:
  KalmanFilter3DVelFromPosEstimator(){};
  ~KalmanFilter3DVelFromPosEstimator(){};

  std::unique_ptr<KalmanFilter> _KFilter[3];

  void init(float dt, Eigen::Vector2f Q, float R, Eigen::Vector3f Xm) {

    int n = 2;// Number of states
    int m = 1;// Number of measurements

    float sigmaQ1 = Q(0);//0.004;  // 0.05
    float sigmaQ2 = Q(1);//0.07;
    float sigmaR = R;    //0.07;  // 2.00

    Eigen::MatrixXf A2(n, n);          // System dynamics matrix
    Eigen::MatrixXf C2(m, n);          // Output matrix
    Eigen::MatrixXf Q2(n, n);          // Process noise covariance
    Eigen::MatrixXf R2(m, m);          // Measurement noise covariance
    Eigen::MatrixXf P2(n, n);          // Estimate error covariance
    Eigen::Matrix<float, 2, 1> X_k0[3];// state vector Y_k0(2), Z_k0(2);

    A2 << 1, dt, 0, 1;
    C2 << 1, 0;
    // Reasonable covariance matrices
    Q2(0, 0) = sigmaQ1 * sigmaQ1 * dt * dt * dt / 3.;
    Q2(0, 1) = sigmaQ1 * sigmaQ2 * dt * dt / 2.;
    Q2(1, 0) = sigmaQ1 * sigmaQ2 * dt * dt / 2.;
    Q2(1, 1) = sigmaQ2 * sigmaQ2 * dt;

    P2(0, 0) = sigmaR * sigmaR;
    P2(0, 1) = sigmaR * sigmaR / (2. * dt);
    P2(1, 0) = sigmaR * sigmaR / (2. * dt);
    P2(1, 1) = 2. / 3. * sigmaQ2 * sigmaQ2 * dt + sigmaR * sigmaR / (2. * dt * dt);

    R2 << sigmaR;

    for (int i = 0; i < 3; i++) {
      _KFilter[i] = std::make_unique<KalmanFilter>(dt, A2, C2, Q2, R2, P2);
      X_k0[i](0) = Xm(i);
      X_k0[i](1) = 0.0;
      _KFilter[i]->init(0., X_k0[i]);
    }
  }

  /**
	  * Update the estimated state based on measured values. The
	  * time step is assumed to remain constant.
	  */
  void update(const Eigen::Vector3f& Xm) {
    //
    Eigen::Matrix<float, 1, 1> X_k[3];
    //
    for (int i = 0; i < 3; i++) {
      X_k[i](0) = Xm(i);
      _KFilter[i]->update(X_k[i]);
    }
  }

  Eigen::Vector3f get_estimate_position() {

    Eigen::Vector3f X_hat;
    for (int i = 0; i < 3; i++) { X_hat(i) = (_KFilter[i]->state())(0); }
    return X_hat;
  }

  Eigen::Vector3f get_estimate_velocity() {
    Eigen::Vector3f dX_hat;
    for (int i = 0; i < 3; i++) { dX_hat(i) = (_KFilter[i]->state())(1); }
    return dX_hat;
  }
};
