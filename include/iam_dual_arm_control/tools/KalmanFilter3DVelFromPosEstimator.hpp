
#pragma once

#include "Eigen/Eigen"
#include "iam_dual_arm_control/tools/KalmanFilter.hpp"

#include <fstream>

class KalmanFilter3DVelFromPosEstimator {
private:
  std::unique_ptr<KalmanFilter> kFilter_[3];

public:
  KalmanFilter3DVelFromPosEstimator(){};
  ~KalmanFilter3DVelFromPosEstimator(){};

  void init(float dt, Eigen::Vector2f Q, float R, Eigen::Vector3f Xm) {

    int n = 2;// Number of states
    int m = 1;// Number of measurements

    float sigmaQ1 = Q(0);
    float sigmaQ2 = Q(1);
    float sigmaR = R;

    Eigen::MatrixXf A2(n, n);         // System dynamics matrix
    Eigen::MatrixXf C2(m, n);         // Output matrix
    Eigen::MatrixXf Q2(n, n);         // Process noise covariance
    Eigen::MatrixXf R2(m, m);         // Measurement noise covariance
    Eigen::MatrixXf P2(n, n);         // Estimate error covariance
    Eigen::Matrix<float, 2, 1> xK0[3];// state vector Y_k0(2), Z_k0(2);

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
      kFilter_[i] = std::make_unique<KalmanFilter>(dt, A2, C2, Q2, R2, P2);
      xK0[i](0) = Xm(i);
      xK0[i](1) = 0.0;
      kFilter_[i]->init(0., xK0[i]);
    }
  }

  /**
	  * Update the estimated state based on measured values. The
	  * time step is assumed to remain constant.
	  */
  void update(const Eigen::Vector3f& Xm) {

    Eigen::Matrix<float, 1, 1> xK[3];

    for (int i = 0; i < 3; i++) {
      xK[i](0) = Xm(i);
      kFilter_[i]->update(xK[i]);
    }
  }

  Eigen::Vector3f getEstimatePosition() {

    Eigen::Vector3f xHat;
    for (int i = 0; i < 3; i++) { xHat(i) = (kFilter_[i]->state())(0); }
    return xHat;
  }

  Eigen::Vector3f getEstimateVelocity() {
    Eigen::Vector3f dXHat;
    for (int i = 0; i < 3; i++) { dXHat(i) = (kFilter_[i]->state())(1); }
    return dXHat;
  }
};
