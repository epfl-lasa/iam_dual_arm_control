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
#include <float.h>
#include <fstream>

template<typename T = float>
class Utils {
public:
  enum ROBOT_ID { KUKA_LWR, FRANKA_PANDA };

  enum DH_CONVENTION { NORMAL, MODIFIED };

  // Class constructor
  Utils() {}

  static Eigen::Matrix<T, 4, 1> quaternionProduct(Eigen::Matrix<T, 4, 1> q1, Eigen::Matrix<T, 4, 1> q2) {
    Eigen::Matrix<T, 4, 1> q;
    q(0) = q1(0) * q2(0) - (q1.segment(1, 3)).dot(q2.segment(1, 3));
    Eigen::Matrix<T, 3, 1> q1Im = (q1.segment(1, 3));
    Eigen::Matrix<T, 3, 1> q2Im = (q2.segment(1, 3));
    q.segment(1, 3) = q1(0) * q2Im + q2(0) * q1Im + q1Im.cross(q2Im);

    return q;
  }

  static Eigen::Matrix<T, 3, 3> getSkewSymmetricMatrix(Eigen::Matrix<T, 3, 1> input) {
    Eigen::Matrix<T, 3, 3> output;

    output << 0.0f, -input(2), input(1), input(2), 0.0f, -input(0), -input(1), input(0), 0.0f;

    return output;
  }

  static Eigen::Matrix<T, 3, 3> eulerAnglesToRotationMatrix(T phi, T theta, T psi) {
    T cphi = std::cos(phi);
    T sphi = std::sin(phi);
    T ctheta = std::cos(theta);
    T stheta = std::sin(theta);
    T cpsi = std::cos(psi);
    T spsi = std::sin(psi);

    Eigen::Matrix<T, 3, 3> R;
    R << cpsi * ctheta, cpsi * stheta * sphi - spsi * cphi, cpsi * stheta * cphi + spsi * sphi, spsi * ctheta,
        spsi * stheta * sphi + cpsi * cphi, spsi * stheta * cphi - cpsi * sphi, -stheta, ctheta * sphi, ctheta * cphi;

    return R;
  }

  static Eigen::Matrix<T, 4, 1> rotationMatrixToQuaternion(Eigen::Matrix<T, 3, 3> R) {
    Eigen::Matrix<T, 4, 1> q;

    float r11 = R(0, 0);
    float r12 = R(0, 1);
    float r13 = R(0, 2);
    float r21 = R(1, 0);
    float r22 = R(1, 1);
    float r23 = R(1, 2);
    float r31 = R(2, 0);
    float r32 = R(2, 1);
    float r33 = R(2, 2);

    float tr = r11 + r22 + r33;
    float tr1 = r11 - r22 - r33;
    float tr2 = -r11 + r22 - r33;
    float tr3 = -r11 - r22 + r33;

    if (tr > 0) {
      q(0) = sqrt(1.0f + tr) / 2.0f;
      q(1) = (r32 - r23) / (4.0f * q(0));
      q(2) = (r13 - r31) / (4.0f * q(0));
      q(3) = (r21 - r12) / (4.0f * q(0));
    } else if ((tr1 > tr2) && (tr1 > tr3)) {
      q(1) = sqrt(1.0f + tr1) / 2.0f;
      q(0) = (r32 - r23) / (4.0f * q(1));
      q(2) = (r21 + r12) / (4.0f * q(1));
      q(3) = (r31 + r13) / (4.0f * q(1));
    } else if ((tr2 > tr1) && (tr2 > tr3)) {
      q(2) = sqrt(1.0f + tr2) / 2.0f;
      q(0) = (r13 - r31) / (4.0f * q(2));
      q(1) = (r21 + r12) / (4.0f * q(2));
      q(3) = (r32 + r23) / (4.0f * q(2));
    } else {
      q(3) = sqrt(1.0f + tr3) / 2.0f;
      q(0) = (r21 - r12) / (4.0f * q(3));
      q(1) = (r31 + r13) / (4.0f * q(3));
      q(2) = (r32 + r23) / (4.0f * q(3));
    }

    return q;
  }

  static Eigen::Matrix<T, 3, 3> quaternionToRotationMatrix(Eigen::Matrix<T, 4, 1> q) {
    Eigen::Matrix<T, 3, 3> R;

    T q0 = q(0);
    T q1 = q(1);
    T q2 = q(2);
    T q3 = q(3);

    R(0, 0) = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
    R(1, 0) = 2.0f * (q1 * q2 + q0 * q3);
    R(2, 0) = 2.0f * (q1 * q3 - q0 * q2);

    R(0, 1) = 2.0f * (q1 * q2 - q0 * q3);
    R(1, 1) = q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3;
    R(2, 1) = 2.0f * (q2 * q3 + q0 * q1);

    R(0, 2) = 2.0f * (q1 * q3 + q0 * q2);
    R(1, 2) = 2.0f * (q2 * q3 - q0 * q1);
    R(2, 2) = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    return R;
  }

  static void quaternionToAxisAngle(Eigen::Matrix<T, 4, 1> q, Eigen::Matrix<T, 3, 1>& axis, T& angle) {
    if ((q.segment(1, 3)).norm() < 1e-3f) {
      axis = q.segment(1, 3);
    } else {
      axis = q.segment(1, 3) / (q.segment(1, 3)).norm();
    }

    angle = 2 * std::acos(q(0));
  }

  static Eigen::Matrix<T, 4, 1> axisAngleToQuaterion(Eigen::Matrix<T, 3, 1> axis, T angle) {
    Eigen::Matrix<T, 4, 1> q;
    q(0) = std::cos(angle / 2);
    q(1) = axis(0) * std::sin(angle / 2);
    q(2) = axis(1) * std::sin(angle / 2);
    q(3) = axis(2) * std::sin(angle / 2);
    return q;
  }

  static Eigen::Matrix<T, 4, 1> slerpQuaternion(Eigen::Matrix<T, 4, 1> q1, Eigen::Matrix<T, 4, 1> q2, T t) {

    Eigen::Matrix<T, 4, 1> q;

    // Change sign of q2 if dot product of the two quaterion is negative => allows to interpolate along the shortest path
    if (q1.dot(q2) < 0.0f) { q2 = -q2; }

    T dotProduct = q1.dot(q2);
    if (dotProduct > 1.0f) {
      dotProduct = 1.0f;
    } else if (dotProduct < -1.0f) {
      dotProduct = -1.0f;
    }

    T omega = acos(dotProduct);

    if (std::fabs(omega) < FLT_EPSILON) {
      q = q1.transpose() + t * (q2 - q1).transpose();
    } else {
      q = (std::sin((1 - t) * omega) * q1 + std::sin(t * omega) * q2) / std::sin(omega);
    }

    return q;
  }

  static Eigen::Matrix<T, 4, 1>
  slerpQuaternion(Eigen::Matrix<T, 4, 1>* q, Eigen::Matrix<T, Eigen::Dynamic, 1> t, int size) {

    if (size == 1) {
      return q[0];
    } else {
      T sum = 0.0f;
      for (int k = 0; k < size; k++) { sum += t(k); }
      if (sum < FLT_EPSILON) {
        return slerpQuaternion(slerpQuaternion(q, t, size - 1), q[size - 1], 0.0f);
      } else {
        return slerpQuaternion(slerpQuaternion(q, t, size - 1), q[size - 1], t(size - 1) / sum);
      }
    }
  }

  static Eigen::Matrix<T, 3, 3> rodriguesRotation(Eigen::Matrix<T, 3, 1> v1, Eigen::Matrix<T, 3, 1> v2) {
    // Compute rotation error between current orientation and plane orientation using Rodrigues' law
    v1.normalize();
    v2.normalize();

    Eigen::Matrix<T, 3, 1> w;
    w = v1.cross(v2);
    float c = v1.dot(v2);
    float s = w.norm();
    w /= s;

    Eigen::Matrix<T, 3, 3> K;
    K << getSkewSymmetricMatrix(w);

    Eigen::Matrix<T, 3, 3> Re;
    if (fabs(s) < FLT_EPSILON) {
      Re = Eigen::Matrix<T, 3, 3>::Identity();
    } else {
      Re = Eigen::Matrix<T, 3, 3>::Identity() + s * K + (1 - c) * K * K;
    }

    return Re;
  }

  static Eigen::Matrix<T, 3, 1>
  quaternionToAngularVelocity(Eigen::Matrix<T, 4, 1> q1, Eigen::Matrix<T, 4, 1> q2, T gain = 1.0f) {
    Eigen::Matrix<T, 4, 1> q1I, wq;
    q1I(0) = q1(0);
    q1I.segment(1, 3) = -q1.segment(1, 3);
    wq = 2.0f * gain * quaternionProduct(q2 - q1, q1I);

    return wq.segment(1, 3);
  }

  static Eigen::Matrix<T, 3, 3> orthogonalProjector(Eigen::Matrix<T, 3, 1> v) {
    return Eigen::Matrix<T, 3, 3>::Identity() - v * v.transpose();
  }

  static T smoothRise(T x, T a, T b) {
    T y;
    if (x < a) {
      y = 0.0f;
    } else if (x > b) {
      y = 1.0f;
    } else {
      y = (1.0f + sin(M_PI * (x - a) / (b - a) - M_PI / 2.0f)) / 2.0f;
    }

    return y;
  }

  static T smoothFall(T x, T a, T b) { return 1.0f - smoothRise(x, a, b); }

  static T smoothRiseFall(T x, T a, T b, T c, T d) { return smoothRise(x, a, b) * smoothFall(x, c, d); }

  static T deadZone(T x, T a, T b) {
    if (x < b && x > a) {
      return 0.0f;
    } else {
      return x;
    }
  }

  static Eigen::Matrix<T, Eigen::Dynamic, 1> deadZone(Eigen::Matrix<T, Eigen::Dynamic, 1> x, T limit) {
    T norm = x.norm();

    if (norm > limit) {
      return x;
    } else {
      return Eigen::Matrix<T, Eigen::Dynamic, 1>::Zero(x.size());
    }
  }

  static T wrapToZero(T x, T a, T b) {
    if (x < b && x > a) {
      return x;
    } else {
      return 0.0f;
    }
  }

  static T bound(T x, T a, T b) {
    if (x > b) {
      return b;
    } else if (x < a) {
      return a;
    } else {
      return x;
    }
  }

  static Eigen::Matrix<T, Eigen::Dynamic, 1> bound(Eigen::Matrix<T, Eigen::Dynamic, 1> x, T limit) {
    T norm = x.norm();

    if (norm > limit) {
      return x * limit / norm;
    } else {
      return x;
    }
  }

  static Eigen::Matrix<T, 4, 4> getDHMatrix(T a, T alpha, T d, T theta, DH_CONVENTION dhConvention = NORMAL) {
    Eigen::Matrix<T, 4, 4> H;

    if (dhConvention == NORMAL) {
      H(0, 0) = std::cos(theta);
      H(0, 1) = -std::cos(alpha) * std::sin(theta);
      H(0, 2) = std::sin(alpha) * std::sin(theta);
      H(0, 3) = a * std::cos(theta);

      H(1, 0) = std::sin(theta);
      H(1, 1) = std::cos(alpha) * std::cos(theta);
      H(1, 2) = -std::sin(alpha) * std::cos(theta);
      H(1, 3) = a * std::sin(theta);

      H(2, 0) = 0.0f;
      H(2, 1) = std::sin(alpha);
      H(2, 2) = std::cos(alpha);
      H(2, 3) = d;

      H(3, 0) = 0.0f;
      H(3, 1) = 0.0f;
      H(3, 2) = 0.0f;
      H(3, 3) = 1.0f;
    } else {
      H(0, 0) = std::cos(theta);
      H(0, 1) = -std::sin(theta);
      H(0, 2) = 0.0f;
      H(0, 3) = a;

      H(1, 0) = std::sin(theta) * std::cos(alpha);
      H(1, 1) = std::cos(theta) * std::cos(alpha);
      H(1, 2) = -std::sin(alpha);
      H(1, 3) = -d * std::sin(alpha);

      H(2, 0) = std::sin(theta) * std::sin(alpha);
      H(2, 1) = std::cos(theta) * std::sin(alpha);
      H(2, 2) = std::cos(alpha);
      H(2, 3) = d * std::cos(alpha);

      H(3, 0) = 0.0f;
      H(3, 1) = 0.0f;
      H(3, 2) = 0.0f;
      H(3, 3) = 1.0f;
    }

    return H;
  }

  static Eigen::Matrix<T, 4, 4> getForwardKinematics(Eigen::Matrix<T, 7, 1> joints, ROBOT_ID robotID = KUKA_LWR) {
    Eigen::Matrix<T, 4, 4> H, H1, H2, H3, H4, H5, H6, H7, H8;

    if (robotID == KUKA_LWR) {
      H1 = getDHMatrix(0.0f, M_PI / 2.0f, 0.3105f, joints(0));
      H2 = getDHMatrix(0.0f, -M_PI / 2.0f, 0.0f, joints(1));
      H3 = getDHMatrix(0.0f, -M_PI / 2.0f, 0.4f, joints(2));
      H4 = getDHMatrix(0.0f, M_PI / 2.0f, 0.0f, joints(3));
      H5 = getDHMatrix(0.0f, M_PI / 2.0f, 0.39f, joints(4));
      H6 = getDHMatrix(0.0f, -M_PI / 2.0f, 0.0f, joints(5));
      H7 = getDHMatrix(0.0f, 0.0f, 0.078f, joints(6));
      H = H1 * H2 * H3 * H4 * H5 * H6 * H7;
    } else if (robotID == FRANKA_PANDA) {
      H1 = getDHMatrix(0.0f, 0.0f, 0.333f, joints(0), MODIFIED);
      H2 = getDHMatrix(0.0f, -M_PI / 2.0f, 0.0f, joints(1), MODIFIED);
      H3 = getDHMatrix(0.0f, M_PI / 2.0f, 0.316f, joints(2), MODIFIED);
      H4 = getDHMatrix(0.0825f, M_PI / 2.0f, 0.0f, joints(3), MODIFIED);
      H5 = getDHMatrix(-0.0825f, -M_PI / 2.0f, 0.384f, joints(4), MODIFIED);
      H6 = getDHMatrix(0.0f, M_PI / 2.0f, 0.0f, joints(5), MODIFIED);
      H7 = getDHMatrix(0.088f, M_PI / 2.0f, 0.0f, joints(6), MODIFIED);
      H8 = getDHMatrix(0.0f, 0.0f, 0.107f, 0.0f, MODIFIED);
      H = H1 * H2 * H3 * H4 * H5 * H6 * H7 * H8;
    }

    return H;
  }

  static Eigen::Matrix<T, 6, 7> getGeometricJacobian(Eigen::Matrix<T, 7, 1> joints,
                                                     Eigen::Matrix<T, 3, 1> rEEx = Eigen::Matrix<T, 3, 1>::Zero(),
                                                     ROBOT_ID robotID = KUKA_LWR) {
    Eigen::Matrix<T, 4, 4> Hee, H[8], Hk;
    Eigen::Matrix<T, 6, 7> J;

    DH_CONVENTION convention;
    if (robotID == KUKA_LWR) {
      convention = NORMAL;
    } else if (robotID = FRANKA_PANDA) {
      convention = MODIFIED;
    }

    if (robotID == KUKA_LWR) {
      H[0] = getDHMatrix(0.0f, M_PI / 2.0f, 0.3105f, joints(0));
      H[1] = getDHMatrix(0.0f, -M_PI / 2.0f, 0.0f, joints(1));
      H[2] = getDHMatrix(0.0f, -M_PI / 2.0f, 0.4f, joints(2));
      H[3] = getDHMatrix(0.0f, M_PI / 2.0f, 0.0f, joints(3));
      H[4] = getDHMatrix(0.0f, M_PI / 2.0f, 0.39f, joints(4));
      H[5] = getDHMatrix(0.0f, -M_PI / 2.0f, 0.0f, joints(5));
      H[6] = getDHMatrix(0.0f, 0.0f, 0.078f, joints(6));
      Hee = H[0] * H[1] * H[2] * H[3] * H[4] * H[5] * H[6];
    } else if (robotID == FRANKA_PANDA) {
      H[0] = getDHMatrix(0.0f, 0.0f, 0.333f, joints(0), convention);
      H[1] = getDHMatrix(0.0f, -M_PI / 2.0f, 0.0f, joints(1), convention);
      H[2] = getDHMatrix(0.0f, M_PI / 2.0f, 0.316f, joints(2), convention);
      H[3] = getDHMatrix(0.0825f, M_PI / 2.0f, 0.0f, joints(3), convention);
      H[4] = getDHMatrix(-0.0825f, -M_PI / 2.0f, 0.384f, joints(4), convention);
      H[5] = getDHMatrix(0.0f, M_PI / 2.0f, 0.0f, joints(5), convention);
      H[6] = getDHMatrix(0.088f, M_PI / 2.0f, 0.107f, joints(6), convention);
      Hee = H[0] * H[1] * H[2] * H[3] * H[4] * H[5] * H[6];
    }

    Eigen::Matrix<T, 3, 1> xEE, z0, x0, xk, zk;

    xEE = Hee.block(0, 3, 3, 1);

    Hk.setIdentity();

    J.setConstant(0.0f);

    for (int k = 0; k < 7; k++) {

      if (convention == MODIFIED) { Hk = Hk * H[k]; }
      xk = Hk.block(0, 3, 3, 1);
      zk = Hk.block(0, 2, 3, 1);

      J.block(0, k, 3, 1) = zk.cross(xEE - xk);
      J.block(3, k, 3, 1) = zk;
      if (convention == NORMAL) { Hk = Hk * H[k]; }
    }

    J.block(0, 0, 3, 7) += -getSkewSymmetricMatrix(rEEx) * J.block(3, 0, 3, 7);

    return J;
  }

  // //////////////////////////////////////////////////////////////////////////////
  static Eigen::Matrix<T, 4, 4> pose2HomoMx(Eigen::Matrix<T, 3, 1> x, Eigen::Matrix<T, 4, 1> q) {
    Eigen::Matrix<T, 4, 4> H;
    H.setIdentity();
    H.block(0, 3, 3, 1) = x;
    Eigen::Quaternion<T> q_(q(0), q(1), q(2), q(3));
    H.block(0, 0, 3, 3) = q_.toRotationMatrix();

    return H;
  }

  static Eigen::Matrix<T, 3, 3>
  getCombinedRotationMatrix(T weight, Eigen::Matrix<T, 3, 3> wRc, Eigen::Matrix<T, 3, 3> wRd) {
    Eigen::Quaternion<T> qc(wRc);// current
    Eigen::Quaternion<T> qd(wRd);// desired
    Eigen::Quaternion<T> qT = qc.slerp(weight, qd);

    return qT.toRotationMatrix();
  }

  static Eigen::Matrix<T, 4, 1>
  getSlerpInterpolation(T weight, Eigen::Matrix<T, 3, 3> wRc, Eigen::Matrix<T, 3, 3> wRd) {
    Eigen::Quaternion<T> qc(wRc);// current
    Eigen::Quaternion<T> qd(wRd);// desired
    Eigen::Quaternion<T> qT = qc.slerp(weight, qd);

    Eigen::Matrix<T, 4, 1> qOut;
    qOut << qT.w(), qT.x(), qT.y(), qT.z();

    return qOut;
  }

  static Eigen::Matrix<T, 6, 1> getPoseErrorCur2Des(Eigen::Matrix<T, 4, 4> dHc) {
    // Pass
    Eigen::Matrix<T, 6, 1> dEtaC(6);
    dEtaC.segment(0, 3) << dHc(0, 3), dHc(1, 3), dHc(2, 3);

    // Extraction of the rotation
    Eigen::Matrix<T, 3, 3> dRc = dHc.block(0, 0, 3, 3);
    Eigen::AngleAxis<T> dAxisAngleC(dRc);
    Eigen::Matrix<T, 3, 1> dAxisC = dAxisAngleC.axis();
    dEtaC(3) = dAxisC(0) * dAxisAngleC.angle();
    dEtaC(4) = dAxisC(1) * dAxisAngleC.angle();
    dEtaC(5) = dAxisC(2) * dAxisAngleC.angle();

    return dEtaC;
  }

  static Eigen::Matrix<T, 3, 1> getOrientationErrorCur2Des(Eigen::Matrix<T, 3, 3> dRc) {
    // Pass
    Eigen::Matrix<T, 3, 1> dEtaC(3);

    // Extraction of the rotation
    Eigen::AngleAxis<T> dAxisAngleC(dRc);
    Eigen::Matrix<T, 3, 1> dAxisC = dAxisAngleC.axis();
    dEtaC(0) = dAxisC(0) * dAxisAngleC.angle();
    dEtaC(1) = dAxisC(1) * dAxisAngleC.angle();
    dEtaC(2) = dAxisC(2) * dAxisAngleC.angle();

    return dEtaC;
  }

  static Eigen::Matrix<T, 3, 3> getMuThetaJacobian(Eigen::Matrix<T, 3, 3> dRc) {
    // Extraction of the rotation
    Eigen::AngleAxis<T> dAxisAngleC(dRc);

    // function sinc(theta) and sinc(theta/2)
    T sincTheta, sincTheta2;
    sincTheta = sin(dAxisAngleC.angle() + 1e-6) / (dAxisAngleC.angle() + 1e-6);
    sincTheta2 = sin((dAxisAngleC.angle() + 1e-6) / 2.) / ((dAxisAngleC.angle() + 1e-6) / 2.);

    Eigen::Matrix<T, 3, 1> dAxisC = dAxisAngleC.axis();
    Eigen::Matrix<T, 3, 3> skewMu;
    skewMu.setZero(3, 3);

    skewMu << 0.0, -dAxisC(2), dAxisC(1), dAxisC(2), 0.0, -dAxisC(0), -dAxisC(1), dAxisC(0), 0.0;

    // Jacobian of the rotation
    Eigen::Matrix<T, 3, 3> lMuTheta;
    lMuTheta.setIdentity(3, 3);
    lMuTheta =
        lMuTheta - (dAxisAngleC.angle() / 2.) * skewMu + (1. - (sincTheta / pow(sincTheta2, 2.))) * skewMu * skewMu;

    return lMuTheta;
  }

  static Eigen::Matrix<T, 3, 1> saturationVect3(T lim, Eigen::Matrix<T, 3, 1> vel) {
    Eigen::Matrix<T, 3, 1> v = vel;

    if ((fabs(vel(0)) > lim) || (fabs(vel(1)) > lim) || (fabs(vel(2)) > lim)) { v = lim * (1. / vel.norm() * vel); }
    return v;
  }

  static Eigen::Matrix<T, 6, 1> saturationTwist(T limL, T limA, Eigen::Matrix<T, 6, 1> vel) {
    Eigen::Matrix<T, 3, 1> lin, ang;
    lin = vel.head(3);
    ang = vel.tail(3);

    if ((fabs(lin(0)) > limL) || (fabs(lin(1)) > limL) || (fabs(lin(2)) > limL)) {
      lin = limL * (1. / lin.norm() * lin);
    }
    if ((fabs(ang(0)) > limA) || (fabs(ang(1)) > limA) || (fabs(ang(2)) > limA)) {
      ang = limA * (1. / ang.norm() * ang);
    }

    Eigen::Matrix<T, 6, 1> vSat;
    vSat.head(3) = lin;
    vSat.tail(3) = ang;

    return vSat;
  }

  static Eigen::Matrix<T, 12, 12> getBimanualTaskTwistMapInv(T aBiIn, T bBiIn) {

    T aBi = aBiIn;
    T bBi = bBiIn;
    if (aBi != 0.0) bBi = 1.0;
    else if (aBi == 1.0)
      bBi = 0.0;

    Eigen::Matrix<T, 12, 12> cHands;
    cHands.setZero();
    Eigen::Matrix<T, 6, 6> idn;
    idn.setIdentity();
    // Bimanual transformation
    cHands.topLeftCorner(6, 6) = idn;
    cHands.topRightCorner(6, 6) = -(1. - aBi) * idn;
    cHands.bottomLeftCorner(6, 6) = bBi * idn;
    cHands.bottomRightCorner(6, 6) = aBi * idn;

    return cHands;
  }

  static void getBimanualTransforms(Eigen::Matrix<T, 4, 4> wHl,
                                    Eigen::Matrix<T, 4, 4> wHr,
                                    Eigen::Matrix<T, 4, 4>& wHa,
                                    Eigen::Matrix<T, 4, 4>& lHr) {
    wHa.setIdentity(4, 4);
    lHr.setIdentity(4, 4);

    // Relative transformation
    // ========================

    // translation expresse wrt. the world
    lHr.block(0, 3, 3, 1) = wHr.block(0, 3, 3, 1) - wHl.block(0, 3, 3, 1);
    // orientation wrt. the left hand
    lHr.block(0, 0, 3, 3) = wHl.block(0, 0, 3, 3).transpose() * wHr.block(0, 0, 3, 3);

    // Find the abolute transformation
    // ======================================================
    // Axis angle of relative hands orientation
    Eigen::Matrix<T, 3, 3> lRr = lHr.block(0, 0, 3, 3);
    Eigen::AngleAxis<T> lOrientationR(lRr);
    // Average orientation between hands
    Eigen::Matrix<T, 3, 1> axis = lOrientationR.axis();
    T theta = 0.5 * lOrientationR.angle();
    Eigen::AngleAxis<T> avRot(theta, axis);

    // Rotation matrix of the absolute hand frame expressed in the asbolute foot frame
    Eigen::Matrix<T, 3, 3> wRa = wHl.block(0, 0, 3, 3) * avRot.toRotationMatrix();
    wHa.block(0, 3, 3, 1) = 0.5 * (wHl.block(0, 3, 3, 1) + wHr.block(0, 3, 3, 1));

    Eigen::Matrix<T, 3, 3> wRl = wHl.block(0, 0, 3, 3);
    Eigen::Matrix<T, 3, 3> wRr = wHr.block(0, 0, 3, 3);
    Eigen::Quaternion<T> qc(wRl);// current
    Eigen::Quaternion<T> qd(wRr);// desired
    T weight = 0.5;
    Eigen::Quaternion<T> qT = qc.slerp(weight, qd);
    Eigen::Matrix<T, 3, 3> wRCdT = qT.toRotationMatrix();

    wHa.block(0, 0, 3, 3) = wRCdT;
  }

  static void getBimanualTwistDistribution(T aBiIn,
                                           T bBiIn,
                                           Eigen::Matrix<T, 6, 1> velA,
                                           Eigen::Matrix<T, 6, 1> velR,
                                           Eigen::Matrix<T, 6, 1>& leftV,
                                           Eigen::Matrix<T, 6, 1>& rightV) {
    Eigen::Matrix<T, 12, 12> Th = Utils<T>::getBimanualTaskTwistMapInv(aBiIn, bBiIn);

    leftV = Th.topLeftCorner(6, 6) * velA + Th.topRightCorner(6, 6) * velR;

    rightV = Th.bottomLeftCorner(6, 6) * velA + Th.bottomRightCorner(6, 6) * velR;
  }

  static Eigen::Matrix<T, 3, 1> getEulerAnglesXYZ_FixedFrame(Eigen::Matrix<T, 3, 3> R) {
    // This function computed for a given rotation matrix the rotation angles around X, Y and Z axis considered as fixed.
    // the rotation matrix is assumed to be a Euler rotation matrix of type ZYX
    Eigen::Matrix<T, 3, 1> angles;
    T psiX, thetaY, phiZ;
    psiX = std::atan2(R(2, 1), R(2, 2));
    thetaY = std::atan2(-R(2, 0), fabs(std::sqrt(std::pow(R(0, 0), 2.) + std::pow(R(1, 0), 2.))));
    phiZ = std::atan2(R(1, 0), R(0, 0));
    if ((thetaY > M_PI / 2.) || (thetaY < -M_PI / 2.)) {
      psiX = std::atan2(-R(2, 1), -R(2, 2));
      thetaY = std::atan2(-R(2, 0), -fabs(std::sqrt(std::pow(R(0, 0), 2.) + std::pow(R(1, 0), 2.))));
      phiZ = std::atan2(-R(1, 0), -R(0, 0));
    }
    angles(0) = psiX;
    angles(1) = thetaY;
    angles(2) = phiZ;

    return angles;
  }

  static T computeCouplingFactor(Eigen::Matrix<T, 3, 1> ep, T alpha, T beta, T gamma, bool secondOrder) {
    T tCpl = 1.0 / (alpha * ep.norm() + 1e-15);
    T cpl = 0.0;
    tCpl = pow(tCpl, gamma);
    if (secondOrder) cpl = 1.0 - exp(-tCpl / beta) * (1.0 + tCpl / beta);// 2nd order critically damped
    else
      cpl = 1.0 - exp(-tCpl / beta);// 1st order increase

    return cpl;
  }

  static Eigen::Matrix<T, 3, 3> create3dOrthonormalMatrixFromVector(Eigen::Matrix<T, 3, 1> inVec) {

    int n = inVec.rows();
    Eigen::Matrix<T, 3, 3> basis;
    basis.setRandom(3, 3);
    basis.col(0) = 1. / inVec.norm() * inVec;

    assert(basis.rows() == basis.cols());
    uint dim = basis.rows();
    basis.col(0).normalize();
    for (uint i = 1; i < dim; i++) {
      for (uint j = 0; j < i; j++) basis.col(i) -= basis.col(j).dot(basis.col(i)) * basis.col(j);
      basis.col(i).normalize();
    }

    if (basis.rows() == 3) {
      Eigen::Matrix<T, 3, 1> u = basis.col(0);
      Eigen::Matrix<T, 3, 1> v = basis.col(1);
      Eigen::Matrix<T, 3, 1> w = u.cross(v);
      basis.col(2) = w;
    }
    return basis;
  }

  static void Orthobasis(Eigen::Matrix<T, 3, 1> v1,
                         Eigen::Matrix<T, 3, 1> v0,
                         Eigen::Matrix<T, 3, 3>& R1,
                         Eigen::Matrix<T, 3, 3>& R0) {
    Eigen::Matrix<T, 3, 1> i = v1;
    Eigen::Matrix<T, 3, 1> id = v0;

    i.normalize();
    id.normalize();
    Eigen::Matrix<T, 3, 1> j = i.cross(id);

    if (j.norm() < 1e-6) {
      Eigen::Matrix<T, 3, 3> Rq = Utils<T>::create3dOrthonormalMatrixFromVector(i);
      j = Rq.col(1);
    }

    j.normalize();
    Eigen::Matrix<T, 3, 1> jd = j;
    Eigen::Matrix<T, 3, 1> k = i.cross(j);
    Eigen::Matrix<T, 3, 1> kd = id.cross(jd);

    R1.col(0) = i;
    R1.col(1) = j;
    R1.col(2) = k;

    R0.col(0) = id;
    R0.col(1) = jd;
    R0.col(2) = kd;
  }

  static void updatePoseFromVelocityTwist(T dt, Eigen::Matrix<T, 6, 1> inVeloTwist, Eigen::Matrix<T, 4, 4>& HmgTrsf) {

    Eigen::Matrix<T, 3, 1> pos = HmgTrsf.block(0, 3, 3, 1);
    Eigen::Matrix<T, 3, 3> rot = HmgTrsf.block(0, 0, 3, 3);

    // update position
    pos = pos + dt * inVeloTwist.head(3);

    // update orientation
    Eigen::Quaternion<T> q(rot);

    Eigen::Matrix<T, 4, 3> TrfQuat;
    TrfQuat << -q.x(), -q.y(), -q.z(), q.w(), q.z(), -q.y(), -q.z(), q.w(), q.x(), q.y(), -q.x(), q.w();
    // update the quaternion
    Eigen::Matrix<T, 4, 1> qcoeff;
    qcoeff << q.w(), q.x(), q.y(), q.z();
    qcoeff = qcoeff + dt * 0.5 * TrfQuat * inVeloTwist.tail(3);

    // normalizing the quaternion
    qcoeff.normalize();
    if (qcoeff.norm() <= 1e-8) { qcoeff = Eigen::Matrix<T, 4, 1>(1.0, 0.0, 0.0, 0.0); }

    Eigen::Quaternion<T> qNew(qcoeff(0), qcoeff(1), qcoeff(2), qcoeff(3));// w, x, y, z

    HmgTrsf.setZero();

    HmgTrsf.block(0, 0, 3, 3) = qNew.toRotationMatrix();
    HmgTrsf.block(0, 3, 3, 1) = pos;
    HmgTrsf(3, 3) = 1.0;
  }

  static void updatePoseFromVelocityTwist(T dt,
                                          Eigen::Matrix<T, 6, 1> inVeloTwist,
                                          Eigen::Matrix<T, 3, 1>& curPos,
                                          Eigen::Matrix<T, 4, 1>& curOrient) {

    Eigen::Matrix<T, 4, 4> cur_Hmg_Trsf = Utils<T>::pose2HomoMx(curPos, curOrient);

    // update position
    curPos = curPos + dt * inVeloTwist.head(3);

    // update orientation
    Eigen::Matrix<T, 4, 3> TrfQuat;
    TrfQuat << -curOrient(1), -curOrient(2), -curOrient(3), curOrient(0), curOrient(3), -curOrient(2), -curOrient(3),
        curOrient(0), curOrient(1), curOrient(2), -curOrient(1), curOrient(0);
    // update the quaternion
    curOrient = curOrient + dt * 0.5 * TrfQuat * inVeloTwist.tail(3);

    // normalizing the quaternion
    curOrient.normalize();
    if (curOrient.norm() <= 1e-8) { curOrient = Eigen::Matrix<T, 4, 1>(1.0, 0.0, 0.0, 0.0); }
  }

  static Eigen::Matrix<T, 3, 1> cartesian2planar(Eigen::Matrix<T, 3, 1> posD) {

    Eigen::Matrix<T, 3, 1> out;
    out.setZero();
    out(0) = posD.head(2).norm();         // r
    out(1) = posD(2);                     // z
    out(2) = std::atan2(posD(1), posD(0));// phi
    return out;
  }

  static Eigen::Matrix<T, 3, 1> cartesian2spherical(Eigen::Matrix<T, 3, 1> posD) {
    //
    Eigen::Matrix<T, 3, 1> out;
    out.setZero();
    out(0) = posD.norm();                             // r
    out(1) = std::atan2(posD.head(2).norm(), posD(2));// theta
    out(2) = std::atan2(posD(1), posD(0));            // phi

    return out;
  }

  static Eigen::Matrix<T, 3, 1> getAbs3D(Eigen::Matrix<T, 3, 1> vLeft, Eigen::Matrix<T, 3, 1> vRight) {
    return 0.5 * (vLeft + vRight);
  }

  static Eigen::Matrix<T, 3, 1> getAbs3D(Eigen::Matrix<T, 4, 4> hLeft, Eigen::Matrix<T, 4, 4> hRight) {
    return 0.5 * (hLeft.block(0, 3, 3, 1) + hRight.block(0, 3, 3, 1));
  }

  static Eigen::Matrix<T, 3, 1> getAbs3D(Eigen::Matrix<T, 3, 1> Vec[2]) { return 0.5 * (Vec[0] + Vec[1]); }

  static Eigen::Matrix<T, 3, 1> getAbs3D(Eigen::Matrix<T, 6, 1> Vec[2], bool top) {

    if (!top) {
      return 0.5 * (Vec[0].tail(3) + Vec[1].tail(3));
    } else {
      return 0.5 * (Vec[0].head(3) + Vec[1].head(3));
    }
  }

  static Eigen::Matrix<T, 3, 1> getAbs3D(Eigen::Matrix<T, 4, 4> H[2]) {
    return 0.5 * (H[0].block(0, 3, 3, 1) + H[1].block(0, 3, 3, 1));
  }
};

template class Utils<float>;
template class Utils<double>;
