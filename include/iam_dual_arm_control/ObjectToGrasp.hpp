#pragma once

#include <deque>
#include <vector>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "sg_filter.h"

#include "iam_dual_arm_control/tools/Utils.hpp"

#define NB_ROBOTS 2                // Number of robots
#define NB_FT_SENSOR_SAMPLES 50    // Number of FT sensors' samples used for initial calibration (compute the offsets)
#define MOVING_FORCE_WINDOW_SIZE 10// Window's size used to average the force data and detect peristent contact
#define NB_OBJECTS 3               // Number of objects

typedef Eigen::Matrix<float, 6, 1> Vector6f;

class ObjectToGrasp {

private:
  float objectMass_;
  Eigen::Vector3f objectDim_;// Object dimensions [m] (3x1)

  Eigen::Vector3f xo_;
  Eigen::Vector3f xDo_;
  Eigen::Vector4f qo_;
  Eigen::Vector4f qDo_;
  Eigen::Matrix4f wHo_;
  Eigen::Matrix4f wHDo_;
  Eigen::Matrix4f wHDgp_[NB_ROBOTS];

  Eigen::Vector3f xGpO_[NB_ROBOTS];
  Eigen::Vector4f qGpO_[NB_ROBOTS];
  Eigen::Matrix4f wHGp_[NB_ROBOTS];

  Eigen::Vector3f vo_;
  Eigen::Vector3f wo_;
  Eigen::Vector3f xPickup_;

  std::unique_ptr<SGF::SavitzkyGolayFilter> xoFiltered_;
  std::unique_ptr<SGF::SavitzkyGolayFilter> qoFiltered_;

  Eigen::Vector3f normalVectSurfObj_[NB_ROBOTS];// Normal vector to surface object for each robot (3x1)
  Vector6f vGpO_[NB_ROBOTS];

public:
  ObjectToGrasp(){};
  ~ObjectToGrasp(){};

  void init(int sgfP[], int sgfO[], float dt, Eigen::Matrix3f oRGpLeft, Eigen::Matrix3f oRGpRight) {
    vo_.setZero();
    wo_.setZero();
    xo_.setZero();
    xDo_.setZero();
    xPickup_.setZero();

    qo_ << 1.0f, 0.0f, 0.0f, 0.0f;
    qDo_ << 1.0f, 0.0f, 0.0f, 0.0f;
    wHo_ = Utils<float>::pose2HomoMx(xo_, qo_);
    wHDo_ = Utils<float>::pose2HomoMx(xDo_, qDo_);
    qGpO_[0] = Utils<float>::rotationMatrixToQuaternion(oRGpLeft);
    qGpO_[1] = Utils<float>::rotationMatrixToQuaternion(oRGpRight);

    // Normal to contact surfaces
    normalVectSurfObj_[0] = oRGpLeft.col(2);
    normalVectSurfObj_[1] = oRGpRight.col(2);
    vGpO_[0].setZero();
    vGpO_[1].setZero();

    xoFiltered_ = std::make_unique<SGF::SavitzkyGolayFilter>(sgfP[0], sgfP[1], sgfP[2], dt);
    qoFiltered_ = std::make_unique<SGF::SavitzkyGolayFilter>(sgfO[0], sgfO[1], sgfO[2], dt);

    this->getDesiredHmgTransform();
  }

  void computeHmgTransform() { wHo_ = Utils<float>::pose2HomoMx(xo_, qo_); }

  void getDesiredHmgTransform() { wHDo_ = Utils<float>::pose2HomoMx(xDo_, qDo_); }

  void getGraspPointHTransform() {
    wHGp_[0] = wHo_ * Utils<float>::pose2HomoMx(xGpO_[0], qGpO_[0]);
    wHGp_[1] = wHo_ * Utils<float>::pose2HomoMx(xGpO_[1], qGpO_[1]);
  }

  void getGraspPointDesiredHTransform() {
    wHDgp_[0] = wHDo_ * Utils<float>::pose2HomoMx(xGpO_[0], qGpO_[0]);
    wHDgp_[1] = wHDo_ * Utils<float>::pose2HomoMx(xGpO_[1], qGpO_[1]);
  }

  void updateGraspNormals() {
    normalVectSurfObj_[0] = wHGp_[0].block(0, 0, 3, 3).col(2);
    normalVectSurfObj_[1] = wHGp_[1].block(0, 0, 3, 3).col(2);
  }

  void getGraspPointVelocity() {
    //Velocity of grasp points on the object
    for (int i = 0; i < NB_ROBOTS; i++) {
      Eigen::Vector3f t = wHo_.block<3, 3>(0, 0) * xGpO_[i];
      Eigen::Matrix3f skewMxGpo;
      skewMxGpo << 0.0f, -t(2), t(1), t(2), 0.0f, -t(0), -t(1), t(0), 0.0f;

      // Velocity
      vGpO_[i].head(3) = vo_ - 0 * skewMxGpo * wo_;
      vGpO_[i].tail(3) = 0 * wo_;

      vGpO_[i].head(3) *= 0.0f;
      vGpO_[i].tail(3) *= 0.0f;
    }
  }

  float getObjectMass() { return objectMass_; }
  float getObjectDimSpecific(int dim) { return objectDim_[dim]; }
  float getXoSpecific(int dim) { return xo_[dim]; }
  Eigen::Vector3f getObjectDim() { return objectDim_; }
  Eigen::Vector3f getXo() { return xo_; }
  Eigen::Vector3f getXGpO(int robotID) { return xGpO_[robotID]; }
  Eigen::Vector3f getVo() { return vo_; }
  Eigen::Vector3f getWo() { return wo_; }
  Eigen::Vector3f getXPickup() { return xPickup_; }
  Eigen::Vector3f getNormalVectSurfObjSpecific(int robotID) { return normalVectSurfObj_[robotID]; }
  Eigen::Vector3f* getNormalVectSurfObj() { return normalVectSurfObj_; }
  Eigen::Vector4f getQo() { return qo_; }
  Eigen::Vector4f getQGpO(int robotID) { return qGpO_[robotID]; }
  Vector6f* getVGpO() { return vGpO_; }
  Eigen::Matrix4f getWHo() { return wHo_; }
  Eigen::Matrix4f getWHDo() { return wHDo_; }
  Eigen::Matrix4f* getWHGp() { return wHGp_; }
  Eigen::Matrix4f getWHGpSpecific(int robotID) { return wHGp_[robotID]; }
  Eigen::Matrix4f* getWHDgp() { return wHDgp_; }
  Eigen::Matrix4f getWHDgpSpecific(int robotID) { return wHDgp_[robotID]; }

  void setObjectMass(float objectMass) { objectMass_ = objectMass; }
  void setObjectDim(Eigen::Vector3f objectDim) { objectDim_ = objectDim; }
  void setXo(Eigen::Vector3f xo) { xo_ = xo; }
  void setXDo(Eigen::Vector3f xDo) { xDo_ = xDo; }
  void setXPickup(Eigen::Vector3f xPickup) { xPickup_ = xPickup; }
  void setXGpO(Eigen::Vector3f xGpO, int robotID) { xGpO_[robotID] = xGpO; }
  void setQo(Eigen::Vector4f qo) { qo_ = qo; }
  void setQDo(Eigen::Vector4f qDo) { qDo_ = qDo; }
  void setWHo(Eigen::Matrix4f wHo) { wHo_ = wHo; }
  void setWHDo(Eigen::Matrix4f wHDo) { wHDo_ = wHDo; }
  void setWHDgp(Eigen::Matrix4f wHDgp, int robotID) { wHDgp_[robotID] = wHDgp; }
  void setVGpO(Vector6f vGpO, int robotID) { vGpO_[robotID] = vGpO; }
};
